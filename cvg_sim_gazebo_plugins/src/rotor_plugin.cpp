//
// Created by Jose Dena Ruiz on 11/09/2018.
//

#include <rotor/rotor_plugin.h>

namespace gazebo{

    rotor_plugin::rotor_plugin():rotor_velocity_slowdown_sim_(10), ref_motor_rot_vel_(0.0)
    {
        cmd_sub_       = node_handle_.subscribe("asctec_pelican/cmd_rotor_vel", 4 ,&rotor_plugin::VelocityCallback, this);
    }

    rotor_plugin::~rotor_plugin()
    {
        event::Events::DisconnectWorldUpdateBegin(updateConnection_);
        node_handle_.shutdown();
    }

    void rotor_plugin::VelocityCallback(const cvg_sim_gazebo_plugins::MotorSpeedPtr& rot_velocities) 
    {
        ref_motor_rot_vel_ = rot_velocities->motor_speed[motor_number_];
    }

    void rotor_plugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        model_ = _model;
        world_ = model_->GetWorld();
        
        if (_sdf->HasElement("linkName"))
            link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
        else
            gzerr << "[gazebo_motor_model] Please specify a linkName.\n";

        link_ = model_->GetLink(link_name_);

        if (_sdf->HasElement("jointName"))
            joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
        else
            gzerr << "[gazebo_motor_model] Please specify a jointName.\n";

        joint_ = model_->GetJoint(joint_name_);

        if (_sdf->HasElement("parentLinkName"))
            parent_link_name_ = _sdf->GetElement("parentLinkName")->Get<std::string>();
        else
            gzerr << "[gazebo_motor_model] Please specify a parentLinkName.\n";

        parentLink_ = model_->GetLink(parent_link_name_);

        if (_sdf->HasElement("turningDirection"))
        {
            std::string turning_direction = _sdf->GetElement("turningDirection")->Get<std::string>();
            if (turning_direction == "cw")
                turning_direction_ = turning_direction::CW;
            else if (turning_direction == "ccw")
                turning_direction_ = turning_direction::CCW;
            else
                gzerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as turningDirection.\n";
        }
        else
            gzerr << "[gazebo_motor_model] Please specify a turning direction ('cw' or 'ccw').\n";

        if (_sdf->HasElement("motorNumber"))
            motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
        else
            gzerr << "[gazebo_motor_model] Please specify a motorNumber.\n";

        if (_sdf->HasElement("rotorDragCoefficient"))
            rotor_drag_coefficient_ = _sdf->GetElement("rotorDragCoefficient")->Get<double>();
        else
            gzwarn << "[gazebo_motor_model] No rotorDragCoefficient value specified for motor " << motor_number_
                   << " using default value " << rotor_drag_coefficient_ << ".\n";

        if (_sdf->HasElement("rollingMomentCoefficient"))
            rolling_moment_coefficient_ = _sdf->GetElement("rollingMomentCoefficient")->Get<double>();
        else
            gzwarn << "[gazebo_motor_model] No rollingMomentCoefficient value specified for motor " << motor_number_
                   << " using default value " << rolling_moment_coefficient_ << ".\n";

        if (_sdf->HasElement("maxRotVelocity"))
            max_rot_velocity_ = _sdf->GetElement("maxRotVelocity")->Get<double>();
        else
            gzerr << "[gazebo_motor_model] Please specify a maxRotVelocity for the joint.\n";

        if (_sdf->HasElement("timeConstant"))
            time_constant_ = _sdf->GetElement("timeConstant")->Get<double>();
        else
            gzerr << "[gazebo_motor_model] Please specify a timeConstant for the joint.\n";

        if (_sdf->HasElement("motorConstant"))
            motor_constant_ = _sdf->GetElement("motorConstant")->Get<double>();
        else
            gzerr << "[gazebo_motor_model] Please specify a motorConstant for the motor.\n";

        if (_sdf->HasElement("momentConstant"))
            moment_constant_ = _sdf->GetElement("momentConstant")->Get<double>();
        else
            gzerr << "[gazebo_motor_model] Please specify a momentConstant for the motor.\n";

        double inertia_ = link_->GetInertial()->GetIZZ();
        double viscous_friction_coefficient_ = inertia_ / time_constant_;
        double max_force_ = max_rot_velocity_ * viscous_friction_coefficient_;

        // Set the maximumForce on the joint
        joint_->SetVelocityLimit(0, max_force_);

        updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&rotor_plugin::OnUpdate, this, _1));

        motor_pub_topic_ = "asctec_pelican/rotor_vel_" + std::to_string(motor_number_);

        motor_vel_pub_ = node_handle_.advertise<std_msgs::Float32>(motor_pub_topic_,1);

    }

     void rotor_plugin::OnUpdate(const common::UpdateInfo& /*_info*/)
     {

        double motor_rot_vel_ = joint_->GetVelocity(0);
        // TODO: We had to add a factor of 10 here and one in the SetVelocity,
        // because currently gazebo doesn't allow to set the velocity to a higher value than 100.
        double real_motor_velocity = motor_rot_vel_ * rotor_velocity_slowdown_sim_;
        double force = real_motor_velocity * real_motor_velocity * motor_constant_;


        //double ref_motor_rot_vel_ = 100;
        // Apply a force to the link
        link_->AddRelativeForce(math::Vector3(0, 0, force));

        // Forces from Philppe Martin's and Erwan Salaün's
        // 2010 IEEE Conference on Robotics and Automation paper
        // The True Role of Accelerometer Feedback in Quadrotor Control
        // - \omega * \lambda_1 * V_A^{\perp}
        math::Vector3 joint_axis = joint_->GetGlobalAxis(0);
        math::Vector3 body_velocity = link_->GetWorldLinearVel();
        math::Vector3 body_velocity_perpendicular = body_velocity - (body_velocity * joint_axis) * joint_axis;
        math::Vector3 air_drag = -std::abs(real_motor_velocity) * rotor_drag_coefficient_ * body_velocity_perpendicular;
        // Apply air_drag to link
        link_->AddForce(air_drag);
        // Moments
         // Getting the parent link, such that the resulting torques can be applied to it.
        // physics::Link_V parent_links = link_->GetParentJointsLinks();
        //std::cout << "Yaw torque: " << -turning_direction_ * force * moment_constant_ << std::endl;
        parentLink_->AddRelativeTorque(math::Vector3(0, 0, -turning_direction_ * real_motor_velocity * real_motor_velocity * moment_constant_));

       math::Vector3 rolling_moment;
       // - \omega * \mu_1 * V_A^{\perp}
       rolling_moment = -std::abs(real_motor_velocity) * rolling_moment_coefficient_ * body_velocity_perpendicular;
       link_->AddRelativeTorque(rolling_moment);
       
       joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel_ / rotor_velocity_slowdown_sim_);
       //std::cout<< turning_direction_ * ref_motor_rot_vel_ / rotor_velocity_slowdown_sim_ << std::endl;
       //joint_->SetVelocity(0, turning_direction_ * 10);
       turning_velocity_msg_.data = joint_->GetVelocity(0);

       motor_vel_pub_.publish(turning_velocity_msg_);
       //
     }
}
