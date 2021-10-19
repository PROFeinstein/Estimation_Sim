//
// Created by Jose Dena Ruiz on 04/09/2018.
//

#include "pelican_ctrl_pkg/rotor_velocities.h"

rotor_velocities::rotor_velocities(ros::NodeHandle* nh):n(*nh), ctrl_signals(0,0,0,0),m_kd(1.65e-05),m_kt(1.81e-05),m_len(0.21),on_off(true)
{
    ctrl_signals_sub = n.subscribe("pelican_ctrl/ctrl_inputs",4,&rotor_velocities::get_ctrl_signals, this);
    pelican_onoff_sub = n.subscribe("pelican_ctrl/start_stop_pelican",4,&rotor_velocities::get_on_off_pelican, this);

    rotor_vels_pub = n.advertise<cvg_sim_gazebo_plugins::MotorSpeed>("/asctec_pelican/cmd_rotor_vel",1);

    torque_2_speeds <<              0,  -1/(2*m_kt*m_len),  1/(4*m_kd), 1/(4*m_kt),
                                    0,   1/(2*m_kt*m_len),  1/(4*m_kd), 1/(4*m_kt),
                     1/(2*m_kt*m_len),                  0, -1/(4*m_kd), 1/(4*m_kt),
                    -1/(2*m_kt*m_len),                  0, -1/(4*m_kd), 1/(4*m_kt);
}

rotor_velocities::~rotor_velocities(void){}

void rotor_velocities::get_on_off_pelican(const std_msgs::Bool::ConstPtr& on_off_)
{
    on_off = on_off_->data;
}

void rotor_velocities::get_ctrl_signals(const pelican_ctrl_pkg::ref_values::ConstPtr& ctrl_sign)
{

    ctrl_signals << ctrl_sign->desired_x, ctrl_sign->desired_y, ctrl_sign->desired_yaw, ctrl_sign->desired_z;

    rotor_vels_sqr = torque_2_speeds * ctrl_signals;

    double rotor_front = (rotor_vels_sqr(0) < 1.0) ? 1.0 : sqrt(fabs(rotor_vels_sqr(0)));
    double rotor_rear  = (rotor_vels_sqr(1) < 1.0) ? 1.0 : sqrt(fabs(rotor_vels_sqr(1)));
    double rotor_left  = (rotor_vels_sqr(2) < 1.0) ? 1.0 : sqrt(fabs(rotor_vels_sqr(2)));
    double rotor_right = (rotor_vels_sqr(3) < 1.0) ? 1.0 : sqrt(fabs(rotor_vels_sqr(3)));

    if (on_off)
        {
        rotor_vels_msg.motor_speed.resize(4);
        rotor_vels_msg.motor_speed[0] = (int)rotor_front;
        rotor_vels_msg.motor_speed[1] = (int)rotor_rear;
        rotor_vels_msg.motor_speed[2] = (int)rotor_left;
        rotor_vels_msg.motor_speed[3] = (int)rotor_right;
        }
    else
        {
        rotor_vels_msg.motor_speed.resize(4);
        rotor_vels_msg.motor_speed[0] = 0;
        rotor_vels_msg.motor_speed[1] = 0;
        rotor_vels_msg.motor_speed[2] = 0;
        rotor_vels_msg.motor_speed[3] = 0;
        }


    rotor_vels_pub.publish(rotor_vels_msg);
    //ROS_INFO_STREAM(rotor_vels_msg);
}
