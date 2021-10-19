//
// Created by Jose Dena Ruiz on 11/09/2018.
//

#ifndef PLUGINS_PELICAN_ROTOR_PLUGIN_H
#define PLUGINS_PELICAN_ROTOR_PLUGIN_H

#include <iostream>
#include <algorithm>
#include <string>
#include <ctime>

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "cvg_sim_gazebo_plugins/MotorSpeed.h"

namespace turning_direction {
const static int CCW = 1;
const static int CW = -1;
};
 
namespace gazebo
{
    class rotor_plugin : public ModelPlugin
            {
    public:

        rotor_plugin();
        virtual ~rotor_plugin(); 

    protected:

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        void OnUpdate(const common::UpdateInfo& /*_info*/); 
        void VelocityCallback(const cvg_sim_gazebo_plugins::MotorSpeedPtr& rot_velocities);

    private:
      ros::NodeHandle node_handle_;
      ros::Publisher motor_vel_pub_;
      ros::Subscriber cmd_sub_;
      std_msgs::Float32 turning_velocity_msg_;

      physics::ModelPtr model_;
      physics::ModelPtr nested_model_;
      physics::JointPtr joint_;
      physics::LinkPtr link_;
      physics::LinkPtr parentLink_;
      physics::Link_V child_links_;
      physics::WorldPtr world_;
      
      std::string model_name_;
      std::string link_name_;
      std::string parent_link_name_;
      std::string joint_name_;
      std::string motor_pub_topic_;
      std::string frame_id_;

      double max_rot_velocity_;
      double time_constant_;
      double motor_constant_;
      double rotor_drag_coefficient_;
      double moment_constant_;
      double rolling_moment_coefficient_;
      int turning_direction_;
      int motor_number_;
      int rotor_velocity_slowdown_sim_;

      double ref_motor_rot_vel_;

      event::ConnectionPtr updateConnection_;
      

    };
    GZ_REGISTER_MODEL_PLUGIN(rotor_plugin)
}

#endif //PLUGINS_PELICAN_ROTOR_PLUGIN_H
