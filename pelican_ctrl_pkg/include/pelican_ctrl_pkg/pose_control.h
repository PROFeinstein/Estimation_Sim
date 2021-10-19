//
// Created by Jose Dena Ruiz on 04/09/2018.
//

#ifndef PELICAN_CTRL_PKG_POSE_CONTROL_H
#define PELICAN_CTRL_PKG_POSE_CONTROL_H

#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include "pelican_ctrl_pkg/ref_values.h"
#include "pelican_ctrl_pkg/controllers.h"
#include "pelican_ctrl_pkg/euler_values.h"
#include "controllers_data_type.h"
#include "geometry_msgs/Pose.h"
#include "pelican_data_types.h"
#include "tf/transform_datatypes.h"
#include <math.h>

using namespace Eigen;
using namespace std;

enum euler_angles{roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate};
typedef Matrix<double, 6, 1>  Vector6d;

class pose_controllers {
private:
    ros::NodeHandle n;

    ros::Subscriber imu_pose_sub;
    ros::Subscriber imu_euler_sub;
    ros::Subscriber new_ref_sub;
    ros::Publisher ctrl_inputs_pub;

    control_variables roll_var;
    control_variables pitch_var;
    control_variables yaw_var;
    control_variables x_var;
    control_variables y_var;
    control_variables z_var;

    double m_mass;
    double grav_acc;
    reference_inputs new_ref_val;
    pelican_ctrl_pkg::ref_values ctrl_inputs_msg;

public:

    controllers roll_ctrl;
    controllers pitch_ctrl;
    controllers yaw_ctrl;
    controllers x_ctrl;
    controllers y_ctrl;        
    controllers z_ctrl;

    pose_controllers(ros::NodeHandle* nh, controller_init roll_init, controller_init pitch_init, controller_init yaw_init, controller_init x_, controller_init y_, controller_init z_);
    virtual ~pose_controllers();
    void get_pose_fnc(const geometry_msgs::Pose::ConstPtr&);
    void get_euler_pose_fnc(const pelican_ctrl_pkg::euler_values::ConstPtr&);
    void get_reference_values(const pelican_ctrl_pkg::ref_values::ConstPtr&);
    template<typename T>
    T saturate(T value, T minimum, T maximum){return min(max(value, minimum), maximum);}

};

#endif //PELICAN_CTRL_PKG_POSE_CONTROL_H