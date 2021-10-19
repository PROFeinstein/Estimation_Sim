//
// Created by Jose Dena Ruiz on 04/09/2018.
//

#ifndef PELICAN_CTRL_PKG_ROTOR_VELOCITIES_H
#define PELICAN_CTRL_PKG_ROTOR_VELOCITIES_H

#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include "controllers_data_type.h"
#include "pelican_ctrl_pkg/ref_values.h"
#include "cvg_sim_gazebo_plugins/MotorSpeed.h"
#include "std_msgs/Bool.h"

using namespace Eigen;
using namespace std;


class rotor_velocities{
private:
    ros::NodeHandle n;
    ros::Subscriber ctrl_signals_sub;
    ros::Subscriber pelican_onoff_sub;
    ros::Publisher rotor_vels_pub;

    cvg_sim_gazebo_plugins::MotorSpeed rotor_vels_msg;

    Vector4d ctrl_signals;
    Vector4d rotor_vels_sqr;
    Matrix4d torque_2_speeds;

    double m_kt;
    double m_kd;
    double m_len;
    bool on_off;

public:
    rotor_velocities(ros::NodeHandle* nh);
    virtual ~rotor_velocities(void);
    void get_ctrl_signals(const pelican_ctrl_pkg::ref_values::ConstPtr&);
    void get_on_off_pelican(const std_msgs::Bool::ConstPtr&);
};


#endif //PELICAN_CTRL_PKG_ROTOR_VELOCITIES_H
