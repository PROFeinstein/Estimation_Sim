//
// Created by Jose Dena Ruiz on 04/09/2018.
//

#ifndef PELICAN_CTRL_PKG_CONTROLLERS_H
#define PELICAN_CTRL_PKG_CONTROLLERS_H

#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include "pelican_ctrl_pkg/control_signals.h"
#include "controllers_data_type.h"

class controllers
{
private:
    ros::NodeHandle n;

    ros::Publisher  ctrl_data_pub;
    pelican_ctrl_pkg::control_signals ctrl_signals_msg;

    control_variables ctrl_data;

    double m_kp, m_kd, m_ki;
    double m_dt;
    std::string m_name;
    double m_prev_error;

public:
    controllers(ros::NodeHandle* nh, controller_init init_values);
    virtual ~controllers();
    control_variables PID_simple  (double desired_pos, double current_pos);
    control_variables PID_simple_2(double desired_pos, double current_pos, double desired_vel, double current_vel);
    control_variables PID_yaw_360 (double desired_pos, double current_pos);
    void increase_decrease_gains  (pid_gains gains);
    void change_all_gains(double kp, double kd, double ki, double step);
    pid_gains get_gains();
    pid_states get_states();

protected:

};


#endif // PELICAN_CTRL_PKG_CONTROLLERS_H
