//
// Created by Jose Dena Ruiz on 04/09/2018.
//
#ifndef PELICAN_CTRL_PKG_CONTROLLERS_DATA_TYPE_H
#define PELICAN_CTRL_PKG_CONTROLLERS_DATA_TYPE_H

struct controller_init{
  double kp_;
  double kd_;
  double ki_;
  double step;
  std::string name;
};

struct pid_gains{
  double kp;
  double kd;
  double ki;
};

struct pid_states{
  double desired;
  double current;
  double error;
};

struct reference_inputs{
  double desired_x;
  double desired_y;
  double desired_z;
  double desired_yaw;
};

struct control_variables{
  double kp_gain;
  double kd_gain;
  double ki_gain;
  double desired_pos;
  double current_pos;
  double error_pos;
  double error_derivative;
  double error_integral;
  double control_signal;
};

struct backstepping_variables{
  double alpha1;
  double alpha2;
  double desired_pos;
  double desired_vel;
  double current_pos;
  double current_vel;
  double z1;
  double z2;
  double control_signal;
};


#endif // PELICAN_CTRL_PKG_CONTROLLERS_DATA_TYPE_H
