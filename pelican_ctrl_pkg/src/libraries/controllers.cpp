//
// Created by Jose Dena Ruiz on 04/09/2018.
//

#include "pelican_ctrl_pkg/controllers.h"
#include <math.h>

controllers::controllers(ros::NodeHandle* nh, controller_init init_values):n(*nh), m_prev_error(0.0)
{
  m_dt = init_values.step;
  m_kp = init_values.kp_;
  m_kd = init_values.kd_;
  m_ki = init_values.ki_;
  m_name = init_values.name;

  std::string topic_name = "pelican_ctrl/"+ m_name +"_ctrl_signals";
  ctrl_data_pub  = n.advertise<pelican_ctrl_pkg::control_signals>(topic_name ,1);

  ctrl_data.desired_pos      = 0.0;
  ctrl_data.current_pos      = 0.0;
  ctrl_data.kp_gain          = 0.0;
  ctrl_data.kd_gain          = 0.0;
  ctrl_data.ki_gain          = 0.0;
  ctrl_data.error_pos        = 0.0;
  ctrl_data.error_derivative = 0.0;
  ctrl_data.error_integral   = 0.0;
  ctrl_data.control_signal   = 0.0;

}

controllers::~controllers()
{
}

void controllers::increase_decrease_gains(pid_gains gains)
{
  m_kp += gains.kp;
  m_kd += gains.kd;
  m_ki += gains.ki;
}

pid_gains controllers::get_gains()
{
  pid_gains gains;
  gains.kp = m_kp;
  gains.kd = m_kd;
  gains.ki = m_ki;
  return gains;
}

pid_states controllers::get_states()
{
  pid_states state;
  state.desired = ctrl_data.desired_pos;
  state.current = ctrl_data.current_pos;
  state.error   = ctrl_data.error_pos;
  return state;
}

void controllers::change_all_gains(double kp, double kd, double ki, double step)
{
  m_kp = kp;
  m_kd = kd;
  m_ki = ki;
  m_dt = step;
  ctrl_data.desired_pos      = 0.0;
  ctrl_data.current_pos      = 0.0;
  ctrl_data.kp_gain          = 0.0;
  ctrl_data.kd_gain          = 0.0;
  ctrl_data.ki_gain          = 0.0;
  ctrl_data.error_pos        = 0.0;
  ctrl_data.error_derivative = 0.0;
  ctrl_data.error_integral   = 0.0;
  ctrl_data.control_signal   = 0.0;
}

control_variables controllers::PID_simple(double desired_pos, double current_pos)
{
  ctrl_data.desired_pos    = desired_pos;
  ctrl_data.current_pos    = current_pos;
  ctrl_data.kp_gain        = m_kp;
  ctrl_data.kd_gain        = m_kd;
  ctrl_data.ki_gain        = m_ki;
  ctrl_data.error_pos      = desired_pos - current_pos;
  ctrl_data.error_derivative = (ctrl_data.error_pos - m_prev_error) / m_dt; 
  m_prev_error = ctrl_data.error_pos;
  ctrl_data.error_integral += ctrl_data.error_pos  * m_dt;
  ctrl_data.control_signal = ctrl_data.error_pos * m_kp + ctrl_data.error_derivative * m_kd + ctrl_data.error_integral * m_ki;

  ctrl_signals_msg.header.stamp = ros::Time::now();
  ctrl_signals_msg.name        = m_name + " Position";
  ctrl_signals_msg.desired_pos = ctrl_data.desired_pos;
  ctrl_signals_msg.current_pos = ctrl_data.current_pos;
  ctrl_signals_msg.error_pos   = ctrl_data.error_pos;
  ctrl_signals_msg.gain_1      = ctrl_data.kp_gain;
  ctrl_signals_msg.gain_2      = ctrl_data.kd_gain;
  ctrl_signals_msg.gain_3      = ctrl_data.ki_gain;
  ctrl_signals_msg.control_signal = ctrl_data.control_signal;
  ctrl_data_pub.publish(ctrl_signals_msg);

return ctrl_data;
}

control_variables controllers::PID_simple_2(double desired_pos, double current_pos, double desired_vel, double current_vel)
{
  ctrl_data.desired_pos    = desired_pos;
  ctrl_data.current_pos    = current_pos;
  ctrl_data.kp_gain        = m_kp;
  ctrl_data.kd_gain        = m_kd;
  ctrl_data.ki_gain        = m_ki;
  ctrl_data.error_pos      = desired_pos - current_pos;
  ctrl_data.error_derivative = desired_vel - current_vel;
  ctrl_data.error_integral += ctrl_data.error_derivative  * m_dt;
  ctrl_data.control_signal = ctrl_data.error_pos * m_kp + ctrl_data.error_derivative * m_kd + ctrl_data.error_integral * m_ki;

  ctrl_signals_msg.header.stamp = ros::Time::now();
  ctrl_signals_msg.name        = m_name + " Position";
  ctrl_signals_msg.desired_pos = ctrl_data.desired_pos;
  ctrl_signals_msg.current_pos = ctrl_data.current_pos;
  ctrl_signals_msg.error_pos   = ctrl_data.error_pos;
  ctrl_signals_msg.gain_1      = ctrl_data.kp_gain;
  ctrl_signals_msg.gain_2      = ctrl_data.kd_gain;
  ctrl_signals_msg.gain_3      = ctrl_data.ki_gain;
  ctrl_signals_msg.control_signal = ctrl_data.control_signal;
  ctrl_data_pub.publish(ctrl_signals_msg);

return ctrl_data;
}

control_variables controllers::PID_yaw_360(double desired_pos, double current_pos)
{
  // ctrl_data.desired_pos    = desired_pos;
  // ctrl_data.desired_vel    = desired_vel;
  // ctrl_data.current_pos    = current_pos;
  // ctrl_data.current_vel    = current_vel;
  // ctrl_data.kp_gain        = m_kp;
  // ctrl_data.kd_gain        = m_kd;
  // ctrl_data.ki_gain        = m_ki;
  // ctrl_data.error_pos      = desired_pos - current_pos;
  // ctrl_data.error_pos      += (ctrl_data.error_pos > M_PI) ? -2*M_PI : ((ctrl_data.error_pos < -M_PI) ? 2*M_PI : 0);
  // ctrl_data.error_vel      = desired_vel - current_vel;
  // if (abs(ctrl_data.error_pos) > 0.01)
  //   m_error_integral         += ctrl_data.error_pos  * m_dt;
  // ctrl_data.control_signal = ctrl_data.error_pos * m_kp + ctrl_data.error_vel * m_kd + m_error_integral * m_ki;

  // return ctrl_data;
}
