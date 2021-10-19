//
// Created by Jose Dena Ruiz on 04/09/2018.
//

#include "pelican_ctrl_pkg/pose_control.h"
//////////////////////////////////////////////Constructor////////////////////////////////////////////////////
pose_controllers::pose_controllers(ros::NodeHandle* nh, controller_init roll_, controller_init pitch_, controller_init yaw_,controller_init x_, controller_init y_, controller_init z_):n(*nh), 
roll_ctrl(&n, roll_), pitch_ctrl(&n, pitch_), yaw_ctrl(&n, yaw_),x_ctrl(&n, x_),y_ctrl(&n, y_),z_ctrl(&n, z_), m_mass(1.54),grav_acc(9.81)
{
imu_pose_sub  = n.subscribe("pelican_ctrl/pose",4,&pose_controllers::get_pose_fnc, this);
imu_euler_sub = n.subscribe("pelican_ctrl/euler_pose",4,&pose_controllers::get_euler_pose_fnc, this);
new_ref_sub  = n.subscribe("pelican_ctrl/des_reference_values",4,&pose_controllers::get_reference_values,this);

ctrl_inputs_pub = n.advertise<pelican_ctrl_pkg::ref_values>("pelican_ctrl/ctrl_inputs" ,1);

new_ref_val.desired_x   = 0.0;
new_ref_val.desired_y   = 0.0;
new_ref_val.desired_z   = 0.0;
new_ref_val.desired_yaw = 0.0;

}
//////////////////////////////////////////////Destructor////////////////////////////////////////////////////
pose_controllers::~pose_controllers(){}
//////////////////////////////////////////////Get Reference values///////////////////////////////////////////////////////
void pose_controllers::get_reference_values(const pelican_ctrl_pkg::ref_values::ConstPtr& ref_val)
{
new_ref_val.desired_x   = ref_val->desired_x;
new_ref_val.desired_y   = ref_val->desired_y;
new_ref_val.desired_z   = ref_val->desired_z;
}
//////////////////////////////////////////////Get current orientation////////////////////////////////////////////////////
void pose_controllers::get_pose_fnc(const geometry_msgs::Pose::ConstPtr& pose_values)
{

}
//////////////////////////////////////////////////////////////////////////////////////////////////
void pose_controllers::get_euler_pose_fnc(const pelican_ctrl_pkg::euler_values::ConstPtr& pose_values)
{

    x_var = x_ctrl.PID_simple_2(new_ref_val.desired_x, pose_values->x, 0, pose_values->x_rate);    
    y_var = y_ctrl.PID_simple_2(new_ref_val.desired_y, pose_values->y, 0, pose_values->y_rate);
    z_var = z_ctrl.PID_simple_2(new_ref_val.desired_z, pose_values->z, 0, pose_values->z_rate);

    z_var.control_signal = ((z_var.control_signal + grav_acc)*m_mass) / (cos(pose_values->roll)*cos(pose_values->pitch));
    //z_var.control_signal = ((grav_acc)*m_mass) / (cos(pose_values->roll)*cos(pose_values->pitch));

    Matrix2d yaw_orientation;
    yaw_orientation << sin(pose_values->yaw), cos(pose_values->yaw), -cos(pose_values->yaw), sin(pose_values->yaw);
    Vector2d r1_r2; r1_r2 << x_var.control_signal, y_var.control_signal;
    Vector2d roll_pitch = (m_mass / z_var.control_signal) * yaw_orientation.inverse() * r1_r2;

    double des_roll = saturate(roll_pitch(0), -1.0, 1.0);//-0.2617, 0.2617);    
    double des_pitch = saturate(roll_pitch(1), -1.0, 1.0);//-0.2617, 0.2617);

    roll_var  = roll_ctrl.PID_simple_2(des_roll, pose_values->roll, 0, pose_values->roll_rate);
    pitch_var = pitch_ctrl.PID_simple_2(des_pitch, pose_values->pitch, 0, pose_values->pitch_rate);
    yaw_var   = yaw_ctrl.PID_simple_2(0.0, pose_values->yaw, 0, pose_values->yaw_rate);

    ctrl_inputs_msg.desired_x = roll_var.control_signal;
    ctrl_inputs_msg.desired_y = pitch_var.control_signal;
    ctrl_inputs_msg.desired_yaw = yaw_var.control_signal;
    ctrl_inputs_msg.desired_z = z_var.control_signal;
    ctrl_inputs_msg.header.stamp = ros::Time::now();
    ctrl_inputs_pub.publish(ctrl_inputs_msg);
}
