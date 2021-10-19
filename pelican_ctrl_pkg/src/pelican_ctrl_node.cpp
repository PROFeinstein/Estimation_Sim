#include <iostream>
#include <ros/ros.h>
#include "keyboard/Key.h"
#include "pelican_ctrl_pkg/sensor_adq_fusion.h"
#include "pelican_ctrl_pkg/controllers_data_type.h"
#include "pelican_ctrl_pkg/pose_control.h"
#include "pelican_ctrl_pkg/rotor_velocities.h"
#include <stdlib.h>

using namespace std;

class pelican_ctrl_node
{
private:
    ros::NodeHandle n;
    ros::Subscriber     keyboard_sub;

    sensor_adq_fusion   pelican_imu;
    rotor_velocities    send_rotor_vels;

    int key_switch;

public:
    pose_controllers    pelican_pose_ctrl;
    string ctrl_name;
    pid_gains gains_tune;
    pid_gains actual_gains;
    pid_states ctrlr_states;

    pelican_ctrl_node(ros::NodeHandle* nh, controller_init roll, controller_init pitch, controller_init yaw, controller_init altitude, controller_init x_position, controller_init y_position);
    ~pelican_ctrl_node();
    void keyboard_down_fnc(const keyboard::Key::ConstPtr&);
};

pelican_ctrl_node::pelican_ctrl_node(ros::NodeHandle* nh, controller_init roll, controller_init pitch, controller_init yaw, controller_init x_position, controller_init y_position, controller_init z_position):
n(*nh), key_switch(0),pelican_imu(&n, 0.01, 0.05, 0.05, 0.033, 0.001), pelican_pose_ctrl(&n, roll, pitch, yaw, x_position, y_position, z_position), ctrl_name("ROLL"), send_rotor_vels(&n)
{
    keyboard_sub = n.subscribe("keyboard/keydown",4,&pelican_ctrl_node::keyboard_down_fnc, this);   
    actual_gains = pelican_pose_ctrl.roll_ctrl.get_gains();
}

pelican_ctrl_node::~pelican_ctrl_node()
{
}


void pelican_ctrl_node::keyboard_down_fnc(const keyboard::Key::ConstPtr &keys)
{
    double kp_keyboard=0.0;
    double kd_keyboard=0.0;
    double ki_keyboard=0.0;

    switch (keys->code)
    {
    case 273:
        kp_keyboard = 0.1;
        break;
    case 274:
        kp_keyboard = -0.1;
        break;
    case 275:
        kd_keyboard = 0.1;
        break;
    case 276:
        kd_keyboard = -0.1;
        break;
    case 112:
        ki_keyboard = 0.01;
        break;
    case 59:
        ki_keyboard = -0.01;
        break;
    case 32:
        if (key_switch <= 5)
            key_switch += 1;
        else
            key_switch = 0;
        break;
    }

    switch (key_switch)
    {
    case 0:
        gains_tune.kp = kp_keyboard; gains_tune.kd = kd_keyboard; gains_tune.ki = ki_keyboard;
        pelican_pose_ctrl.roll_ctrl.increase_decrease_gains(gains_tune);
        actual_gains = pelican_pose_ctrl.roll_ctrl.get_gains();
        ctrlr_states = pelican_pose_ctrl.roll_ctrl.get_states();
        ctrl_name = "ROLL      ";
        break;
    case 1:
        gains_tune.kp = kp_keyboard; gains_tune.kd = kd_keyboard; gains_tune.ki = ki_keyboard;
        pelican_pose_ctrl.pitch_ctrl.increase_decrease_gains(gains_tune);
        actual_gains = pelican_pose_ctrl.pitch_ctrl.get_gains();
        ctrlr_states = pelican_pose_ctrl.pitch_ctrl.get_states();
        ctrl_name = "PITCH     ";
        break;
    case 2:
        gains_tune.kp = kp_keyboard; gains_tune.kd = kd_keyboard; gains_tune.ki = ki_keyboard;
        pelican_pose_ctrl.yaw_ctrl.increase_decrease_gains(gains_tune);
        actual_gains = pelican_pose_ctrl.yaw_ctrl.get_gains();
        ctrlr_states = pelican_pose_ctrl.yaw_ctrl.get_states();
        ctrl_name = "YAW       ";
        break;
    case 3:
        gains_tune.kp = kp_keyboard; gains_tune.kd = kd_keyboard; gains_tune.ki = ki_keyboard;
        pelican_pose_ctrl.x_ctrl.increase_decrease_gains(gains_tune);
        actual_gains = pelican_pose_ctrl.x_ctrl.get_gains();
        ctrlr_states = pelican_pose_ctrl.x_ctrl.get_states();
        ctrl_name = "X_POSITION";
        break;
    case 4:
        gains_tune.kp = kp_keyboard; gains_tune.kd = kd_keyboard; gains_tune.ki = ki_keyboard;
        pelican_pose_ctrl.y_ctrl.increase_decrease_gains(gains_tune);
        actual_gains = pelican_pose_ctrl.y_ctrl.get_gains();
        ctrlr_states = pelican_pose_ctrl.y_ctrl.get_states();
        ctrl_name = "Y_POSITION";
        break;
    case 5:
        gains_tune.kp = kp_keyboard; gains_tune.kd = kd_keyboard; gains_tune.ki = ki_keyboard;
        pelican_pose_ctrl.z_ctrl.increase_decrease_gains(gains_tune);
        actual_gains = pelican_pose_ctrl.z_ctrl.get_gains();
        ctrlr_states = pelican_pose_ctrl.z_ctrl.get_states();
        ctrl_name = "Z_POSITION";
        break;
    }
}

int main (int argc, char **argv)
{
    
    ros::init(argc, argv, "pelican_ctrl_node");
    ros::NodeHandle nh;

    controller_init roll;
    controller_init pitch;
    controller_init yaw;

    roll.kp_  = 5.0 ;roll.kd_  = 0.5; roll.ki_  = 0.0; roll.step  = 0.01; roll.name = "roll";
    pitch.kp_ = 5.0 ;pitch.kd_ = 0.5; pitch.ki_ = 0.0; pitch.step = 0.01; pitch.name = "pitch";
    yaw.kp_   = 2.0 ;yaw.kd_   = 0.4; yaw.ki_   = 0.0; yaw.step   = 0.01; yaw.name = "yaw";

    controller_init x_position;
    controller_init y_position;
    controller_init z_position;

    //Position controllers
    x_position.kp_  = 2.0 ;x_position.kd_  = 0.6; x_position.ki_  = 0.0; x_position.step  = 0.01; x_position.name = "x";
    y_position.kp_  = 2.0 ;y_position.kd_  = 0.6; y_position.ki_  = 0.0; y_position.step  = 0.01; y_position.name = "y";
    z_position.kp_  = 6.0 ;z_position.kd_  = 3.0; z_position.ki_  = 0.0; z_position.step  = 0.01; z_position.name = "z";

    /// Velocity controllers
    //  x_position.kp_  = 0.0 ;x_position.kd_  =  7.0; x_position.ki_ = 3.5; x_position.step  = 0.001; x_position.name = "x";
    //  y_position.kp_  = 0.0 ;y_position.kd_  =  7.0; y_position.ki_ = 3.5; y_position.step  = 0.001; y_position.name = "y";
    //  z_position.kp_  = 0.0 ;z_position.kd_  =  7.0; z_position.ki_ = 5.5; z_position.step  = 0.001; z_position.name = "z";
    
    
    pelican_ctrl_node pelican(&nh, roll, pitch, yaw, x_position, y_position, z_position);
    
    cout << endl;
    while(ros::ok())
        {
        // cout << "\rController Tuning Selection: " << pelican.ctrl_name  << endl<< endl;
        // printf("\rProportional_gain = %2.2f\n",pelican.actual_gains.kp);
        // printf("\rDerivative_gain   = %2.2f\n",pelican.actual_gains.kd);
        // printf("\rIntegral_gain     = %2.2f\n",pelican.actual_gains.ki);
        
        // cout << "\033[A \033[A \033[A \033[A \033[A ";
        ros::spinOnce();
        }
    // system("CLS");
    //ros::spin();
    return 0;
}
