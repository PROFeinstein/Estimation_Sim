//
// Created by Jose Dena Ruiz on 22/11/2018.
//

#ifndef SENSOR_ADQ_FUSION_H
#define SENSOR_ADQ_FUSION_H

#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>
#include <random>
///////////////////////////////////////
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
///////////////////////////////////////
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
///////////////////////////////////////
#include "pelican_ctrl_pkg/pose_estimation.h"
#include "pelican_ctrl_pkg/euler_values.h"
#include "pelican_ctrl_pkg/pelican_data_types.h"
#include "pelican_ctrl_pkg/kalman_filters.h"
#include "tf/transform_datatypes.h"

using namespace Eigen;
using namespace std;

class sensor_adq_fusion{
private:
    ros::NodeHandle n;

    ros::Subscriber imu_sub;
    ros::Subscriber mag_sub;
    ros::Subscriber sonar_sub;
    ros::Subscriber ground_truth_sub;
    ros::Subscriber gt_2_sub;

    ros::Publisher pelican_pose_pub;
    ros::Publisher pelican_pose_euler_pub;
    ros::Publisher acc_only_pub;
    ros::Publisher pose_estimation_pub;

    geometry_msgs::Vector3 acc_only_msg;
    geometry_msgs::Pose pelican_pose_msg;
    pelican_ctrl_pkg::euler_values pelican_pose_euler_msg;
    pelican_ctrl_pkg::pose_estimation pose_estimation_msg;

    double roll_, pitch_, yaw_;


public:
    sensor_adq_fusion(ros::NodeHandle* nh, double imu_freq, double mag_freq, double sonar_freq, double camera_freq, double gt_freq);
    virtual ~sensor_adq_fusion(void);
    void IMU_Pose_Estimator(const sensor_msgs::Imu::ConstPtr&);
    void magnetic_receiver(const geometry_msgs::Vector3Stamped::ConstPtr&);
    void get_sonar(const sensor_msgs::Range::ConstPtr&);
    void get_ground_thrut(const nav_msgs::Odometry::ConstPtr&);
    void get_ground_thrut_2(const gazebo_msgs::ModelStates::ConstPtr& );
    double GuassianKernel(double mu, double sigma);
    Vector3d low_pass_filter_vector(Vector3d vector_);

};
#endif 
