//
// Created by Jose Dena Ruiz on 04/09/2018.
//

#include "pelican_ctrl_pkg/sensor_adq_fusion.h"
//////////////////////////////////////////Constructor and Destructor/////////////////////////////////////////////////////
sensor_adq_fusion::sensor_adq_fusion(ros::NodeHandle* nh, double imu_freq, double mag_freq, double sonar_freq, double camera_freq, double gt_freq):n(*nh)
{
    
    imu_sub   = n.subscribe("/ardrone/imu", 4 ,&sensor_adq_fusion::IMU_Pose_Estimator, this);
    mag_sub   = n.subscribe("/magnetic", 4, &sensor_adq_fusion::magnetic_receiver,this);
    sonar_sub = n.subscribe("/sonar_height",4,&sensor_adq_fusion::get_sonar, this);
    ground_truth_sub = n.subscribe("/ground_truth/state",4, &sensor_adq_fusion::get_ground_thrut, this);
    gt_2_sub = n.subscribe("/gazebo/model_states", 4, &sensor_adq_fusion::get_ground_thrut_2, this);
    //image_sub = it_.subscribe("/mv_26803026/image_rect_color", 1, &sensor_adq_fusion::get_image, this);

    pelican_pose_pub = n.advertise<geometry_msgs::Pose>("pelican_ctrl/pose",1);
    pelican_pose_euler_pub = n.advertise<pelican_ctrl_pkg::euler_values>("pelican_ctrl/euler_pose",1);
    acc_only_pub = n.advertise<geometry_msgs::Vector3>("pelican_ctrl/acc_only",1);
    pose_estimation_pub = n.advertise<pelican_ctrl_pkg::pose_estimation>("pelican_ctrl/pose_estimation", 1);

}

sensor_adq_fusion::~sensor_adq_fusion(void) { cout << "Object imu_2_attitude destroyed" << endl; }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Vector3d sensor_adq_fusion::low_pass_filter_vector(Vector3d vector_)
{

}

////////////////////////////////////Receiver Magnetometer readings func/////////////////////////////////////////////////////
void sensor_adq_fusion::magnetic_receiver(const geometry_msgs::Vector3Stamped::ConstPtr& mag_data)
{

}
////////////////////////////////////Receiver IMU readings func/////////////////////////////////////////////////////
void sensor_adq_fusion::IMU_Pose_Estimator(const sensor_msgs::Imu::ConstPtr& imu_data)
{

}
////////////////////////////////////////////Get Ground Thrut////////////////////////////////////////////////////
void sensor_adq_fusion::get_ground_thrut(const nav_msgs::Odometry::ConstPtr& pose_)
{

}

void sensor_adq_fusion::get_ground_thrut_2(const gazebo_msgs::ModelStates::ConstPtr& pose_)
{
int model_no = 1;

tf::Quaternion q_;
tf::quaternionMsgToTF(pose_->pose[model_no].orientation, q_);
//double roll, pitch, yaw;
tf::Matrix3x3(q_.normalize()).getRPY(roll_, pitch_, yaw_);
pelican_pose_euler_msg.header.stamp = ros::Time::now();
pelican_pose_euler_msg.roll       = roll_;
pelican_pose_euler_msg.pitch      = pitch_;
pelican_pose_euler_msg.yaw        = yaw_;
pelican_pose_euler_msg.roll_rate  = pose_->twist[model_no].angular.x;
pelican_pose_euler_msg.pitch_rate = pose_->twist[model_no].angular.y;
pelican_pose_euler_msg.yaw_rate   = pose_->twist[model_no].angular.z;

pelican_pose_euler_msg.x = pose_->pose[model_no].position.x;
pelican_pose_euler_msg.y = pose_->pose[model_no].position.y;
pelican_pose_euler_msg.z = pose_->pose[model_no].position.z;
pelican_pose_euler_msg.x_rate = pose_->twist[model_no].linear.x;
pelican_pose_euler_msg.y_rate = pose_->twist[model_no].linear.y;
pelican_pose_euler_msg.z_rate = pose_->twist[model_no].linear.z;

pelican_pose_euler_pub.publish(pelican_pose_euler_msg);
}

/////////////////////////////////////////////////////UltraSound/////////////////////////////////////////////////////

void sensor_adq_fusion::get_sonar(const sensor_msgs::RangeConstPtr& range_)
{

}
/////////////////////////////////////////////////////WGN/////////////////////////////////////////////////////
double sensor_adq_fusion::GuassianKernel(double mu, double sigma)
{
// Define random generator with Gaussian distribution
const double mean = 0.0;
// construct a trivial random generator engine from a time-based seed:
unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
default_random_engine generator(seed);
normal_distribution<double> distribution(mean, sigma);

return mu + distribution(generator);
}
