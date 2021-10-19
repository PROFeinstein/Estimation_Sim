#ifndef TARGET_POSITION_ESTIMATION_H
#define TARGET_POSITION_ESTIMATION_H

#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <tgt_pstion/tgt_kalman_filters.h>
#include "pelican_ctrl_pkg/azimut_elevation.h"
#include "pelican_ctrl_pkg/euler_values.h"
#include "pelican_ctrl_pkg/pelican_data_types.h"
#include <geometry_msgs/Point.h>
#include "detection_network/pixel.h"


using namespace std;
using namespace Eigen;

class tgt_postion_est
{
private:
  ros::NodeHandle nh_;

  ros::Publisher azimut_elevation_pub;
  ros::Subscriber angles_sub_;
  ros::Subscriber pixel_sub_;
  ros::Publisher estimation_pub;

  pelican_ctrl_pkg::azimut_elevation angles_objects;
  geometry_msgs::Point tgt_position_;

  vector<tgt_kalman_filters> estimator;
  tgt_kalman_filters estimator_2;
  

  string name_;
  int c;

  int first_detection;
  Vector3d attitude;
  Vector4d pstion;

  vector<Vector2d> meas;

  const double focal_length;
  const int x,y,z;
  double a,b;
  const Vector3d camera_offset;
  Matrix4d drone2world;


 public:
  tgt_postion_est(ros::NodeHandle* nh);
  ~tgt_postion_est();
  void estimation_objects(vector<Vector3d>& Xhat, double& depth_est);
  void angles_and_position(const pelican_ctrl_pkg::euler_values::ConstPtr& );
  void EstimateCallback(const detection_network::pixel::ConstPtr& msg);
  double mod(double a, double m);
  };

#endif // TARGET_POSITION_ESTIMATION_H

