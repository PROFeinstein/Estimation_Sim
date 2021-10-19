#ifndef TRAJECTORY_PELICAN_H
#define TRAJECTORY_PELICAN_H

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "pelican_ctrl_pkg/ref_values.h"
#include "pelican_ctrl_pkg/euler_values.h"
#include "tgt_pstion/target_position_estimation.h"
#include "math.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/Twist.h"

const char x=0;
const char y=1;
const char z=2;

using namespace Eigen;
using namespace std;
typedef Matrix<double, 6, 1> Vector6d;

class trajectory
{
private:
  ros::NodeHandle n;
  ros::Rate loop_rate;
  ros::Publisher trajectory1_pub;
  ros::Subscriber linear_sub;
  ros::Publisher velocity_publisher;

  pelican_ctrl_pkg::ref_values new_trajectory;

  Vector3d position;
  int samples;

public:

  trajectory(ros::NodeHandle* nh, int rate);
  virtual ~trajectory();
  void linear_position(const pelican_ctrl_pkg::euler_values::ConstPtr& );
  void send_position(Vector4d );
  vector<Vector3d> doing_circle_trajectory(Vector3d start_position, double radius);
  vector<Vector3d> doing_sine_trajectory(Vector3d start_position, double radius);
  double doing_lateral_trajectory(Vector3d start_position, double radius);
  double vector_magnitude(Vector3d& vector);
};

#endif // TRAJECTORY_PELICAN_H
