#include "ros/ros.h"
#include "std_msgs/String.h"

#include "trajectory_creator/trajectory_pelican.h"
#include "math.h"
#include <iostream>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;
typedef Matrix<double, 6, 1> Vector6d;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory");
  ros::NodeHandle n;

  trajectory first_trajectory(&n, 20);
//  Vector3d start_position(-7, 7, 1);
//  aux = first_trajectory.doing_sine_trajectory(start_position,3);
  Vector3d start_position1(0,  0, 1);
//  Vector3d start_position2(-17,  2, 2);
//  Vector3d start_position3(-20, -2, 2);
//  first_trajectory.doing_linear_trajectory(start_position1, start_position2, start_position3);
    first_trajectory.doing_circle_trajectory(start_position1, 0.165);
}
