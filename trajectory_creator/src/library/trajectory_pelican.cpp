#include "trajectory_creator/trajectory_pelican.h"

trajectory::trajectory(ros::NodeHandle* nh, int rate):n(*nh), loop_rate(rate)
{
    trajectory1_pub = n.advertise<pelican_ctrl_pkg::ref_values>("pelican_ctrl/des_reference_values", 1);
    linear_sub = n.subscribe("pelican_ctrl/euler_pose" , 1 ,&trajectory::linear_position, this);
     velocity_publisher = n.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
trajectory::~trajectory(){}
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void trajectory::linear_position(const pelican_ctrl_pkg::euler_values::ConstPtr& linear_position_values)
{
    position << linear_position_values->x , linear_position_values->y, linear_position_values->z;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double trajectory::vector_magnitude(Vector3d &vector)
{
 return sqrt(pow(vector(0),2)+pow(vector(1),2)+pow(vector(2),2));
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void trajectory::send_position(Vector4d new_position)
{
  new_trajectory.desired_x = new_position(x);
  new_trajectory.desired_y = new_position(y);
  new_trajectory.desired_z = new_position(z);
  new_trajectory.desired_yaw = new_position(3);
  new_trajectory.header.stamp = ros::Time::now();
  trajectory1_pub.publish(new_trajectory);
  ros::spinOnce();
  loop_rate.sleep();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
vector<Vector3d> trajectory::doing_circle_trajectory(Vector3d start_position, double radius)
{
  Vector4d circle_values;
  Vector4d start_position_(start_position(x), start_position(y), start_position(z), 0.0);
  tgt_postion_est mines(&n);
  vector<Vector3d>  estimated_objects;
  double ukf_depth_estimate;
  float ang = 0.0;
  double mag_diff = 1;
  double a;
  double b;
  double c;
// while ( mag_diff > 0.2 && ros::ok())
  //{
    //cout <<"moving to location ... ("<< start_position(x)<<","<< start_position(y)<<","<< start_position(z)<< ") Current position... ("<< position(x)<<","<< position(y)<<","<< position(z)<< ")"  <<endl;
    //send_position(start_position_);
    //double mag_start  = vector_magnitude(start_position);
    //double mag_pstion = vector_magnitude(position);
      //    mag_diff   = mag_start - mag_pstion;
    //cout << mag_start << "<-->" << mag_pstion << "<-->" << mag_diff<<endl;
    //}
geometry_msgs::Twist vel_msg;
  //cout <<"In location. " <<endl;
  //sleep(5);

  samples = 0;
  do//(ros::ok())
  {
    samples ++;
    ang += 0.07;
    //circle_values(x) = (radius-0.5) * cos(ang+M_PI) + (start_position(x) + (radius-0.5));
   //circle_values(y) = radius * sin(ang+M_PI) + start_position(y);
   //circle_values(z) = start_position(z) + (0.075*ang);
    //circle_values(z) = start_position(z);
//////////////////////////////////////////////////////////////////////////////////////////////////////

//b = 1.2 * (radius * cos(ang+M_PI)); 
//a = radius * -sin(ang+M_PI);
//b = radius; 
//a = radius;  
c = 0;
vel_msg.linear.x = a;
vel_msg.linear.y = b;
vel_msg.linear.z = c;
circle_values(x) = a;
circle_values(y) = b;
circle_values(z) = c;

       vel_msg.angular.x = radius * cos(ang+M_PI);
	vel_msg.angular.y = radius *  sin(ang+M_PI);
	vel_msg.angular.z =0.0;//0.0025;
velocity_publisher.publish(vel_msg);
//////////////////////////////////////////////////////////////////////////////////////////////////////
    //circle_values(z) = ang+M_PI;
    circle_values(3) = 0.0;
    send_position(circle_values);
   //tgt_postion_est::estimation_objects(estimated_objects, ukf_depth_estimate);
   mines.estimation_objects(estimated_objects, ukf_depth_estimate);
    for (int i=0; i< estimated_objects.size(); i++)
      cout<<"Object"<<i+1<<" position = ("<<estimated_objects[i](x)<<", "<<estimated_objects[i](y)<<", "<<estimated_objects[i](x)<<")"<<endl;
      
    cout << endl;
    //cout << "Samples: " << samples << endl;


  }while (ang < 9*M_PI && ros::ok());
  cout << "Trajectory Done!!..."<< endl;
  sleep(5);
  
  return estimated_objects;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double trajectory::doing_lateral_trajectory(Vector3d start_position, double radius)
{
  Vector4d circle_values;
  tgt_postion_est mines(&n);
  vector<Vector3d>  estimated_objects;
  double depth_estimate;

  samples = 0;
  do
  {
    samples ++;
    circle_values(x) = 0.0;
    circle_values(y) = 0.05;
    circle_values(z) = 0.0;
    circle_values(3) = 0.0;
    send_position(circle_values);
    mines.estimation_objects(estimated_objects, depth_estimate);
  }while (position(y) < start_position(y) + radius && ros::ok());

  do
  {
    samples ++;
    circle_values(x) = 0.0;
    circle_values(y) = -0.05;
    circle_values(z) = 0.0;
    circle_values(3) = 0.0;
    send_position(circle_values);
    mines.estimation_objects(estimated_objects, depth_estimate);
  }while (position(y) > start_position(y) - radius && ros::ok());

  do
  {
    samples ++;
    circle_values(x) = 0.0;
    circle_values(y) = 0.05;
    circle_values(z) = 0.0;
    circle_values(3) = 0.0;
    send_position(circle_values);
    mines.estimation_objects(estimated_objects, depth_estimate);
  }while (position(y) < start_position(y) && ros::ok());

  circle_values(x) = 0.0;
  circle_values(y) = 0.0;
  circle_values(z) = 0.0;
  send_position(circle_values);

  cout << "Trajectory Done!!..."<< endl;
  sleep(5);
  return depth_estimate;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
vector<Vector3d> trajectory::doing_sine_trajectory(Vector3d start_position, double radius)
{
  Vector4d postion;
  Vector4d start_position_(start_position(x), start_position(y), start_position(z), 0.0);
  //tgt_postion_est mines;
  vector<Vector3d>  estimated_objects;
  float ang = 0.0;

  while (position != start_position && ros::ok())
    {
    cout <<"moving to location... " <<endl;
    send_position(start_position_);
    }
  cout <<"In location. " <<endl;
  sleep(5);

  samples = 0;
  do//(ros::ok())
  {
    samples ++;
    ang += 0.01;
    postion(x) = (radius + 3) * sin(ang)  + start_position(x);
    postion(y) = radius * cos(ang) + start_position(y) - radius;
    postion(z) = start_position(z);
    postion(3) = 0.0;
    send_position(postion);
    cout << postion << endl;
    //mines.estimation_objects(estimated_objects);

  }while (ang < M_PI && ros::ok());
  cout << "Trajectory Done!!..."<< endl;
  sleep(5);
  return estimated_objects;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
