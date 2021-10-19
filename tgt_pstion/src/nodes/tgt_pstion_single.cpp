#include <ros/ros.h>
//////////////////////////////////////////////////////////
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//////////////////////////////////////////////////////////
#include <pelican_msgs/imu_convertion.h>
#include <tgt_pstion/linear_mov_kalman_filter.h>
//////////////////////////////////////////////////////////
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//////////////////////////////////////////////////////////
#include <iostream>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

using namespace std;
using namespace Eigen;

class odometry_vision_est
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

public:
void odometry_vision_est::imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    Mat gray;
    //Point chess_center, drone_position;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cvtColor( cv_ptr->image, gray, CV_BGR2GRAY );

    namedWindow("processed2",WINDOW_AUTOSIZE);
    imshow("processed2", cv_ptr->image);//cv_ptr->image);
    waitKey(1);

};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{


}
