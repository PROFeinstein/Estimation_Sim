#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "yolov2_detect.hpp"

//int32_T main(int32_T, const char * const [])
int main(int argc, char **argv)
{
    ROS_INFO("Node Starting");
    
    ros::init(argc, argv, "GpuNet");
    
    ros::NodeHandle n;
    
    /* Here is where we setup the publishing class defined in alexnet.hpp.
     * We define the names of the two topic we want to publish our output data to.
     * We chose to publish the classification index to /network_out and the
     * image being passed to alexnet to /network_view.
     */
    ///////////////////////////////////////////////////////////////////





/////////////////////////////////////////////////////////////////////////
    GpuNetPub img_pub("center_pixel","network_view", 1, &n);
    
    /* Setup the subscriber to the topic publishing the raw image messages.
     * We utilize the msgCallback method of our publisher as the new message callback.
     */
    
    ros::Subscriber img_sub = n.subscribe("/ardrone/image_raw", 1, &GpuNetPub::msgCallback, &img_pub);
    
    ROS_INFO("Node Started Successfully");
    ros::spin();
    ROS_INFO("Node Shutting Down");
}
