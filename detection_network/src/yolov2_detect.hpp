#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt32.h"
#include "yolov2_detect.h"
#include "detection_network/pixel.h"

/* This is a class that is setup as a method for publishing as part of a
 * subscriber callback. It has two publishers for the image and classification
 * data and a method msgCallback that defines what to do when it recieves an
 * image message from the camera.
 */

class GpuNetPub
{
    private:
    //Publishers for image and classification data
    ros::Publisher data_pub;
    ros::Publisher img_pub;
    
    public:
    //Constructor to setup the publishers
    GpuNetPub(std::string data_topic_name, std::string img_topic_name, int msg_limit, ros::NodeHandle* n)
    {
        data_pub = n->advertise<detection_network::pixel>(data_topic_name, msg_limit);
        img_pub = n->advertise<sensor_msgs::Image>(img_topic_name, msg_limit);
    }
    
    //Callback for new image message data
    void msgCallback(const sensor_msgs::Image::ConstPtr& inmsg)
    {
        //Declare output message
        detection_network::pixel dataoutmsg;
        sensor_msgs::Image imgoutmsg;
        
        //Unpack the message data
        uint32_t width = inmsg->width;
        uint32_t height = inmsg->height;
        uint32_t step = inmsg->step;
        
        //Declare the input data
        uint8_t input[224*224*3];
        uint8_t output[224*224*3];
        double* x;
        double* y;
        
        double jjj;
         //real_T bbox[3136];
        int32_t bbox_size[2];
        //Declare the output data
         
        
        std::vector<uint8_t> img_msg_data(224*224*3);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
//This loop selects a centered subset of the image data and
//reformats the encoding (rgb8 row major -> uint8_t [R | G | B] column major).
//This is done because the generated code expects the data to be in
//the same format as MATLAB uses.
         
        
        int32_t sub_height = 224;
        int32_t sub_width = 224;
        
        int32_t input_idx;
        int32_t msg_idx;
        int32_t out_idx = 0;
        
        for(int32_t i = 0; i<(sub_height); i++)
        {
            for(int32_t j = 0; j<(sub_width); j++)
            {
                for(int32_t k = 0; k<(3); k++)
                {
                    input_idx = (sub_width*sub_height*k) + (sub_height*j) + (i);
                    msg_idx = 3*width*(floor((height - sub_height)/2) + i) + 3*(floor((width - sub_width)/2) + j) + (k); // If the image is 227x227x3 : (width*3*i) + (3*j) + (k);
                    input[input_idx] = (inmsg->data[msg_idx]);
                    //img_msg_data[out_idx] = inmsg->data[msg_idx];
                     out_idx++;
                }
            }
        }
        
        //Call generated network on the input data
      
       	yolov2_detect(input,output,x,y); //bbox);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//This is done to convert the output and reformats the encoding (uint8_t RGB -> rgb8)
//so as to be able to publish as image data


      int32_t o_idx = 0;
      int32_t m_idx;
      int32_t i_idx;
          
        for(int32_t i = 0; i<(sub_height); i++)
        {
            for(int32_t j = 0; j<(sub_width); j++)
            {
                for(int32_t k = 0; k<(3); k++)
                {
                    i_idx = (sub_width*sub_height*k) + (sub_height*j) + (i);
                    msg_idx = 3*width*(floor((height - sub_height)/2) + i) + 3*(floor((width - sub_width)/2) + j) + (k); // If the image is 227x227x3 : (width*3*i) + (3*j) + (k);
                     img_msg_data[o_idx] = output[i_idx];
                    //img_msg_data[o_idx] = output[m_idx];
                     o_idx++;
                }
            }

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SECTION FOR PUBLISHING OUTPUT 

       //ROS_INFO("Image Classification Index", output);
//      std::vector<uint8_t> img_msg_data(std::begin(output), std::end(output));

    
     //Publish the classification data and image messages to the new topics
        ///dataoutmsg.data = output;
        //data_pub.publish(dataoutmsg);
       //std::vector<uint8_t> img_msg_data(std::begin(output), std::end(output));


  //dataoutmsg.data = bbox_size;
    //    data_pub.publish(dataoutmsg);

        double a = *x;
        double b = *y;
      
         dataoutmsg.x = a;
         dataoutmsg.y = b;
         data_pub.publish(dataoutmsg);
        imgoutmsg.height = sub_height;
        imgoutmsg.width = sub_width;
        imgoutmsg.encoding = "rgb8";
        imgoutmsg.step = 3*sub_width;
        
        
	imgoutmsg.data = img_msg_data;
        img_pub.publish(imgoutmsg);
        
    }
    
};
