#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt32.h"
#include "yolov2_detect.h"



class detection {

 public: 
  detection(std::string name, uint32_t width, uint32_t height, uint32_t step)
  : name(name), width(width), height(height), step(step) {}
  std::string name;
  uint32_t width;
  uint32_t height;
  uint32_t step;
  };

class detection_publish {
 public:
detection_publish(const ros::Publisher& image_pub)
: /*publish_()*/ image_pub_(image_pub) {}

 void msgCallback(const sensor_msgs::Image::ConstPtr& inmsg)
    {
        //Declare output message
        std_msgs::Int32 dataoutmsg;
        sensor_msgs::Image imgoutmsg;
        
        //Unpack the message data
        uint32_t width = inmsg->width;
        uint32_t height = inmsg->height;
        uint32_t step = inmsg->step;
        
        //Declare the input data
        uint8_t input[224*224*3];
        uint8_t output[224*224*3];
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
      
       	yolov2_detect(input,output,bbox_size); //bbox);

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

        imgoutmsg.height = sub_height;
        imgoutmsg.width = sub_width;
        imgoutmsg.encoding = "rgb8";
        imgoutmsg.step = 3*sub_width;
        
        
	imgoutmsg.data = img_msg_data;
        image_pub_.publish(imgoutmsg);
}

private: 
ros::Publisher image_pub_;

};

int main(int argc, char** argv){

ros::init(argc, argv, "detection_publish");
ros::NodeHandle nh;
ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("network_view", 1);
detection_publish dictate(image_pub);
 ros::Subscriber img_sub = nh.subscribe("/ardrone/image_raw", 1, &detection_publish::msgCallback, &dictate);
ros::spin();
return 0;
}






