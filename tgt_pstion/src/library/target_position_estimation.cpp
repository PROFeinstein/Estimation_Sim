#include "tgt_pstion/target_position_estimation.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
tgt_postion_est::tgt_postion_est(ros::NodeHandle* nh): nh_(*nh), first_detection(1),attitude(0,0,0), pstion(0,0,0,1), x(0), y(1), z(2), a(0), b(0),
                                    //camera_offset(0.225, 0 , 0.035) ,focal_length(381.36) // simulation_values
                                    camera_offset(0.21, 0 , -0.04), focal_length(583.871099)// real_platform_values camera(200wC 666.66 pixels) camera(202bC 1066.66 pixels)
{
angles_sub_ = nh_.subscribe("/pelican_ctrl/euler_pose" ,1 ,&tgt_postion_est::angles_and_position, this);
pixel_sub_ =  nh_.subscribe("/center_pixel" ,1 ,&tgt_postion_est::EstimateCallback, this);
azimut_elevation_pub = nh_.advertise<pelican_ctrl_pkg::azimut_elevation>("/tgt_pstion/azimut_elevation_angles",1);
estimation_pub = nh_.advertise<geometry_msgs::Point>("/tgt_pstion/xyz_estimation_position",1);
c=0;  
drone2world.setZero();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
tgt_postion_est::~tgt_postion_est()
{

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double tgt_postion_est::mod(double a, double m)
{
  return a - m*floor(a/m);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void tgt_postion_est::EstimateCallback(const detection_network::pixel::ConstPtr& msg) {

 a=msg->x;
 b=msg->y;

//myvector.push_back (x);
//myvector.push_back (y);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void tgt_postion_est::angles_and_position(const pelican_ctrl_pkg::euler_values::ConstPtr& pelican_angles)
{
  attitude << pelican_angles->roll, pelican_angles->pitch, pelican_angles->yaw;
  pstion   << pelican_angles->x, pelican_angles->y, pelican_angles->z, 1.0;
  drone2world.block(0,0,3,3) = RPY_ROT(pelican_angles->roll, pelican_angles->pitch, pelican_angles->yaw);
  drone2world.block(0,3,4,1) = Eigen::Vector4d(pelican_angles->x, pelican_angles->y, pelican_angles->z, 1);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void tgt_postion_est::estimation_objects(vector<Vector3d>& Xhat, double& depth_est)
{
  if (first_detection == 1)
    {
       Vector3d x_hat(0,0,0);
       estimator.resize(2);
       Xhat.resize(2);
       meas.resize(2);
       first_detection = 0;
      return;
    }
     
  else
    {
      for (int i=0; i<2; i++)
      {
          double u0 = 376.5;
          double v0 = 240.5;
          double rho= 6E-6;
          double fl = 0.0035;    
          MatrixXd K;   
          K.setZero(3,3);      
          K << 1.0/rho,      0,  u0,
                      0,1.0/rho,  v0,
                      0,      0, 1.0;

          Matrix4d cam2drone;
          cam2drone << 0, 0, 1, 0.00,
                      -1, 0, 0, 0.00,
                       0,-1, 0, 0.00,
                       0, 0, 0, 1.00;

          Matrix4d target2cam;
          target2cam <<-1, 0, 0, 0,
                        0,-1, 0, 0,
                        0, 0, 1, 0,
                        0, 0, 0, 1;

          Vector3d uvw_point;
          //uvw_point << tracking_objects[0].x, tracking_objects[0].y, 1.0;
          
          uvw_point << a, b, 1.0;
         // std::cout<<uvw_point<<std::endl;
          
          uvw_point << K.inverse() * uvw_point;
          
          Vector4d uvw_homogeneous(uvw_point(0), uvw_point(1), fl, 1.0);
          uvw_homogeneous << drone2world * cam2drone * uvw_homogeneous;
          uvw_homogeneous << pstion - uvw_homogeneous;
/////////////////////////////////////////////////////////////////////////////////////////////////
          double beta = atan(uvw_homogeneous(1) / uvw_homogeneous(0));
        
          double alpha = atan(uvw_homogeneous(2) / sqrt(pow(uvw_homogeneous(0),2) + pow(uvw_homogeneous(1),2)));
          //double alpha = (uvw_homogeneous(2)/cos(90 - (atan(uvw_homogeneous(2) / sqrt(pow(uvw_homogeneous(0),2) + pow(uvw_homogeneous(1),2))))));
          //double alpha =sqrt(pow(ali,2) +  pow(uvw_homogeneous(2),2));
            //double alpha = sqrt(pow(uvw_homogeneous(0),2) + pow(uvw_homogeneous(1),2));
           
          meas[i] << beta, alpha;

          Vector3d position(pstion(0),pstion(1),pstion(2));
          position = position + camera_offset;
          //x_hat = estimator_2.kalman_unscented_estimator(position, meas[i]);
	  Xhat[i] = estimator[i].kalman_unscented_estimator(position, meas[i]);
 }
}
  return;
}

