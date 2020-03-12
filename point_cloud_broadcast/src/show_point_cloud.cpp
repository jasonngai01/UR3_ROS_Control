#include <ros/ros.h> 
#include <pcl/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h> 
#include <math.h>

#define PI 3.14159265

int capture_x[21] = {0};
int capture_y[21] = {0};


float output_x[21] = {0};
float output_y[21] = {0};

void capture_coorindates()
{
   capture_x[0] = 227; capture_y[0] = 251;
   capture_x[1] = 219; capture_y[1] = 276;
   capture_x[2] = 108; capture_y[2] = 211;
   capture_x[3] = 331; capture_y[3] = 211;
   capture_x[4] = 147; capture_y[4] = 131;
   capture_x[5] = 115; capture_y[5] = 203;
   capture_x[6] = 83; capture_y[6] = 163;
   capture_x[7] = 60; capture_y[7] = 132;
   capture_x[8] = 44; capture_y[8] = 100;
   capture_x[9] = 147; capture_y[9] = 180;
   capture_x[10] = 132; capture_y[10] = 131;
   capture_x[11] = 116; capture_y[11] = 75;
   capture_x[12] = 115; capture_y[12] = 28;
   capture_x[13] = 179; capture_y[13] = 172;
   capture_x[14] = 179; capture_y[14] = 108;
   capture_x[15] = 220; capture_y[15] = 59;
   capture_x[16] = 227; capture_y[16] = 20;
   capture_x[17] = 220; capture_y[17] = 171;
   capture_x[18] = 227; capture_y[18] = 123;
   capture_x[19] = 227; capture_y[19] = 60;
   capture_x[20] = 227; capture_y[20] = 20;  
}

void coord_shift()
{
   const float end_effector_x = 0.13;
   const float end_effector_y = 0.52;

   const float image_ref_x = 164.0;
   const float image_ref_y = 243.0;
   
   float world_distance = 0.095;

   float image_distance = 0.0;
   image_distance = sqrt(pow(image_ref_x,2)+pow(image_ref_y,2));

   float transform_ratio = 0.0;
   transform_ratio = world_distance/image_distance;

   for(int i=0;i<=21;i++)
   {
     output_x[i] = capture_x[i]*transform_ratio+end_effector_x;
     output_y[i] = capture_y[i]*transform_ratio+end_effector_y;
   }

}

int main (int argc, char **argv) 
{ 
  ros::init (argc, argv, "show_point_cloud"); 
	
  ros::NodeHandle nh; 
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);     
  pcl::PointCloud<pcl::PointXYZ> cloud; 
  sensor_msgs::PointCloud2 output; 


  // Fill in the cloud data 
  cloud.width = 21; 
  cloud.height = 2; 

  capture_coorindates();
  
  coord_shift();
  //Height and width = row and colomns of point cloud
  cloud.points.resize(cloud.width * cloud.height); 

  for (int i = 0;i <= 21;i++)
  {
    cloud.points[i].x = output_x[i];
    cloud.points[i].y = output_y[i];
    cloud.points[i].z = 0;
    printf("Index:%d ",i);
    printf("x = %.2f ",output_x[i]);
    printf("y = %.2f\r\n",output_y[i]);
  }

  //Convert the cloud to ROS message 
  pcl::toROSMsg(cloud, output); 
  output.header.frame_id = "odom"; 

  ros::Rate loop_rate(1); 
  while (ros::ok()) 
  { 
    pcl_pub.publish(output);
    ros::spinOnce(); 
    loop_rate.sleep(); 
  } 
  return 0; 
}
