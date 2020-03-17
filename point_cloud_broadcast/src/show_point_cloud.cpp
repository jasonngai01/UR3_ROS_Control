#include <ros/ros.h> 
#include <pcl/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h> 
#include <math.h>
#include <visualization_msgs/Marker.h>

#define PI 3.14159265

int capture_x[21] = {0};
int capture_y[21] = {0};


float output_x[21] = {0};
float output_y[21] = {0};



void capture_coorindates()
{
   capture_x[0] = 171; capture_y[0] = 331;
   capture_x[1] = 188; capture_y[1] = 284;
   capture_x[2] = 236; capture_y[2] = 252;
   capture_x[3] = 275; capture_y[3] = 219;
   capture_x[4] = 308; capture_y[4] = 188;
   capture_x[5] = 91; capture_y[5] = 227;
   capture_x[6] = 60; capture_y[6] = 195;
   capture_x[7] = 44; capture_y[7] = 164;
   capture_x[8] = 28; capture_y[8] = 140;
   capture_x[9] = 123; capture_y[9] = 211;
   capture_x[10] = 107; capture_y[10] = 156;
   capture_x[11] = 91; capture_y[11] = 115;
   capture_x[12] = 84; capture_y[12] = 68;
   capture_x[13] = 147; capture_y[13] = 203;
   capture_x[14] = 139; capture_y[14] = 139;
   capture_x[15] = 132; capture_y[15] = 84;
   capture_x[16] = 131; capture_y[16] = 51;
   capture_x[17] = 180; capture_y[17] = 196;
   capture_x[18] = 187; capture_y[18] = 124;
   capture_x[19] = 180; capture_y[19] = 99;
   capture_x[20] = 180; capture_y[20] = 59;  
}

void coord_shift()
{
   //Coordinates of end_effector (capture) in Rviz
   const float end_effector_x = 0.14;
   const float end_effector_y = 0.44;
   //Coordinates of end_effecor (capture) in AI
   const float image_ref_x = 127.5;
   const float image_ref_y = 299.5;
   
   //Distance of Image Origin --> A Image point
   float world_distance = 0.125;

   float image_distance = 0.0;
   image_distance = sqrt(pow(image_ref_x,2)+pow(image_ref_y,2));

   float transform_ratio = 0.0;
   transform_ratio = world_distance/image_distance;

   for(int i=0;i<=21;i++)
   {
     output_x[i] = capture_x[i]*transform_ratio+end_effector_x-0.045;
     output_y[i] = capture_y[i]*transform_ratio+end_effector_y-0.026-0.06;
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

  for (int i = 0;i < 21;i++)
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
