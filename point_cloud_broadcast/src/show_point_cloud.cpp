#include <ros/ros.h> 
#include <pcl/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h> 
#include <math.h>
#include <visualization_msgs/Marker.h>

#define PI 3.14159265

int capture_x[13] = {0};
int capture_y[13] = {0};


float output_x[13] = {0};
float output_y[13] = {0};



void capture_coorindates()
{
   //Alpha
   capture_x[0] = 66; capture_y[0] = 194;
   //Beta
   capture_x[1] = 66; capture_y[1] = 294;
   //Gammar
   capture_x[2] = 598; capture_y[2] = 186;
   //Delta
   capture_x[3] = 598; capture_y[3] = 343;
   //Index 0
   capture_x[4] = 79; capture_y[4] = 244;
   //Index 2
   capture_x[5] = 79; capture_y[5] = 219;
   //Index 1
   capture_x[6] = 79; capture_y[6] = 269;
   //Index 8
   capture_x[7] = 294; capture_y[7] = 221;
   //Index 6
   capture_x[8] = 466; capture_y[8] = 258;
   //Index 7
   capture_x[9] = 552; capture_y[9] = 224;
   //Index 3
   capture_x[10] = 595; capture_y[10] = 224;
   //Index 4
   capture_x[11] = 595; capture_y[11] = 302;
   //Index 5
   capture_x[12] = 595; capture_y[12] = 263;
}

void coord_shift()
{
   //Coordinates of end_effector (capture) in Rviz
   const float end_effector_x = 0.13;
   const float end_effector_y = 0.44;
   //Coordinates of end_effecor (capture) in AI
   const float image_ref_x = 320;
   const float image_ref_y = 240;
   
   //Distance of Image Origin --> A Image point
   float world_distance = 0.136;

   float image_distance = 0.0;
   image_distance = sqrt(pow(image_ref_x,2)+pow(image_ref_y,2));

   float transform_ratio = 0.0;
   transform_ratio = world_distance/image_distance;

   const float correct_x = -0.11;
   const float correct_y = -0.03;

   for(int i=0;i<13;i++)
   {
     output_x[i] = capture_x[i]*transform_ratio+end_effector_x+correct_x;
     output_y[i] = capture_y[i]*transform_ratio+end_effector_y+correct_y;
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
  cloud.width = 13; 
  cloud.height = 2; 

  capture_coorindates();
  
  coord_shift();
  //Height and width = row and colomns of point cloud
  cloud.points.resize(cloud.width * cloud.height); 

  for (int i = 0;i < 13;i++)
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
  output.header.frame_id = "world"; 

  ros::Rate loop_rate(1); 
  while (ros::ok()) 
  { 
    pcl_pub.publish(output);
    ros::spinOnce(); 
    loop_rate.sleep(); 
  } 
  return 0; 
}
