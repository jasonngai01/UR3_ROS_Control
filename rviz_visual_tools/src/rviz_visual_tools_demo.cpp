/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, PickNik Consulting
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Demo implementation of rviz_visual_tools
           To use, add a Rviz Marker Display subscribed to topic /rviz_visual_tools
*/

// ROS
#include <ros/ros.h>

// For visualizing things in rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

// C++
#include <string>
#include <vector>
#include <math.h>

#define PI 3.14159265

int capture_x[13] = {0};
int capture_y[13] = {0};


double output_x[13] = {0};
double output_y[13] = {0};

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
   const double end_effector_x = 0.13;
   const double end_effector_y = 0.44;
   //Coordinates of end_effecor (capture) in AI
   const double image_ref_x = 320;
   const double image_ref_y = 240;
   
   //Distance of Image Origin --> A Image point
   double world_distance = 0.136;

   double image_distance = 0.0;
   image_distance = sqrt(pow(image_ref_x,2)+pow(image_ref_y,2));

   double transform_ratio = 0.0;
   transform_ratio = world_distance/image_distance;

   const double correct_x = -0.11;
   const double correct_y = -0.03;

   for(int i=0;i<13;i++)
   {
     output_x[i] = capture_x[i]*transform_ratio+end_effector_x+correct_x;
     output_y[i] = capture_y[i]*transform_ratio+end_effector_y+correct_y;
   }

}

namespace rvt = rviz_visual_tools;

namespace rviz_visual_tools
{
class RvizVisualToolsDemo
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  // For visualizing things in rviz
  rvt::RvizVisualToolsPtr visual_tools_;

  std::string name_;

public:
  /**
   * \brief Constructor
   */
  RvizVisualToolsDemo() : name_("rviz_demo")
  {
    visual_tools_.reset(new rvt::RvizVisualTools("world", "/rviz_visual_tools"));
    visual_tools_->loadMarkerPub();  // create publisher before waiting

    ROS_INFO("Sleeping 2 seconds before running demo");
    ros::Duration(2).sleep();

    // Clear messages
    visual_tools_->deleteAllMarkers();
    visual_tools_->enableBatchPublishing();
  }

  void publishLabelHelper(const Eigen::Isometry3d& pose, const std::string& label)
  {
    Eigen::Isometry3d pose_copy = pose;
    pose_copy.translation().x() += 0.2;
    pose_copy.translation().y() += 0.08;
    visual_tools_->publishText(pose_copy, label, rvt::WHITE, rvt::LARGE, false);
  }

  void drawing()
  {
    // Create pose
    Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();



    double y = 0;

    // --------------------------------------------------------------------
    ROS_INFO_STREAM_NAMED(name_, "Displaying Arrows");
    pose1 = Eigen::Isometry3d::Identity();
    for (int i = 4; i < 13; i++)
    {
      pose1.translation().y() = output_y[i];
      pose1.translation().x() = output_x[i];
      visual_tools_->publishZArrow(pose1, rvt::RAND,SMALL);
      if (i == 4)
      {
        publishLabelHelper(pose1, "ACP");
      }
   
    }
    visual_tools_->trigger();

    // --------------------------------------------------------------------
    ROS_INFO_STREAM_NAMED(name_, "Displaying Sized Wireframe Cuboid");
    pose1 = Eigen::Isometry3d::Identity();
    double difference = 0.0;
    //Most widest: gammar_y <-> delta_y
    difference = (output_y[3]+output_y[2]);
    pose1.translation().y() = difference*0.5;
    //Longest: alpha_x <-> gammar_x
    difference = (output_x[3]+output_x[0]);
    pose1.translation().x() = difference*0.5;
    double depth = 0.2, width = 0.2, height = 0.2;
  
    width = output_y[0]-output_y[3];
    depth = output_x[2]-output_x[1];
    visual_tools_->publishWireframeCuboid(pose1, depth, width, height, rvt::CYAN);
    publishLabelHelper(pose1, "AI Boundary");

    visual_tools_->trigger();  
  }
};  // end class

}  // namespace rviz_visual_tools

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_tools_demo");
  ROS_INFO_STREAM("Visual Tools Demo");

  capture_coorindates();
  
  coord_shift();

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  rviz_visual_tools::RvizVisualToolsDemo demo;
  //x_location and y_location are the center of the box
  demo.drawing();
  ROS_INFO_STREAM("Shutting down.");

  return 0;
}
