#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from math import pi
import numpy as np

robot = None
group = None
scene = None
display_trajectory_publisher = None

def euler_to_quaternion(Yaw, Pitch, Roll):
  yaw   = Yaw   * pi / 180 
  pitch = Roll  * pi / 180 
  roll  = Pitch * pi / 180 

  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return [qx, qy, qz, qw]

def ur3_config():
  global robot,group,scene,display_trajectory_publisher
  ## First initialize moveit_commander and rospy.
  #print "============ Starting system"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the ur3
  ## arm.  This interface can be used to plan and execute motions on the ur3
  ## arm.
  group = moveit_commander.MoveGroupCommander("ur3")

  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)


  ## Getting Basic Information
  ## We can get the name of the reference frame for this robot
  #print "[INFO] Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  #print "[INFO] Reference frame: %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  #print "[INFO] Robot Groups:"
  #print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  #print "[INFO]Printing robot state"
  #print robot.get_current_state()
  #print "========================================"
  #print "[UR3] Configuration Done"


def ur3_set_initial_pose():
  global robot,group,scene,display_trajectory_publisher
  #Clear any pose in previous
  group.clear_pose_targets()
  ## Then, we will get the current set of joint values for the group
  group_variable_values = group.get_current_joint_values()

  group_variable_values[0] = 0
  group_variable_values[1] = -pi/2-0.5
  group_variable_values[2] = -pi/2+0.5
  group_variable_values[3] = -pi/2
  group_variable_values[4] = pi/2
  group_variable_values[5] = 0
  group.set_joint_value_target(group_variable_values)

  plan_initial = group.plan()
  #If you want to control the true robot,or move the simulated robot
  #Instead of only showing trajectory
  group.go(wait=True)
  group.stop()
  group.clear_pose_targets()
  current_pose = group.get_current_pose().pose
  print "------------ Initial Pose Information ------------"
  print current_pose
  print "[INFO] Initial Pose is executed"


def ur3_set_end_effector_capture():
  global robot,group,scene,display_trajectory_publisher
  current_pose = group.get_current_pose().pose
  print "Original Pose Information"
  print current_pose
  # XYZ are in terms of meters
  x_target = 0.20
  y_target = 0.43
  z_target = 0.13
  #Yaw,pitch and roll should be in degree    
  #They are all relative to base link coorindates     
  roll = 90   #Green axis
  yaw = 90    #Blue axis
  pitch = 90   #Red axis

  pose_goal = geometry_msgs.msg.Pose()
  Q = euler_to_quaternion(yaw , pitch, roll)
  print Q
  pose_goal.orientation.x = Q[0]
  pose_goal.orientation.y = Q[1]
  pose_goal.orientation.z = Q[2]
  pose_goal.orientation.w = Q[3]

  pose_goal.position.x = x_target
  pose_goal.position.y = y_target
  pose_goal.position.z = z_target
  group.set_pose_target(pose_goal)

  plan = group.go(wait=True)
  group.stop()
  current_pose = group.get_current_pose().pose
  print "------------ Target Pose Information ------------"
  print current_pose
  group.clear_pose_targets()
  print "[INFO] UR3 is ready to capture"

def ur3_set_end_effector_goal():
  global robot,group,scene,display_trajectory_publisher
  current_pose = group.get_current_pose().pose
  print "Original Pose Information"
  print current_pose
  # XYZ are in terms of meters
  x_target = 0.13
  y_target = 0.42
  z_target = 0.18
  #Yaw,pitch and roll should be in degree    
  # They are all relative to base link coorindates     
  roll = 0
  yaw = 0
  pitch = 0

  pose_goal = geometry_msgs.msg.Pose()
  Q = euler_to_quaternion(yaw , pitch, roll)
  print Q
  pose_goal.orientation.x = Q[0]
  pose_goal.orientation.y = Q[1]
  pose_goal.orientation.z = Q[2]
  pose_goal.orientation.w = Q[3]

  pose_goal.position.x = x_target
  pose_goal.position.y = y_target
  pose_goal.position.z = z_target
  group.set_pose_target(pose_goal)

  plan = group.go(wait=True)
  group.stop()
  current_pose = group.get_current_pose().pose
  print "------------ Target Pose Information ------------"
  print current_pose
  group.clear_pose_targets()
  print "[INFO] Goal of end effector is arrived"

if __name__=='__main__':
  try:
    ur3_config()
    #Pose for initialization
    ur3_set_initial_pose()
    rospy.sleep(3)
    #Pose for capture
    ur3_set_end_effector_capture()
    rospy.sleep(3)
    #Acupuncture
    #ur3_set_end_effector_goal()
  except rospy.ROSInterruptException:
    pass
