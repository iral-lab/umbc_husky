#!/usr/bin/env python

import sys
import copy
import math
import numpy as np
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def norm(x):
  return math.sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2])


def get_ee_position(p, r):
  p = np.asarray(p)
  
  return r*(p)/norm(p)

def get_ee_orientation(p):
  x = p[0]
  y = p[1]
  z = p[2]

  yaw = math.atan2(y,x)
  pitch = math.atan2(z, math.sqrt(x*x + y*y))


  return pitch,yaw


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "right_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self):
    current_pose = self.move_group.get_current_pose().pose
    #print(current_pose)

    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] =  0.0
    joint_goal[1] =  2.9
    joint_goal[2] =  1.3
    joint_goal[3] = -2.0
    joint_goal[4] =  1.4
    joint_goal[5] =  0.0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

    current_pose = self.move_group.get_current_pose().pose
    #print(current_pose)

    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self):
    current_pose = self.move_group.get_current_pose().pose
    print("start:", current_pose)
    '''
    x = 0.2912
    y =-0.197
    z = 0.560
    '''

    pose_goal=geometry_msgs.msg.Pose()
    '''
    pose_goal.position.x= 1.0377335981
    pose_goal.position.y= -0.207925526458
    pose_goal.position.z= 0.7498956456
    pose_goal.orientation.x= 0.0283788251864
    pose_goal.orientation.y= -0.730188307821
    pose_goal.orientation.z= -0.00276158105613
    pose_goal.orientation.w= 0.682650753367
    '''

    p = [10.0, -10.0, -10.0]

    pos = get_ee_position(p, 0.8)
    pose_goal.position.x= pos[0] + 0.2912
    pose_goal.position.y= pos[1] - 0.197
    pose_goal.position.z= 1.0#pos[2] + 0.560
    
    pitch,yaw = get_ee_orientation(p)
    pitch = -(pitch+(math.pi/2.0))

    q = quaternion_from_euler(0.0, pitch, yaw)
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2] 
    pose_goal.orientation.w = q[3]
    
    #pose_goal.orientation.w = 1.0
    
    print("goal:", pose_goal)

    self.move_group.set_pose_target(pose_goal)
    ## Now, we call the planner to compute the plan and execute it.
    plan = self.move_group.go(wait=True)

    print(plan)
    
    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose
    print("end:", current_pose)


    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)





def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the MoveIt MoveGroup Python Interface Tutorial"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
    #raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()
    '''
    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    raw_input()
    tutorial.go_to_joint_state()
    '''
    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    tutorial.go_to_pose_goal()

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

