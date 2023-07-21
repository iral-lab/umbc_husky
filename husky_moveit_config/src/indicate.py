#!/usr/bin/env python2
import sys
import math
import rospy
import tf
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
import numpy as np
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from husky_moveit_config.srv import Indicate

def norm(x):
    return math.sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2])

#the point may be farther than the robot may reach
#project the point onot a spere r merters from the robots base
#returns this position
def get_ee_position(p, r):
    norm = math.sqrt(p.x*p.x + p.y*p.y + p.z*p.z)
    
    new_point = Point()
    new_point.x = p.x * (r/norm)
    new_point.y = p.y * (r/norm)
    new_point.z = p.z * (r/norm)

    return new_point

def get_ee_orientation(ee, p):
    #get r,p,y from the goal position of the gripper to the point
    x = p.x - ee.x
    y = p.y - ee.y
    z = p.z - ee.z

    yaw = math.atan2(y,x)
    pitch = math.atan2(z, math.sqrt(x*x + y*y))

    '''
    convert from the robots frame of reference
    where roll,pitch,yaw of 0,0,0 is forward to
    grippers frame of reference where
    roll,pitch,yaw of 0,0,0 is down
    '''
    pitch = -(pitch + (math.pi/2.0))
    
    q = quaternion_from_euler(0.0, pitch, yaw)

    return q

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

  elif type(goal) is PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class IndicateService:
    def __init__(self):
        rospy.init_node('indicate_service')

        self.listener = tf.TransformListener()

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "right_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)


        s = rospy.Service('indicate', Indicate, self.serv_callback)
        rospy.spin()

    def serv_callback(self, req):
        pose_goal = Pose()

        self.listener.waitForTransform(req.point.header.frame_id, "/base_link",  rospy.Time(0),rospy.Duration(4.0))
        p = self.listener.transformPoint("/base_link", req.point)

        pose_goal.position = p.point
        pose_goal.orientation.w = 1.0

        self.move_group.set_max_velocity_scaling_factor(1.0)                
        self.move_group.set_pose_target(pose_goal)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose

        return all_close(pose_goal, current_pose, 0.01)

if __name__ == "__main__":
    IndicateService()