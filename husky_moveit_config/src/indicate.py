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

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "right_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        print(planning_frame)
        self.eef_link = eef_link
        self.group_names = group_names

        s = rospy.Service('indicate', Indicate, self.serv_callback)
        rospy.spin()

    def serv_callback(self, req):
        current_pose = self.move_group.get_current_pose().pose
        pose_goal = Pose()

        self.listener.waitForTransform(req.point.header.frame_id, "/base_link",  rospy.Time(0),rospy.Duration(4.0))
        start = self.listener.transformPoint("/base_link", req.point)
        pose_goal.position = start.point


        '''
        get_ee_position(req.point.point, 0.8)
        pose_goal.position.z = 0.85
        q = get_ee_orientation(pose_goal.position, req.point.point)

        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2] 
        pose_goal.orientation.w = q[3]
        '''
                
        self.move_group.set_pose_target(pose_goal)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose

        return all_close(pose_goal, current_pose, 0.01)

if __name__ == "__main__":
    IndicateService()
