#!/usr/bin/env python2
import sys
import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
from std_msgs.msg import String

class wooz_hand_control:
    def __init__(self):
        rospy.init_node('wooz_hand_control')

        moveit_commander.roscpp_initialize(sys.argv)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "right_hand"
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
        self.eef_link = eef_link
        self.group_names = group_names
        self.s = rospy.Subscriber('/buttons', String, self.buttons_cb)
        rospy.spin()

    def buttons_cb(self, buttons_str):

        print(buttons_str.data)
        target = None

        if (buttons_str.data == "grabbed") :
            target = "close"
        if (buttons_str.data == "released") :
            target = "open"

        print(target)

        if (target != None):
            self.move_group.set_named_target(target)
            self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()

if __name__ == "__main__":
    wooz_hand_control()
