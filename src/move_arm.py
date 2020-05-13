#! /usr/bin/env python
import time
import math
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PointStamped
import tf


class ArmController:
    def __init__(self):
        rospy.init_node('move_robot_using_trajectory_msg')		
        self.prefix = 'j2n6s300'
        self.nbJoints = 6 
        self.nbfingers = 3

        self.listener = tf.TransformListener()


        self.home_angles = [0.0,2.9,1.3,4.2,1.4,0.0]

        arm_topic_name = '/effort_joint_trajectory_controller/command'
        self.arm_pub = rospy.Publisher(arm_topic_name, JointTrajectory, queue_size=1)

        finger_topic_name = '/effort_finger_trajectory_controller/command'
        self.finger_pub = rospy.Publisher(finger_topic_name, JointTrajectory, queue_size=1)  
        self.sub = rospy.Subscriber("/indicate", PointStamped, self.indicate)

        rospy.spin()

    def indicate(self, point):

        self.listener.waitForTransform("/j2n6s300_link_base", point.header.frame_id, rospy.Time(0),rospy.Duration(4.0))
        p = self.listener.transformPoint("/j2n6s300_link_base", point)

        x = p.point.x
        y = p.point.y
        h = math.sqrt(x*x + y*y)

        theta = math.asin(y/h)

        print(p)
        print(theta)

        #foward +1.5 left -1.5 right
        angles = [-theta, 1.9, 3.3, 2.2, 2.4,0.0]

        self.moveJoint(angles, self.prefix, self.nbJoints)
        self.moveFingers([1, 1, 1], self.prefix, self.nbfingers)
 
    def moveJoint(self, jointcmds, prefix, nbJoints):

        jointCmd = JointTrajectory()  
        point = JointTrajectoryPoint()
        jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
        point.time_from_start = rospy.Duration.from_sec(5.0)
        for i in range(0, nbJoints):
            jointCmd.joint_names.append(prefix +'_joint_'+str(i+1))
            point.positions.append(jointcmds[i])
            point.velocities.append(0)
            point.accelerations.append(0)
            point.effort.append(0) 
        jointCmd.points.append(point)
        rate = rospy.Rate(100)
        count = 0
        while (count < 50):
            self.arm_pub.publish(jointCmd)
            count = count + 1
            rate.sleep()     

    def moveFingers(self, jointcmds,prefix,nbJoints):
        jointCmd = JointTrajectory()  
        point = JointTrajectoryPoint()
        jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
        point.time_from_start = rospy.Duration.from_sec(5.0)
        for i in range(0, nbJoints):
            jointCmd.joint_names.append(prefix +'_joint_finger_'+str(i+1))
            point.positions.append(jointcmds[i])
            point.velocities.append(0)
            point.accelerations.append(0)
            point.effort.append(0) 
        jointCmd.points.append(point)
        rate = rospy.Rate(100)
        count = 0
        while (count < 500):
            self.finger_pub.publish(jointCmd)
            count = count + 1
            rate.sleep()     

if __name__ == '__main__':
    a = ArmController()

