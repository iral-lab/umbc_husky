#! /usr/bin/env python2
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
        self.init = False

        self.listener = tf.TransformListener()

#        self.home_angles = [0.0,math.pi/2,2*math.pi,0,0,0]
        self.home_angles = [0.0, 2.9-math.pi/2, 1.3+3*math.pi/2, -2.07, 1.4, 0.0]

        arm_topic_name = self.prefix + '/effort_joint_trajectory_controller/command'
        self.arm_pub = rospy.Publisher(arm_topic_name, JointTrajectory, queue_size=1)

        finger_topic_name = self.prefix + '/effort_finger_trajectory_controller/command'
        self.finger_pub = rospy.Publisher(finger_topic_name, JointTrajectory, queue_size=1)  
        self.sub = rospy.Subscriber("/indicate", PointStamped, self.indicate, queue_size=1)

        rospy.spin()

    def indicate(self, point):
        now = rospy.Time.now()
        self.listener.waitForTransform("/j2n6s300_link_base", point.header.frame_id, now, rospy.Duration(1.0))
        point.header.stamp = now
        p = self.listener.transformPoint("/j2n6s300_link_base", point)

        # Point relative to base of arm
        x0, y0, z0 = point.point.x, point.point.y, point.point.z
#        print(x0,y0, z0)

        # Point relative to robot
        x1, y1, z1 = p.point.x, p.point.y, p.point.z

        # Distance from base to target
        r = min(1, max(0, math.sqrt(x0**2+y0**2+z0**2)))

        # Distance from base to target on plane
        R = min(1, max(0, math.sqrt(x0**2+y0**2)))

        # Length of upperarm
        j1 = 0.51

        #Length of lower arm
        j2 = 0.51
        if r > 0.01:
            alpha = self.home_angles[1]+math.acos((j1**2+r**2-j2**2)/(2*j1*r))+math.atan2(z0, R)
            beta = self.home_angles[2] - math.acos((j2**2+j1**2-r**2)/(2*j1*j2))
            theta = self.home_angles[0]-math.atan2(y0, x0) # Base Rotation
            #foward +1.57 left -1.57 right
            angles = [theta, alpha, beta, math.pi/2,0,0]

            print(angles)

            self.moveJoint(angles, self.prefix, self.nbJoints)
        #self.moveFingers([1, 1, 1], self.prefix, self.nbfingers)
 
    def moveJoint(self, jointcmds, prefix, nbJoints):

        jointCmd = JointTrajectory()  
        point = JointTrajectoryPoint()
        jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
        point.time_from_start = rospy.Duration.from_sec(0.5)
        for i in range(0, nbJoints):
            jointCmd.joint_names.append(prefix +'_joint_'+str(i+1))
            point.positions.append(jointcmds[i])
            point.velocities.append(0)
            point.accelerations.append(0)
            point.effort.append(0) 
        jointCmd.points.append(point)
        self.arm_pub.publish(jointCmd)

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

