#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PointStamped

from husky_moveit_config.srv import Indicate

class Test:

    def callback(self, p):
        print(p)
        resp = self.serv(p)
        print(resp.result)
        
    def __init__(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('listener', anonymous=True)

        self.serv = rospy.ServiceProxy('indicate', Indicate)

        rospy.Subscriber("clicked_point", PointStamped, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    Test()