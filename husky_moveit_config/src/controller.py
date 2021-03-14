#!/usr/bin/env python2

import rospy
import tf
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import String
# from msg import PointAction, PointActionArray

from husky_moveit_config.srv import Indicate
from husky_moveit_config.srv import Collect

class Controller:
    def __init__(self):
        # establish as node
        rospy.init_node('controller', anonymous=True)

        self.listener = tf.TransformListener()

        # subscribe to python API messages giving ((x,y,z), action)
        #self.api_sub = rospy.Subscriber("/collect", PointStamped, self.callback)
        self.api_sub = rospy.Service("/collect", Collect, self.callback)

        # call the Indicate service to move the arm
        self.serv = rospy.ServiceProxy("indicate", Indicate)

        # publish buttons to button topic
        self.button_pub = rospy.Publisher("/buttons", String, queue_size=10)

        # publish transformations to move
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # publish controller feedback messages
        self.cont_feedback_pub = rospy.Publisher("/controller_feedback", String, queue_size=10)

        # define basket location that's fixed
        ps = PointStamped()
        ps.header.frame_id = "kinect2_link"
        ps.point.x = 0.0
        ps.point.y = 0.0
        ps.point.z = 0.50
        self.basket_location = ps

        rospy.spin()

    def callback(self, req):
        data = req.point
        print(type(data))
        # create feedback message
        feedback = String()
        # TODO: have list of points
        # for datum in data:
        #     # convert input type to point // verify this is needed
        #     #object_location.header.frame_id = "/base_link"
        #     #object_location.header.stamp = TODO set this...
        #
        #     # based on action, figure out what to do
        #     if datum.action == 'to_basket':
        #         self.move_object(datum.loc, self.basket_location)
        #     else:
        #         feedback.data = "Unknown action"
        #         self.cont_feedback_pub.publish(feedback)

        self.move_object(data, self.basket_location)

        # populate done feedback and publish
        feedback.data = "Done"
        self.cont_feedback_pub.publish(feedback)

    def move_object(self, start, end):
        # create feedback message
        feedback = String()
        # move to start location
        self.listener.waitForTransform(start.header.frame_id, "/base_link",  rospy.Time(0),rospy.Duration(4.0))
        start = self.listener.transformPoint("/base_link", start)
        start.point.z += 0.15
        #end = start

        resp = self.serv(start)
        # Should we try a few times? How do we fail?
        if resp.result == False:
            feedback.data = "Unable to move to start"
            self.cont_feedback_pub.publish(feedback)

        # grasp the object
        grab = String()
        grab.data = "grabbed"
        self.button_pub.publish(grab)

        # move to end location
        self.listener.waitForTransform(end.header.frame_id, "/base_link", rospy.Time(0),rospy.Duration(4.0))
        end = self.listener.transformPoint("/base_link", end)
        #end.point.z += 0.15

        resp = self.serv(end)
        if resp.result == False:
            feedback.data = "Unable to move to end"
            self.cont_feedback_pub.publish(feedback)

        # release the object
        release = String()
        release.data = "released"
        self.button_pub.publish(release)
        # TODO: maybe the grabbed and release can be object properties that this controller can do...

if __name__ == '__main__':
    controller = Controller()
