#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class SimplePublisher:

    def __init__(self):
        rospy.init_node('circles_node')
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo('circles_node has been started')
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def on_shutdown(self):
        rospy.loginfo('stopping vel')  
        self.set_vel(0,0)

    def set_vel(self, linear_x, angular_z):
        rospy.loginfo('publishing vel')
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = angular_z
        self.pub.publish(msg)

    def start(self):
        try:   
            while not rospy.is_shutdown():
                self.set_vel(0.1, 0.1)
                self.rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo('rospy is shutdown is {}'.format(rospy.is_shutdown()))
            rospy.loginfo('process finished')

if __name__ == '__main__':
    simplePublisher = SimplePublisher()
    simplePublisher.start()

