#!/usr/bin/env python

import rospy
import actionlib

from calqui_msgs.msg import GoToPlaceAction
from calqui_msgs.msg import GoToPlaceGoal
from calqui_msgs.msg import GoToPlaceResult
from geometry_msgs.msg import Twist

class GoToServer:
    def __init__(self):
        rospy.on_shutdown(self.on_shutdown)
        self._as = actionlib.SimpleActionServer('/goto_place', 
                GoToPlaceAction, execute_cb=self.on_goal, auto_start=False)
        self._vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self._as.start()
        
        rospy.loginfo('Simple Action Server has been started')

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
        self._vel_pub.publish(msg)

    def on_goal(self, goal):
        rospy.loginfo('a goal has been received!')
        rospy.loginfo(goal)

        wait_duration = 1
        rate = rospy.Rate(1.0/wait_duration)

        self._count = 0
        while self._count < 4:
            rospy.loginfo('setting vel')
            self._count = self._count + 1
            self.set_vel(0.1, 0.1)
            rate.sleep()

        result = GoToPlaceResult()
        result.x = 1
        result.y = 2
        result.theta = 3

        self._as.set_succeeded(result)

if __name__ == '__main__':
        rospy.init_node('goto_place_server')

        server = GoToServer()

        rospy.spin()
