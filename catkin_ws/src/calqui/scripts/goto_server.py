#!/usr/bin/env python

import rospy
import actionlib
import numpy
import math

from dynamic_reconfigure.server import Server
from calqui.cfg import CalquiConfig

from calqui_msgs.msg import GoToPlaceAction
from calqui_msgs.msg import GoToPlaceGoal
from calqui_msgs.msg import GoToPlaceResult
from calqui_msgs.msg import GoToPlaceFeedback
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class GoToServer:
    def __init__(self):
        rospy.on_shutdown(self.on_shutdown)
        self._as = actionlib.SimpleActionServer('/goto_place', 
                GoToPlaceAction, execute_cb=self.on_goal, auto_start=False)
        self._vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self._fb_pub = rospy.Publisher("/goto_place/feedback", GoToPlaceFeedback, queue_size=10)
        self._cfgSrv = Server(CalquiConfig, self.cfg_callback)
        self._odom_sub = rospy.Subscriber("/odom", Odometry, self.on_odom)
        self._current_pose = Pose2D()
        self._goal_pose = Pose2D()
        self._speed = 0.5
        self._as.start()
        
        rospy.loginfo('Simple Action Server has been started')

    def cfg_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request:  {calqui_SPEED}""".format(**config))
        self._speed = config.calqui_SPEED

        return config

    def on_shutdown(self):
        rospy.loginfo('stopping vel')  
        self.set_vel(0,0)

    def on_odom(self, msg):
        global roll, pitch, yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self._current_pose.x = msg.pose.pose.position.x
        self._current_pose.y = msg.pose.pose.position.y
        self._current_pose.theta = yaw
        
        # rospy.loginfo('current pose is {}'.format(self._current_pose))
        fb = GoToPlaceFeedback()
        fb.distance_to_goal = self.distance_to_goal()
        self._fb_pub.publish(fb)
        

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

    def distance_to_goal(self):
        return math.sqrt(math.pow((self._goal_pose.y - self._current_pose.y),2) + math.pow((self._goal_pose.x - self._current_pose.x),2))

    # absolute angle to the goal
    def angle_to_goal(self):
        # avoid division by 0
        if self._current_pose.x == 0:
            return 0 
        else:
            return numpy.arctan2((self._goal_pose.y - self._current_pose.y), (self._goal_pose.x - self._current_pose.x))

    # how far are we of pointing to the goal?
    def deviation(self):
        return self._current_pose.theta - self.angle_to_goal()

    # do we consider that we reached the goal
    def goal_was_reached(self):
        return self._current_pose and self._goal_pose and self.distance_to_goal() < 0.5

    def calculate_angular_speed(self):
        if numpy.abs(self.deviation()) < 0.1 or self.goal_was_reached():
            rotation_speed = 0
        else:
            rotation_speed = self._speed * -0.5 * self.deviation()

        rospy.loginfo('rotation_speed {}'.format(rotation_speed))
        rospy.loginfo('angle_to_goal {}, current_angle {}'.format(self.angle_to_goal(), self._current_pose.theta))
        return rotation_speed

    def calculate_linear_speed(self, angular_speed):
        if self.goal_was_reached() or numpy.abs(self.deviation()) > 0.5:
            return 0
        return self._speed

    def calculate_vel(self):
        # rospy.loginfo('current pose {} -> goal {}'.format(self._current_pose, self._goal_pose))
        msg = Twist()
        angular_speed = self.calculate_angular_speed()
        linear_speed = self.calculate_linear_speed(angular_speed)
        msg.linear.x = linear_speed
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = angular_speed
        return msg

    def on_goal(self, goal):
        rospy.loginfo('a goal has been received! {}'.format(goal))

        self._goal_pose = Pose2D()
        self._goal_pose.x = goal.x
        self._goal_pose.y = goal.y
        self._goal_pose.theta = goal.theta

        rospy.loginfo(self._goal_pose)

        wait_duration = 1
        rate = rospy.Rate(1.0/wait_duration)

        while True:
            vel = self.calculate_vel()
            self.set_vel(vel.linear.x, vel.angular.z)
            rate.sleep()

        self.set_vel(0.0, 0.0)
        result = GoToPlaceResult()
        result.x = self._current_pose.x
        result.y = self._current_pose.y
        result.theta = self._current_pose.theta

        self._as.set_succeeded(result)

if __name__ == '__main__':
        rospy.init_node('goto_place_server')
        server = GoToServer()
        rospy.spin()
