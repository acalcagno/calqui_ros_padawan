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
        self._reset_flags()
        self._as = actionlib.SimpleActionServer('/goto_place', 
                GoToPlaceAction, execute_cb=self.on_goal, auto_start=False)
        self._vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self._cfgSrv = Server(CalquiConfig, self.cfg_callback)
        self._odom_sub = rospy.Subscriber("/odom", Odometry, self.on_odom)
        self._current_pose = Pose2D()
        self._goal_pose = Pose2D()
        self._speed = 0.5
        self._as.start()
        
        rospy.loginfo('Simple Action Server has been started')

    def _reset_flags(self):
        self._rotating = False
        self._going_forward = False
        self._goal_position_reached = False


    def cfg_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request:  {calqui_SPEED}""".format(**config))
        self._speed = config.calqui_SPEED
        return config

    def on_shutdown(self):
        rospy.loginfo('on shutdown')  
        self.set_vel(0,0)

    def on_odom(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self._current_pose.x = msg.pose.pose.position.x
        self._current_pose.y = msg.pose.pose.position.y
        self._current_pose.theta = yaw

    def set_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = angular_z
        self.log_angular_speed(angular_z)
        self.log_linear_speed(linear_x)
        self._vel_pub.publish(msg)

    def send_feedback(self):
        fb = GoToPlaceFeedback()
        fb.distance_to_goal = self.distance_to_goal()
        self._as.publish_feedback(fb)

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

    def _log_goal_position_reached(self, reached):
        if not reached:
            return
        if not self._goal_position_reached:
            self._goal_position_reached = True
            rospy.loginfo('goal position reached, rotating to goal orientation')

    # do we consider that goal succeeded
    def goal_was_reached(self):
        return self.goal_position_reached() and self.angle_matches_goal_angle()

    # we are in the position of the goal (don't know about the goal angle)
    def goal_position_reached(self):
        reached = self._current_pose and self._goal_pose and self.distance_to_goal() < 0.5
        self._log_goal_position_reached(reached)
        return reached

    def calculate_angular_speed(self):
        # when goal has succedded, stop
        if self.goal_was_reached():
            rospy.loginfo('Goal completed')  
            return 0

        # when not close to the goal position
        if not self.goal_position_reached():
            # and robot is facing the goal position, don't rotate
            if numpy.abs(self.deviation()) < 0.1:
                return 0
            else:
                # robot is not close to goal position, and also not facing it, correct the angle
                return self._speed * -0.5 * self.deviation()

        # robot is in the right spot, and with the correct angle. Stop
        if self.angle_matches_goal_angle():
            return 0

        # robot is in the right spot, but with incorrect angle
        return self._speed * -0.5

    def angle_matches_goal_angle(self):
        return numpy.abs(self._goal_pose.theta - self._current_pose.theta) < 0.2

    def calculate_linear_speed(self):
        if self.goal_position_reached() or numpy.abs(self.deviation()) > 0.5:
            return 0
        return self._speed

    def log_linear_speed(self, linear_speed):
        if linear_speed == 0:
            if self._going_forward:
                rospy.loginfo('stopped going forward')
            self._going_forward = False
        else:
            if not self._going_forward:
                rospy.loginfo('started going forward')
            self._going_forward = True

    def log_angular_speed(self, angular_speed):
        if angular_speed == 0:
            if self._rotating:
                rospy.loginfo('stopping rotation')
            self._rotating = False
        else:
            if not self._rotating:
                if angular_speed < 0:
                    rospy.loginfo('starting rotation right')
                else:
                    rospy.loginfo('starting rotation left')
            self._rotating = True

    def calculate_vel(self):
        msg = Twist()
        angular_speed = self.calculate_angular_speed()
        linear_speed = self.calculate_linear_speed()
        msg.linear.x = linear_speed
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = angular_speed
        return msg

    def on_goal(self, goal):
        rospy.loginfo('a goal has been received! {}'.format(goal))
        self._reset_flags()

        self._goal_pose = Pose2D()
        self._goal_pose.x = goal.x
        self._goal_pose.y = goal.y
        self._goal_pose.theta = goal.theta

        rospy.loginfo(self._goal_pose)

        wait_duration = 1
        rate = rospy.Rate(1.0/wait_duration)

        while not self.goal_was_reached():
            vel = self.calculate_vel()
            self.set_vel(vel.linear.x, vel.angular.z)
            self.send_feedback()
            rate.sleep()


        rospy.loginfo('Program finished!')
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
