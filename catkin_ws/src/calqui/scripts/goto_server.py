#!/usr/bin/env python

import rospy
import actionlib
import numpy
import math

from calqui_msgs.msg import GoToPlaceAction
from calqui_msgs.msg import GoToPlaceGoal
from calqui_msgs.msg import GoToPlaceResult
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
        self._odom_sub = rospy.Subscriber("/odom", Odometry, self.on_odom)
        self._current_pose = Pose2D()
        self._as.start()
        
        rospy.loginfo('Simple Action Server has been started')

    def on_shutdown(self):
        rospy.loginfo('stopping vel')  
        self.set_vel(0,0)

    def on_odom(self, msg):
        global roll, pitch, yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        # rospy.loginfo('yaw:{}, x:{} y:{} z:{}'.format(round(yaw,2), round(msg.pose.pose.position.x,2),round(msg.pose.pose.position.y,2),round(msg.pose.pose.position.z,2) ))
        self._current_pose.x = msg.pose.pose.position.x
        self._current_pose.y = msg.pose.pose.position.y
        self._current_pose.theta = yaw
        # rospy.loginfo('current pose is {}'.format(self._current_pose))

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

    def calculate_angular_speed(self):
        # avoid division by 0
        if self._current_pose.x == 0:
            angle_to_goal = 0 
        else:
            angle_to_goal = numpy.arctan((self._goal_pose.y - self._current_pose.y) / (self._goal_pose.x - self._current_pose.x))
        
        current_angle = self._current_pose.theta

        if current_angle < angle_to_goal + 0.1 and current_angle > angle_to_goal - 0.1:
            rotation_speed = 0
        else:
            rotation_speed = 0.1

        rospy.loginfo('rotation_speed {}'.format(rotation_speed))
        rospy.loginfo('angle_to_goal {}, current_angle {}'.format(angle_to_goal, current_angle))
        return rotation_speed

    def calculate_linear_speed(self, angular_speed):
        if angular_speed:
            return 0
        distnace_to_goal = self.distance_to_goal()
        if distnace_to_goal < 0.5:
            return 0
        return -0.1

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

        self._count = 0
        while True:
            self._count = self._count + 1
            vel = self.calculate_vel()
            self.set_vel(vel.linear.x, vel.angular.z)
            rate.sleep()

        self.set_vel(0.0, 0.0)
        result = GoToPlaceResult()
        result.x = 1
        result.y = 2
        result.theta = 3

        self._as.set_succeeded(result)

if __name__ == '__main__':
        rospy.init_node('goto_place_server')

        server = GoToServer()

        rospy.spin()
