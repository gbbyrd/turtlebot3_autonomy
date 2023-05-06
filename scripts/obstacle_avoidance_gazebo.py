#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import numpy as np

"""This script allows the turtlebot to navigate wall following and obstacle avoidance.

Author(s): Grayson Byrd, Tongshen Huo, Parth Shinde, Saathana

Date: April 6, 2023
"""

class Wall_Follower():
    def __init__(self):
        # Nodes, publishers, subscribers
        rospy.init_node('wall_follower', anonymous=True)
        self.lidar_subscriber = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist)
        rospy.sleep(2)
        
        # Bot controls
        self.vel_msg = Twist()
        self.zero_controls()
        self.starting_vel = .2
        self.vel_msg.linear.x = self.starting_vel
        self.vel_publisher.publish(self.vel_msg)
        
        # Controller
        self.Kp = .1
        self.Ki = 0
        self.Kd = 0
        self.prev_error = 0
        self.I = 0
        
    def lidar_callback(self, msg):
        self.left = self.normalize_lidar(list(msg.ranges[-60:-1]))
        self.right = self.normalize_lidar(list(msg.ranges[0:60]))
    
    def sigmoid(self, input):
        if input > 3.5:
            return 1
        return 1 / (1 + np.exp(-input))
    
    def normalize_lidar(self, lidar_list):
        for idx, val in enumerate(lidar_list):
            normalized = self.sigmoid(val)
            normalized = 2* (normalized - .5)
            normalized = 1 - normalized
            lidar_list[idx] = normalized
            
        # the values now should be in between 0 and 1 with closer objects representing
        # a value closer to 1
        return lidar_list
        
    def avoid_obstacles(self):
        
        error = -(sum(self.right) - sum(self.left))
        
        # send error to PID controller
        angular_vel = self.pid_control(error)
        
        # rotate
        self.rotate(angular_vel)
        
    def rotate(self, angular_vel):
        self.zero_rotation()
        if angular_vel < 0:
            self.vel_msg.angular.z = angular_vel
        else:
            self.vel_msg.angular.z = angular_vel
        self.vel_publisher.publish(self.vel_msg)
        
    def zero_rotation(self):
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0
        self.vel_publisher.publish(self.vel_msg)
        
    def zero_controls(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0
        self.vel_publisher.publish(self.vel_msg)
        
        return
    
    def pid_control(self, error):
        P = error
        self.I = self.I + error
        D = error - self.prev_error
        self.prev_error = error
        
        # when tuning, it was found that the integral and derivate were not necessary
        total = P*self.Kp # + self.I*self.Ki + D*self.Kd
        return total
            
    def run(self):
        while not rospy.is_shutdown():
            self.avoid_obstacles()
    
if __name__=='__main__':
    OA = Wall_Follower()
    OA.run()