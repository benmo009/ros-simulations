#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from numpy.random import normal


class DiffDriveControl:
    def __init__(self, name, noise_cov):
        rospy.init_node(name)
        
        self.pub = rospy.Publisher("/mobile_bot/diff_drive_controller/cmd_vel", Twist, queue_size=0)
        self.sub = rospy.Subscriber("/cmd_vel", Twist, self.callback)

        self.noise_cov = noise_cov


    def callback(self, data):
        # Take the data, add some noise, push it 
        data.linear.x += normal(0, self.noise_cov)
        data.angular.z += normal(0, self.noise_cov)

        self.pub.publish(data)
