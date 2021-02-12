#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
import math


class DeadReckoning:
    def __init__(self, name):
        # Initialize node
        rospy.init_node(name)

        # Subscribe to topics
        rospy.Subscriber("/imu", Imu, self.imu_subscriber, queue_size=1)
        rospy.Subscriber("/mobile_bot/diff_drive_controller/odom", Odometry, self.odom_subscriber, queue_size=1)

    
    # Construct model matrix
    def diff_drive_model(self, theta): 
        F = np.zeros((3,2))             
        F[0,0] = math.cos(theta)        # [  cos(theta)  0  ]
        F[1,0] = math.sin(theta)        # [  sin(theta)  0  ]
        F[2,1] = 1                      # [      0       1  ]

        return F

    
    def imu_subscriber(self, data):

        # Store useful information from IMU sensor
        q = data.orientation
        w = data.angular_velocity
        a = data.linear_acceleration

        #rospy.loginfo("angular velocity: x: %.3f y: %.3f z: %.3f" % (w.x, w.y, w.z))



    def odom_subscriber(self, data):
        # Store useful information from Odometry

        v = data.twist.twist.linear
        rospy.loginfo("linear velocity: x: %.3f y: %.3f z: %.3f" % (v.x, v.y, v.z))
        pass


    # Uses IMU and Odometry data to estimate where the robot should be
    def estimate_pose(self):
        pass



if __name__ == "__main__":
    try:
        # Start the subscriber node
        dead_reck = DeadReckoning("dead_reckoning")

        while not rospy.is_shutdown():
            dead_reck.estimate_pose()
        

    except rospy.ROSInterruptException:
        pass