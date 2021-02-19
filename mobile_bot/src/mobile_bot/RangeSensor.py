#!/usr/bin/env python

# Subscribe to gps topic
# compute robot's relative pose to room corners (add a little bit of noise)
# Do range sensor math (subproblems)
# Publish the estimated pose

# Range sensor kalman filter
# Initialization:
#   get initial state and error covariance
# Compute 

import rospy
import math
import numpy as np
from numpy.random import normal
from geometry_msgs.msg import Pose
from mobile_bot import ExtendedKalmanFilter as EKF 
from mobile_bot import mobile_bot_helpers as mobile_bot
from nav_msgs.msg import Odometry

class RangeSensor:
    def __init__(self, name, position, sensor_noise):
        # Initialize ros node
        rospy.init_node(name)

        # Subscribe to gps topic to get robot's position
        self.gps_sub = rospy.Subscriber("/gps/true", Pose, self.gps_callback)
        # Subscribe to differential drive controller odometry to get input command
        self.vel_sub = rospy.Subscriber("/mobile_bot/diff_drive_controller/odom", Odometry, self.odom_subscriber, queue_size=1)
        # Publish kalman filter results
        self.pub = rospy.Publisher("/mobile_bot/range_sensor", Pose, queue_size=1)

        # Store range sensor position
        self.sensor_pos = np.array(position)
        self.sensor_pos.shape = (2,1)

        # Get noise parameters
        self.state_noise = 0.1*np.eye(3)  # P
        self.vel_noise = np.eye(2)  # W
        if rospy.has_param("/mobile_bot/diff_drive_controller/twist_covariance_diagonal"):
            twist_covar = rospy.get_param("/mobile_bot/diff_drive_controller/twist_covariance_diagonal")
            self.vel_noise[0,0] = twist_covar[0]
            self.vel_noise[1,1] = twist_covar[-1]

        self.sensor_noise = sensor_noise * np.eye(1)  # V
        self.noise_std_dev = math.sqrt(sensor_noise)

        # Get the robot's initial position
        self.state = mobile_bot.get_initial_state()
        print("Initial State:", self.state)
        # Initialize input command vector
        self.input_command = np.zeros((2,1))

        # Initialize sensor data
        self.sensor_data = 0

        # Initialize Kalman Filter
        self.range_ekf = EKF.ExtendedKalmanFilter(mobile_bot.A,     # A
                                                mobile_bot.B,       # B
                                                self.C,             # C
                                                mobile_bot.f,       # f
                                                self.h,             # h
                                                self.sensor_noise,  # V
                                                self.vel_noise,     # W
                                                self.state_noise,   # P0
                                                self.state)         # q0

        self.update_rate = rospy.Rate(1)  # Update state on 1 Hz


    # Linearized output model
    def C(self, q):
        Ck = np.zeros((1,3))

        x_range = self.sensor_pos[0]
        y_range = self.sensor_pos[1]
        vec_to_robot = self.sensor_pos - q[0:2]

        p = -vec_to_robot / np.linalg.norm(vec_to_robot)
        p.shape = (2,)
        Ck[0, 0:2] = p
        return Ck

    # Compute the distance of range sensor to q
    def h(self, q):
        return np.linalg.norm(self.sensor_pos - q[0:2])
        
    # Gets range data
    def gps_callback(self, data):
        # Get the robot's x and y position
        robot_pose = np.zeros((3,1))
        robot_pose[0] = data.position.x 
        robot_pose[1] = data.position.y 

        # Update the sensor data with noise
        self.sensor_data = self.h(robot_pose)
        self.sensor_data += normal(0, self.noise_std_dev)
        

    
    # Stores input command from odometry topic
    def odom_subscriber(self, data):
        # Store the twist data
        self.input_command[0] = data.twist.twist.linear.x
        self.input_command[1] = data.twist.twist.angular.z

        # # Update the twist covariance
        # twist_covar = data.twist.covariance
        # self.vel_noise[0,0] = twist_covar[0]
        # self.vel_noise[1,1] = twist_covar[-1]
        # #self.dead_reck_ekf.W = self.vel_noise


    def estimate_and_publish_state(self):
        # Update state estimate using extended kalman filter
        self.state = self.range_ekf.estimate(self.sensor_data, self.input_command)
        print(self.state)

        pose_msg = Pose()
        pose_msg.position.x = self.state[0]
        pose_msg.position.y = self.state[1]
        # Just set the x quaternion value to theta for now
        pose_msg.orientation.x = self.state[2]

        # Publish the estimate
        self.pub.publish(pose_msg)

        self.update_rate.sleep()
    

if __name__ == "__main__":
    try:
        range_sensor = RangeSensor("range_sensor")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass