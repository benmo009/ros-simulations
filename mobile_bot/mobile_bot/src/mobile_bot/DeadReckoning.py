#!/usr/bin/env python

# Subscribe to /mobile_bot/diff_drive_controller/odom
# Get initial state from ros service
# Use odometry twist data to estimate input command
# Compute predicted next state using the unicycle model

# Publish estimated state to /mobile_bot/estimated_pose

import rospy
import math
import numpy as np
from numpy.random import normal
from mobile_bot import ExtendedKalmanFilter as EKF
from mobile_bot import mobile_bot_helpers as mobile_bot
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose



class DeadReckoning:
    def __init__(self, name, noise_covariance):
        rospy.init_node(name)

        # Subscribe to odometry for its Twist data
        self.sub = rospy.Subscriber("/mobile_bot/diff_drive_controller/odom", Odometry, self.odom_subscriber, queue_size=1)
        self.pub = rospy.Publisher("/mobile_bot/dead_reckoning", Pose, queue_size=1)
    
        # Get noise parameters
        self.state_noise = noise_covariance
        self.vel_noise = np.eye(2)
        if rospy.has_param("/mobile_bot/diff_drive_controller/twist_covariance_diagonal"):
            twist_covar = rospy.get_param("/mobile_bot/diff_drive_controller/twist_covariance_diagonal")
            self.vel_noise[0,0] = twist_covar[0]
            self.vel_noise[1,1] = twist_covar[-1]
        
        # Get the robot's initial position
        self.state = mobile_bot.get_initial_state(self.state_noise)

        # Initialize input command vector
        self.input_command = np.zeros((2,1))

        # Initialize Kalman Filter
        self.dead_reck_ekf = EKF.ExtendedKalmanFilter(mobile_bot.A,  # A matrix
                                                    mobile_bot.B,   # B
                                                    self.C,  # C (zeros)
                                                    mobile_bot.f,  # f
                                                    self.h,  # h
                                                    np.zeros((1,1)),  # V
                                                    self.vel_noise,  # W
                                                    self.state_noise, # P0
                                                    self.state) # q0
        
        # Set update rate
        self.update_rate = rospy.Rate(1)  # Update the state at 1 Hz


    # Gets the initial state while adding noise
    def get_initial_state(self):
        # Call the model state service
        get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        model_state = get_model_state("mobile_bot", "world")
        model_pose =model_state.pose

        # Create state vector
        self.state = np.zeros((3,1))

        # Add x and y positions with noise
        self.state[0] = model_pose.position.x + normal(0,self.noise_std_dev[0,0])
        self.state[1] = model_pose.position.y + normal(0,self.noise_std_dev[1,1])
        
        # Get the orientation in euler angle
        q = model_pose.orientation
        euler_angles = euler_from_quaternion([q.x, q.y, q.z, q.w])
        # Add the orientation with noise
        self.state[2] = euler_angles[2] + normal(0,self.noise_std_dev[2,2])


    # Callback function for odometry topic. Every time it gets new odometry
    # data, save the linear velocity in x and angular velocity in z. 
    def odom_subscriber(self, data):
        self.input_command[0] = data.twist.twist.linear.x
        self.input_command[1] = data.twist.twist.angular.z

        # Update the twist covariance
        twist_covar = data.twist.covariance
        self.vel_noise[0,0] = twist_covar[0]
        self.vel_noise[1,1] = twist_covar[-1]
        self.dead_reck_ekf.W = self.vel_noise
       

    def estimate_and_publish_state(self):
        # Update state estimate using extended kalman filter
        self.state = self.dead_reck_ekf.estimate(0, self.input_command)
        print(self.state)
        pose_msg = Pose()
        pose_msg.position.x = self.state[0]
        pose_msg.position.y = self.state[1]
        # Just set the x quaternion value to theta for now
        pose_msg.orientation.x = self.state[2]

        # Publish the estimate
        self.pub.publish(pose_msg)

        self.update_rate.sleep()


    def C(self, q):
        return np.zeros((1,3))

    def h(self, q):
        return 0
