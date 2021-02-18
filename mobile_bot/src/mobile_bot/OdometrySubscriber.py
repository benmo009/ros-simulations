#!/usr/bin/env python

# Subscribe to /mobile_bot/diff_drive_controller/odom
# Get initial state from ros service
# Use odometry twist data to estimate input command
# Compute predicted next state using the unicycle model

# Publish estimated state to /mobile_bot/estimated_pose

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.srv import GetModelState
import numpy as np
import math

class OdometrySubscriber:
    def __init__(self, name):
        rospy.init_node(name)
        rospy.wait_for_service("/gazebo/get_model_state")

        # Subscribe to odometry
        self.sub = rospy.Subscriber("/mobile_bot/diff_drive_controller/odom", Odometry, self.odom_subscriber, queue_size=1)
        self.pub = rospy.Publisher("/mobile_bot/estimated_pose", Pose, queue_size=1)
    
        self.input_command = np.zeros((2,1))
        self.get_initial_state()

        self.prev_time = rospy.get_time()

    def get_initial_state(self):
        # Call the model state service
        get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        model_state = get_model_state("mobile_bot", "world")

        # Create state vector
        self.state = np.zeros((3,1))
        self.state[0] = model_state.pose.position.x
        self.state[1] = model_state.pose.position.y

        q = model_state.pose.orientation
        euler_angles = euler_from_quaternion([q.x, q.y, q.z, q.w])
        print(euler_angles)
        self.state[2] = euler_angles[2]


    # Callback function for odometry topic
    def odom_subscriber(self, data):
        self.input_command[0] = data.twist.twist.linear.x
        self.input_command[1] = data.twist.twist.angular.z
        
        model = self.diff_drive_model(self.state[2])

        delta_state = model @ self.input_command

        dt = rospy.get_time() - self.prev_time

        self.state = self.state + dt*delta_state

        self.prev_time = rospy.get_time()

        # Publish the estimated state
        self.publish_state()
    

    def publish_state(self):
        pose_msg = Pose()
        pose_msg.position.x = self.state[0]
        pose_msg.position.y = self.state[1]
        # Just set the x quaternion value to theta for now
        pose_msg.orientation.x = self.state[2]


        self.pub.publish(pose_msg)

    
    # Construct model matrix
    def diff_drive_model(self, theta): 
        F = np.zeros((3,2))             
        F[0,0] = math.cos(theta)        # [  cos(theta)  0  ]
        F[1,0] = math.sin(theta)        # [  sin(theta)  0  ]
        F[2,1] = 1                      # [      0       1  ]

        return F
