#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.srv import GetModelState
import numpy as np
import math
from tf.transformations import euler_from_quaternion


class DeadReckoning:
    def __init__(self, name):
        # Initialize node
        rospy.init_node(name)

        # Subscribe to topics
        rospy.Subscriber("/imu", Imu, self.imu_subscriber, queue_size=1)
        rospy.Subscriber("/mobile_bot/diff_drive_controller/odom", Odometry, self.odom_subscriber, queue_size=1)

        # Get the starting position
        rospy.wait_for_service("/gazebo/get_model_state")
        get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        model_state = get_model_state("mobile_bot", "world")
        start_pose = model_state.pose

        #print("="*12, "Starting Pose:")
        #print("   Position: (%.3f, %.3f, %.3f)" %(self.start_pose.position.x, self.start_pose.position.y, self.start_pose.position.z))
        #print("Orientation: %.3fi + %.3fj + %.3fk + %.3f" % (self.start_pose.orientation.x, self.start_pose.orientation.y, self.start_pose.orientation.z, self.start_pose.orientation.w))
        self.current_pose = start_pose
        self.current_twist = model_state.twist

        self.current_time = rospy.get_time()
        self.prev_time_imu = rospy.get_time()
        self.prev_time_odom = rospy.get_time()
        self.prev_time_est_pose = rospy.get_time()

        q = start_pose.orientation
        q = [q.x, q.y, q.z, q.w]
        euler_ang = euler_from_quaternion(q)
        theta = euler_ang[2]
        self.current_state = np.zeros((3,1))
        self.current_state[0] = self.current_pose.position.x
        self.current_state[1] = self.current_pose.position.y
        self.current_state[2] = theta


    
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
        self.current_twist.angular = data.angular_velocity
        accel = data.linear_acceleration

        # Estimate linear velocity based on linear acceleration
        self.current_twist.linear.x = accel.x * (rospy.get_time() - self.prev_time_imu) + self.current_twist.linear.x
        print("Estimated linear velocity: %.3f\t angular velocity: %.3f" % (self.current_twist.linear.x, self.current_twist.angular.z))

        # Update the time
        self.prev_time_imu = rospy.get_time()

        # Estimate the new pose using the model

        # Average the estimated pose orientation with the IMU orientation

        



    def odom_subscriber(self, data):
        # Store useful information from Odometry

        v = data.twist.twist.linear
        #rospy.loginfo("linear velocity: x: %.3f y: %.3f z: %.3f" % (v.x, v.y, v.z))
        pass


    # Uses IMU and Odometry data to estimate where the robot should be
    def estimate_pose(self):
        # Estimate the pose
        q = [self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w]
        euler_ang = euler_from_quaternion(q)
        theta = euler_ang[2]

        F = self.diff_drive_model(theta)
        u = np.zeros((2,1))
        u[0] = self.current_twist.linear.x
        u[1] = self.current_twist.angular.z

        q_dot = F @ u



        # Publish the estimated pose on set frequency

        pass



if __name__ == "__main__":
    try:
        # Start the subscriber node
        dead_reck = DeadReckoning("dead_reckoning")

        while not rospy.is_shutdown():
            dead_reck.estimate_pose()
        

    except rospy.ROSInterruptException:
        pass