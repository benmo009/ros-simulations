#!/usr/bin/env python

import sys
import rospy
import math
import numpy as np
from numpy.random import normal
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState


class NoisyGPS:
    def __init__(self, name, argv=[10, 0.1]):
        # Initialize ROS Node
        rospy.init_node(name)
        rospy.wait_for_service("/gazebo/get_model_state")

        self.rate = rospy.Rate(float(argv[1]))  # Rate to publish pose data
        self.noise_covariance = float(argv[2])  # Noise covariance of GPS
        self.noise_std_dev = math.sqrt(self.noise_covariance)

        self.noisy_pub = rospy.Publisher("/gps/noisy", Pose, queue_size=1)
        self.true_pub = rospy.Publisher("/gps/true", Pose, queue_size=1)

        self.noisy_pose = Pose()
        self.true_pose = Pose()
    
    def get_pose_and_pub(self):
        # Call the model state service
        get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        model_state = get_model_state("mobile_bot", "world")

        self.true_pose = model_state.pose
        self.add_noise()

        self.true_pub.publish(self.true_pose)
        self.noisy_pub.publish(self.noisy_pose)

    def add_noise(self):
        self.noisy_pose.position.x = self.true_pose.position.x + normal(0, self.noise_std_dev)
        self.noisy_pose.position.y = self.true_pose.position.y + normal(0, self.noise_std_dev)
        self.noisy_pose.position.z = self.true_pose.position.z + normal(0, self.noise_std_dev)
        self.noisy_pose.orientation.x = self.true_pose.orientation.x + normal(0, self.noise_std_dev)
        self.noisy_pose.orientation.y = self.true_pose.orientation.y + normal(0, self.noise_std_dev)
        self.noisy_pose.orientation.z = self.true_pose.orientation.z + normal(0, self.noise_std_dev)
        self.noisy_pose.orientation.w = self.true_pose.orientation.w + normal(0, self.noise_std_dev)

    def sleep(self):
        self.rate.sleep()


if __name__ == "__main__":
    myargv = rospy.myargv(sys.argv)
    try:
        gps = NoisyGPS("mobile_bot_gps", myargv)
        while not rospy.is_shutdown():
            gps.get_pose_and_pub()
            gps.sleep()
    except rospy.ROSInterruptException:
        pass