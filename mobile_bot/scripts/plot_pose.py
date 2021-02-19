#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion

class PosePlotter:
    def __init__(self, name):
        # Initialize ROS node
        rospy.init_node(name)

        # Subscribe to gps topics
        self.noisy_sub = rospy.Subscriber("/gps/noisy", Pose, self.noisy_gps_callback, queue_size=1)
        self.true_sub = rospy.Subscriber("/gps/true", Pose, self.true_gps_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber("/mobile_bot/dead_reckoning", Pose, self.odom_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber("/mobile_bot/range_sensor", Pose, self.range_callback, queue_size=1)

        # Initialize dictionaries for storing gps data
        self.noisy_data = {"t": [], "x": [], "y": [], "theta": []}
        self.true_data = {"t": [], "x": [], "y": [], "theta": []}
        self.odom_data = {"t": [], "x": [], "y": [], "theta": []}
        self.range_data = {"t": [], "x": [], "y": [], "theta": []}

    # Callback function for noisy GPS data
    def noisy_gps_callback(self, data):
        # Store the time, x, and y positions
        self.noisy_data["t"].append(rospy.get_time())
        self.noisy_data["x"].append(data.position.x)
        self.noisy_data["y"].append(data.position.y)

        # Convert quaternion data to euler angles to get orientation
        q = data.orientation
        theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        theta = theta[2]
        self.noisy_data["theta"].append(theta)

    # Callback function for true GPS data
    def true_gps_callback(self, data):
        # Store the time, x, and y positions
        self.true_data["t"].append(rospy.get_time())
        self.true_data["x"].append(data.position.x)
        self.true_data["y"].append(data.position.y)

        # Convert quaternion data to euler angles to get orientation
        q = data.orientation
        theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        theta = theta[2]
        self.true_data["theta"].append(theta)

    def odom_callback(self, data):
        self.odom_data["t"].append(rospy.get_time())
        self.odom_data["x"].append(data.position.x)
        self.odom_data["y"].append(data.position.y)
        self.odom_data["theta"].append(data.orientation.x)

    def range_callback(self, data):
        self.range_data["t"].append(rospy.get_time())
        self.range_data["x"].append(data.position.x)
        self.range_data["y"].append(data.position.y)
        self.range_data["theta"].append(data.orientation.x)

    # Plot the GPS data
    def plot(self):
        fig, ax = plt.subplots(3,1)
        ax[0].plot(self.true_data["t"], self.true_data["x"], label="True x")
        ax[1].plot(self.true_data["t"], self.true_data["y"], label="True y")
        ax[2].plot(self.true_data["t"], self.true_data["theta"], label="True $\\theta$")

        # ax[0].plot(self.noisy_data["t"], self.noisy_data["x"], '--', label="Noisy x")
        # ax[1].plot(self.noisy_data["t"], self.noisy_data["y"], '--', label="Noisy y")
        # ax[2].plot(self.noisy_data["t"], self.noisy_data["theta"], '--', label="Noisy $\\theta$")

        ax[0].plot(self.odom_data["t"], self.odom_data["x"], '--', label="Odom x")
        ax[1].plot(self.odom_data["t"], self.odom_data["y"], '--', label="Odom y")
        ax[2].plot(self.odom_data["t"], self.odom_data["theta"], '--', label="Odom $\\theta$")

        ax[0].plot(self.range_data["t"], self.range_data["x"], '--', label="Range x")
        ax[1].plot(self.range_data["t"], self.range_data["y"], '--', label="Range y")
        ax[2].plot(self.range_data["t"], self.range_data["theta"], '--', label="Range $\\theta$")

        for i in range(3):
            ax[i].legend()
            ax[i].set_xlabel("Time (s)")
            ax[i].set_ylabel("Position")

        plt.tight_layout()
        plt.show()
            
if __name__ == "__main__":
    try:
        # Get sampling time from parameter server
        if rospy.has_param("mobile_bot/plotter/sample_time"):
            sample_time = rospy.get_param("mobile_bot/plotter/sample_time")
        else:
            sample_time = 30

        # Start plotter node
        plotter = PosePlotter("mobile_bot_pose_plotter")

        # Define how long to collect data for. 
        rospy.sleep(sample_time)

        # Plot the GPS data
        plotter.plot()
    
    except rospy.ROSInterruptException:
        pass
