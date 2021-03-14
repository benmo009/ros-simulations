#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np
import matplotlib.pyplot as plt
import math

class ProximitySensor:
    def __init__(self, name, tolerance=0.7):
        # Initialize ROS Node
        rospy.init_node(name)

        self.tolerance = tolerance

        # Start laser subscriber
        self.sub = rospy.Subscriber("/mobile_bot/laser/scan", LaserScan, self.laser_callback, queue_size=1)
        # Start publisher
        self.pub = rospy.Publisher("/mobile_bot/proximity_sensor", LaserScan, queue_size=1)

        # Stop flag
        self.stop = Bool()
        self.stop.data = False

    # Radian and degree conversion functions
    def radian_to_degree(self, angle):
        return 180*angle/math.pi

    def degree_to_radian(self, angle):
        return math.pi*angle/180

    # Callback for laser scan data
    def laser_callback(self, data):
        # Compute all laser scan angles
        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_increment = data.angle_increment

        ranges = np.array(data.ranges)

        angles = np.arange(angle_min, angle_max, angle_increment)

        cone_ang = 20
        angle_widths = np.zeros((4,2))
        angle_widths[0,:] = [0 - cone_ang/2, 0 + cone_ang/2]
        angle_widths[1,:] = [90 - cone_ang/2, 90 + cone_ang/2]
        angle_widths[2,:] = [-90 - cone_ang/2, -90 + cone_ang/2]
        angle_widths[3,:] = [180 - cone_ang/2, -180 + cone_ang/2]
        angle_widths = self.degree_to_radian(angle_widths)

        sensor_ranges = np.zeros((4,))


        # Check front of robot
        idx = np.argwhere(abs(angles) >= angle_widths[3,0])
        sensor_ranges[0] = np.mean(ranges[idx])

        # Check left of robot
        idx_lower = np.argwhere(angles >= angle_widths[2,0])
        idx_upper = np.argwhere(angles <= angle_widths[2,1])
        idx = np.intersect1d(idx_lower, idx_upper)
        sensor_ranges[1] = np.mean(ranges[idx])

#        Check back of robot
        idx = np.argwhere(abs(angles) <= angle_widths[0,1])
        sensor_ranges[2] = np.mean(ranges[idx])
        
        # Check right of robot
        idx_lower = np.argwhere(angles >= angle_widths[1,0])
        idx_upper = np.argwhere(angles <= angle_widths[1,1])
        idx = np.intersect1d(idx_lower, idx_upper)
        sensor_ranges[3] = np.mean(ranges[idx])

        pub_msg = LaserScan()
        pub_msg.angle_increment = math.pi / 2
        pub_msg.angle_min = -math.pi/2
        pub_msg.angle_max = math.pi
        pub_msg.ranges = sensor_ranges
    
        self.pub.publish(pub_msg)



if __name__ == "__main__":
    try:       
        laser_sub = ProximitySensor("mobile_bot_proximity_sensor")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
