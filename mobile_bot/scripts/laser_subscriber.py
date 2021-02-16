#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import math

class LaserSubscriber:
    def __init__(self, name):
        rospy.init_node(name)
        self.sub = rospy.Subscriber("/mobile_bot/laser/scan", LaserScan, self.laser_callback, queue_size=1)

    
    def radian_to_degree(self, angle):
        return 180*angle/math.pi

    def degree_to_radian(self, angle):
        return math.pi*angle/180

    def laser_callback(self, data):
        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_increment = data.angle_increment

        angles = np.arange(angle_min, angle_max, angle_increment)

        laser_range = self.degree_to_radian(120)

        idx_upper = np.argwhere(angles > laser_range)
        idx_lower = np.argwhere(angles < -laser_range)
        idx = np.concatenate( (idx_lower, idx_upper), axis=0)

        ranges = np.array(data.ranges)

        distances = ranges[idx]

        proximity = np.argwhere(distances <= 0.2)

        if proximity.size > 0:
            print("\nTOO CLOSE!!\n")



if __name__ == "__main__":
    try:       
        laser_sub = LaserSubscriber("Laser_Subscriber")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
