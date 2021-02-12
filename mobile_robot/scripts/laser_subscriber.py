#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import math


def radian_to_degree(angle):
    return 180*angle/math.pi

def laser_callback(data):
    print(radian_to_degree(data.angle_min))


if __name__ == "__main__":
    try:
        rospy.init_node("Laser_Subscriber")
        sub = rospy.Subscriber("/mobile_bot/laser/scan", LaserScan, laser_callback, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
