#!/usr/bin/env python
import rospy
# from mobile_bot import ProximitySensor
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import numpy as np
from numpy.random import random


class VelocityController:
    def __init__(self, name):
        rospy.init_node(name)

        self.sub = rospy.Subscriber("/mobile_bot/proximity_sensor", Bool, self.proximity_callback)
        self.vel_pub = rospy.Publisher("/mobile_bot/diff_drive_controller/cmd_vel", Twist, queue_size=1)

        self.vel_msg = Twist()

    def proximity_callback(self, data):
        # If data == True --> Stop
        # Twist.linear.x = 0, Twist.angular.z = +/- 0.1

        # If data == False --> Go
        # Twist.linear.x = 0.3
        # Twist.angular.z = 0

        stop = data
        self.vel_msg = Twist()

        if stop.data == True:
            self.vel_msg.linear.x = 0

            # direction = self.random_direction()
            self.vel_msg.angular.z = 0.1
        else:
            self.vel_msg.linear.x = 0.3
            self.vel_msg.angular.z = 0

        self.vel_pub.publish(self.vel_msg)
    

    def random_direction(self):
        if random() < 0.5:
            return -1
        else:
            return 1

        

if __name__ == '__main__':
    walker = VelocityController("mobile_bot_random_walk")
    rospy.spin()


