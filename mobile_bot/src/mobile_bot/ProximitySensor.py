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
        self.pub = rospy.Publisher("/mobile_bot/proximity_sensor", Bool, queue_size=1)

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

        angles = np.arange(angle_min, angle_max, angle_increment)

        # Range of laser angles to check for proximity
        laser_range = self.degree_to_radian(120)

        # Check if the lasers point forwards are too close to an object
        idx_upper = np.argwhere(angles > laser_range)
        idx_lower = np.argwhere(angles < -laser_range)
        idx = np.concatenate( (idx_lower, idx_upper), axis=0)

        ranges = np.array(data.ranges)

        distances = ranges[idx]

        proximity = np.argwhere(distances <= self.tolerance)

        # Set stop flag 
        if proximity.size > 0:
            print("\nTOO CLOSE!!\n")
            self.stop.data = True
        else:
            print("You're good, keep going")
            self.stop.data = False

        self.pub.publish(self.stop)

    def check_stop(self):
        return self.stop



if __name__ == "__main__":
    try:       
        laser_sub = ProximitySensor("mobile_bot_proximity_sensor")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
