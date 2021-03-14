#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle

class CostmapSubscriber:
    def __init__(self):
        rospy.init_node("costmap_subscriber")
        rospy.Subscriber("/costmap", OccupancyGrid, self.map_callback)
        rospy.wait_for_message("/costmap", OccupancyGrid)
        
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
        
        self.pub = rospy.Publisher("/near_object", Bool, queue_size=0)
        self.rate = rospy.Rate(10)

        self.map_resolution = 0.05
        self.map_origin = np.array([25, 25])
        self.current_pose_on_map = np.array([25,25])
        self.costmap = np.zeros((200,200))

        self.object_detected = False

        while not rospy.is_shutdown():
            self.pub.publish(self.object_detected)
            self.rate.sleep()

    def map_callback(self, data):
        print(data.info)
        self.map_resolution = data.info.resolution

        self.map_origin = data.info.origin.position
        self.map_origin = np.array([self.map_origin.x, self.map_origin.y])

        self.map_origin = (-self.map_origin / self.map_resolution) - 1975

        self.costmap = data.data
        self.costmap = np.array(self.costmap)
        self.costmap.shape = (data.info.width, data.info.height)
        self.costmap = self.costmap[1975:2185, 1975:2185]
    
    def odom_callback(self, data):
        self.current_pose = data.pose.pose

        self.current_pose_on_map = np.array([self.current_pose.position.x, self.current_pose.position.y])

        self.current_pose_on_map =  self.current_pose_on_map / self.map_resolution
              
        self.current_pose_on_map = self.current_pose_on_map + self.map_origin
        self.current_pose_on_map = self.current_pose_on_map.astype(int)

        print(self.current_pose_on_map)

        print(self.costmap.shape)


            
        #print(self.current_pose_on_map)
        if self.costmap[self.current_pose_on_map[0], self.current_pose_on_map[1]] >= 50:
            self.object_detected = True
            
        else:
            self.object_detected = False

if __name__ == "__main__":    

    cmap_sub = CostmapSubscriber()
    

