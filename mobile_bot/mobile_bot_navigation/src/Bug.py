#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool
import math
import numpy as np
from enum import Enum
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker

class Bug:
    def __init__(self):
        # Initialize ros node and any subscribers/publishers needed
        rospy.init_node("bug1_node")

        self.map_sub = rospy.Subscriber("/near_object", Bool, self.map_callback)
        self.odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
        self.prox_sub = rospy.Subscriber("/mobile_bot/proximity_sensor", LaserScan, self.prox_callback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=0)
        self.goal_marker_pub = rospy.Publisher("/goal_marker", Marker, queue_size=0)
        rospy.wait_for_message("/odometry/filtered", Odometry)


        # Initialize states
        self.states = Enum("STATES", "QUERY_GOAL TURN DRIVE FOLLOW_OBSTACLE")
        self.current_state = self.states.QUERY_GOAL

        self.near_object = False
        self.object_detected = False
        self.goal_reached = False
   
        # Run the main loop
        self.main_loop()
    

    def main_loop(self):
        while True:

            if self.current_state == self.states.QUERY_GOAL:
                self.get_goal()

            elif self.current_state == self.states.TURN:
                print("turning towards goal")
                self.turn_towards_goal()
                
            elif self.current_state == self.states.DRIVE:
                print("driving towards goal")
                self.drive_towards_goal()
                print(self.near_object)
            
                if self.near_object:
                    self.stop()
                    self.current_state = self.states.FOLLOW_OBSTACLE

                elif self.goal_reached == True:
                    self.stop()
                    self.current_state = self.states.QUERY_GOAL

            elif self.current_state == self.states.FOLLOW_OBSTACLE:
                print("Follow Obstacle")
                while self.left_range >= 0.6:
                    vel_msg = Twist()
                    vel_msg.angular.z = -0.1
                    self.vel_pub.publish(vel_msg)
                self.stop()
                self.follow_obstacle()


        # Publish marker for rviz
        goal_marker = Marker()
        goal_marker.type = Marker.ARROW
        goal_marker.action = Marker.ADD
        goal_marker.pose.orientation.y = math.sqrt(2) / 2
        goal_marker.pose.orientation.w = math.sqrt(2) / 2
        goal_marker.pose.position = self.goal.position
        goal_marker.pose.position.z = 0.5
        goal_marker.scale.x = 0.5
        goal_marker.scale.y = 0.1
        goal_marker.scale.z = 0.1

        goal_marker.header.frame_id = 'map'


        self.goal_marker_pub.publish(goal_marker)


        rospy.spin()
                

    def stop(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        self.vel_pub.publish(vel_msg)

    def prox_callback(self, data):
        self.left_range = data.ranges[1]

    def map_callback(self, data):
        self.near_object = data.data

    def odom_callback(self, data):
        self.current_pose = data.pose.pose

    def get_goal(self):
        # Get user input goal
        goal_x = float(input("Enter desired x location: "))
        goal_y = float(input("Enter desired y location: "))


        print("Desired goal: (%.2f, %.2f)" % (goal_x, goal_y))

        # Put in form of pose
        self.goal = Pose()
        self.goal.position.x = goal_x
        self.goal.position.y = goal_y

        # Compute m line

        # Publish marker for rviz
        goal_marker = Marker()
        goal_marker.type = Marker.ARROW
        goal_marker.action = Marker.ADD
        goal_marker.pose.orientation.y = math.sqrt(2) / 2
        goal_marker.pose.orientation.w = math.sqrt(2) / 2
        goal_marker.pose.position = self.goal.position
        goal_marker.pose.position.z = 0.5
        goal_marker.scale.x = 0.5
        goal_marker.scale.y = 0.1
        goal_marker.scale.z = 0.1

        goal_marker.header.frame_id = 'map'


        self.goal_marker_pub.publish(goal_marker)

        # Change state to turn
        self.current_state = self.states.TURN

    def turn_towards_goal(self):
        # Use current pose
        q = self.current_pose.orientation
        current_angle = euler_from_quaternion([q.x, q.y, q.z, q.w])
        current_angle = current_angle[2]

        goal_x = self.goal.position.x
        goal_y = self.goal.position.y

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        angle_to_goal = math.atan2(goal_y - current_y, goal_x - current_x)
        print(angle_to_goal*180/math.pi)

        while abs(current_angle - angle_to_goal) >= 0.01:
            # Publish to cmd vel

            vel_msg = Twist()
            vel_msg.angular.z = 0.2
            self.vel_pub.publish(vel_msg)

            # Update current_angle
            q = self.current_pose.orientation
            current_angle = euler_from_quaternion([q.x, q.y, q.z, q.w])
            current_angle = current_angle[2]
            print(current_angle, angle_to_goal)


        # Turn until 

        self.current_state = self.states.DRIVE

    def drive_towards_goal(self):
        # Drive until obstacle is encountered or goal is reached
        vel_msg = Twist()
        vel_msg.linear.x = 0.1
        self.vel_pub.publish(vel_msg)


    def follow_obstacle(self):
        starting_point = self.current_pose
        starting_point = np.array([starting_point.position.x, starting_point.position.y])
        g = np.array([self.goal.position.x, self.goal.position.y])

        distance_from_start = 100
        min_dist = np.linalg.norm(starting_point - g)
        min_point = starting_point
        

        while distance_from_start > 0.1:
            if self.left_range > 0.6:
                vel_msg = Twist()
                vel_msg.angular.z = 0.1
                self.vel_pub.publish(vel_msg)
            else:
                vel_msg = Twist()
                vel_msg.linear.x = 0.1
                self.vel_pub.publish(vel_msg)

            # Get current_point
            current_point = self.current_pose
            current_point = np.array([self.current_pose.position.x, self.current_pose.position.y])

            distance_from_start = np.linalg.norm(current_point - starting_point)
            distance_from_goal = np.linalg.norm(current_point - g)
            if min_dist >= distance_from_goal:
                min_dist = distance_from_goal
                min_point = current_point
            
        
        # Go back



if __name__ == "__main__":
    bug = Bug()