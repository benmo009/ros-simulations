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

class Bug:
    def __init__(self):
        # Initialize ros node and any subscribers/publishers needed
        rospy.init_node("bug1_node")

        self.map_sub = rospy.Subscriber("/near_object", Bool, self.map_callback)
        self.odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
        self.prox_sub = rospy.Subscriber("/mobile_bot/proximity_sensor", LaserScan, self.prox_callback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=0)
        rospy.wait_for_message("/odometry/filtered", Odometry)


        # Initialize states
        self.states = Enum("STATES", "QUERY_GOAL TURN DRIVE FOLLOW_OBSTACLE")
        self.current_state = self.states.QUERY_GOAL

        self.near_object = False
        self.object_detected = False
        self.goal_reached = False

        self.right_at_obstacle = True
   
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
                self.turn_for_obstacle()
                self.stop()
                self.follow_obstacle()

        rospy.spin()
                

    def stop(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        self.vel_pub.publish(vel_msg)

    def prox_callback(self, data):
        self.left_range = data.ranges[1]
        self.right_range = data.ranges[3]

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

    
    # Turns right or left until parallel to obstacle
    def turn_for_obstacle(self):
        # Turn right 
        if self.right_at_obstacle:
            while self.left_range >= 0.6:
                    vel_msg = Twist()
                    vel_msg.angular.z = -0.1
                    self.vel_pub.publish(vel_msg)

        # Turn left
        else:
            while self.right_range >= 0.6:
                    vel_msg = Twist()
                    vel_msg.angular.z = 0.1
                    self.vel_pub.publish(vel_msg)


    # Follows the obstacle
    def follow_obstacle(self):
        # Get current angle
        q = self.current_pose.orientation
        current_angle = euler_from_quaternion([q.x, q.y, q.z, q.w])
        current_angle = current_angle[2]

        # Get current robot pose
        current = self.current_pose.position
        goal = self.goal.position
 
        # Follow obstacle until the angle to the goal matches the starting angle
        tol = 0.01
        angle_to_goal = math.atan2(goal.y - current.y, goal.x - current.x)
        count = 0  # Add a small count so the loop doesn't break too early

        if self.right_at_obstacle:
            range_sensor = self.left_range
            factor = 1
        else:
            range_sensor = self.right_range
            factor = -1

        while abs(current_angle - angle_to_goal) > tol or count < 3:
            if range_sensor > 0.6:
                vel_msg = Twist()
                vel_msg.angular.z = factor * 0.1
                self.vel_pub.publish(vel_msg)
            else:
                vel_msg = Twist()
                vel_msg.linear.x = 0.1
                self.vel_pub.publish(vel_msg)

            # Get current angle
            q = self.current_pose.orientation
            current_angle = euler_from_quaternion([q.x, q.y, q.z, q.w])
            current_angle = current_angle[2]
            current = self.current_pose.
            
            # Measure angle to goal
            current_angle = math.atan2(angle_to_goal = math.atan2(goal.y - current.y, goal.x - current.x))
            count += 1

        self.current_state = self.states.DRIVE

        # Turn at the opposite direction next time
        self.right_at_obstacle = not self.right_at_obstacle


if __name__ == "__main__":
    bug = Bug()