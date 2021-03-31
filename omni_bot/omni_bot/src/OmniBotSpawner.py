#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import numpy as np
from numpy.random import normal

import argparse as ap

import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt

class OmniBotSpawner:
    def __init__(self, N):
        # Number of robots to spawn
        self.N = N
        self.variance = N / 2

        # Initialize ROS Node and spawn_urdf client
        rospy.init_node("robot_spawner")
        rospy.wait_for_service("gazebo/spawn_urdf_model")
        self.spawn_urdf = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)

        rospy.loginfo("Spawning %d Omni-bots"  % args.N)

        # Get the robot description from the parameter server
        self.model_xml = rospy.get_param("robot_description")

        # Add some code to make sure there are no overlaps??
        self.get_body_radius()  # Get the radius of the robot

        # Generate the origins for each robot 
        self.generate_origins(1)

        # Spawn the robots
        self.spawn_robots()

        # Load the origins to ros parameter server
        origin_param = self.origins.tolist()
        rospy.set_param('omni_bot/origins', origin_param)

    # Parses the omni-bot urdf to get the robot's radius
    def get_body_radius(self):
        root = ET.fromstring(self.model_xml)

        # Default robot radius
        self.robot_radius = 0

        for child in root:
            if child.tag == "footprint":
                self.robot_radius = float( child.attrib['value'] )
                return

    # Plot the spawn origins to check for overlapping spawns 
    def plot_points(self):
        plt.close('all')
        fig, ax = plt.subplots(1)

        for i in range(self.N):
            ax.scatter(self.origins[i,0], self.origins[i,1], c='black')
            ax.add_patch(plt.Circle(self.origins[i,:], radius=self.robot_radius, 
                                    fc='red', ec='black', alpha=0.5))

        # Set axis limits
        x_max = np.max( self.origins[:,0] ) + self.robot_radius
        x_min = np.min( self.origins[:,0] ) - self.robot_radius
        y_max = np.max( self.origins[:,1] ) + self.robot_radius
        y_min = np.min( self.origins[:,1] ) - self.robot_radius

        x_dist = x_max - x_min
        y_dist = y_max - y_min

        if x_dist > y_dist:
            y_max += (x_dist - y_dist) / 2
            y_min -= (x_dist - y_dist) / 2
        else:
            x_max += (y_dist - x_dist) / 2
            x_min -= (y_dist - x_dist) / 2
        
        x_lim = 1.1 * np.array([x_min, x_max])
        y_lim = 1.1 * np.array([y_min, y_max])

        ax.set_xlim( x_lim ) 
        ax.set_ylim( y_lim )

        ax.set_aspect('equal', adjustable='box')

        plt.show(block=False)
        

    # Generates random origins for each robot
    def generate_origins(self, variance):
        self.origins = normal(0, variance, (self.N, 2))
        self.plot_points()

        # Check for overlap
        reroll = input("Is there overlap? (y/n) ")
        if reroll.lower() == 'y' or reroll.lower() == 'yes':
            print("Need to reroll")
            self.generate_origins(1.1*variance)

    def spawn_robots(self):
        # Spawn each robot at the generated positions
        for i in range(self.N):
            # Set the initial position
            initial_pose = Pose()
            initial_pose.position.x = self.origins[i,0]
            initial_pose.position.y = self.origins[i,1]
            
            # Name each robot with the number
            robot_name = ("omni_bot_%d" % i)

            # Call the spawn_urdf_model service
            response = self.spawn_urdf(robot_name, self.model_xml, robot_name, initial_pose, "world")

            # Print any errors
            if not response.success:
                rospy.logerr(response.status_message)
                break
        
        if response.success:
            rospy.loginfo("Successfully Spawned %d Robots!" % self.N)


if __name__ == "__main__":
    # Argument parser to get number of robots to spawn
    parser = ap.ArgumentParser(description="Spawn multiple omni-bots at random positions")
    parser.add_argument("N", type=int, help="Number of omni-bots to spawn")
    args, unknown = parser.parse_known_args()

    spawner = OmniBotSpawner(args.N)
