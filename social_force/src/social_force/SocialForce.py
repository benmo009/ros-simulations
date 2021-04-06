#!/usr/bin/env python

import rospy

import numpy as np 
import math
from numpy.linalg import norm
from numpy.random import multivariate_normal
import matplotlib.pyplot as plt

from social_force.AccelerationForce import AccelerationForce
from social_force.PedestrianForce import PedestrianForce
from social_force.BoundaryForce import BoundaryForce
from social_force.Boundary import Boundary

from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist

class SocialForce:
    def __init__(self, T, rate=10):
        # Initialize ros node
        rospy.init_node("social_force")
        # Initialize service client for get_model_state
        rospy.wait_for_service("gazebo/get_model_state")
        self.model_state = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)

        # Store time parameters
        self.T = T
        self.rate = rospy.Rate(rate)
        self.dt = 1 / rate

        # Get the initial positions of each robot from parameter server
        rospy.loginfo("Waiting for robot origins...")
        while not rospy.has_param('omni_bot/origins'):
            continue

        self.state = rospy.get_param('omni_bot/origins')
        self.state = np.array(self.state)
        (self.N, self.num_states) = self.state.shape

        rospy.loginfo("Recieved robot origins! There are %d robots" % self.N)

        # Invert the origins for goals
        mu = np.mean(self.state, axis=0)
        mu[1] *= -1
        mu[0] += 5
        var = np.var(self.state, axis=0) * np.eye(2)
        self.GenerateGoals(mu, var)
        
        # Get boundary information
        rospy.loginfo("Waiting for boundaray parameters...")
        while not rospy.has_param('social_force/boundaries'):
            continue

        # Get boundaries from parameter server
        boundaries = rospy.get_param('social_force/boundaries')
        for i in range(len(boundaries)):
                boundaries[i] = Boundary(np.array(boundaries[i]))

        rospy.loginfo("Recieved boundary parameters")

        # Initialize force classes
        self.af = AccelerationForce(self.N, tau=0.2)
        self.pf = PedestrianForce(self.dt, sigma=0.6, c=1.0)
        self.bf = BoundaryForce(boundaries)

        # Initialize velocity matrix
        self.velocities = np.zeros((self.N, 2))
        self.desired_vel = np.zeros((self.N, 2))
        self.max_vel = 1.3 * self.af.v_pref 

        # Set up publisher to each /omni_bot_#/cmd_vel topic
        self.omni_pubs = []
        for n in range(self.N):
            topic = ( "/omni_bot_%d/cmd_vel" % n )
            self.omni_pubs.append( rospy.Publisher(topic, Twist, queue_size=0))


    def PublishToOmnibots(self, vel):
        for n in range(self.N):
            twist_msg = Twist()
            twist_msg.linear.x = vel[n,0]
            twist_msg.linear.y = vel[n,1]

            self.omni_pubs[n].publish(twist_msg)

    def GetRobotPositions(self):
        for i in range(self.N):
            robot_name = ( "omni_bot_%d" % i )
            model_states = self.model_state(robot_name, 'world')
            p = model_states.pose.position
            self.state[i] = [p.x, p.y]


    def plot_points(self, points):
        plt.close('all')
        fig, ax = plt.subplots(1)

        for i in range(self.N):
            ax.scatter(points[i,0], points[i,1], c='black')
            ax.add_patch(plt.Circle(points[i,:], radius=0.2, 
                                    fc='red', ec='black', alpha=0.5))

        # Set axis limits
        x_max = np.max( points[:,0] ) + 0.2
        x_min = np.min( points[:,0] ) - 0.2
        y_max = np.max( points[:,1] ) + 0.2
        y_min = np.min( points[:,1] ) - 0.2

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
    def GenerateGoals(self, mean, variance):
        self.goals = multivariate_normal(mean, variance, self.N)
        self.plot_points(self.goals)

        # Check for overlap
        reroll = input("Is there overlap? (y/n) ")
        if reroll.lower() == 'y' or reroll.lower() == 'yes':
            print("Need to reroll")
            self.GenerateGoals(mean, 1.1*variance)


    def ComputeDirections(self, states):
        # Compute vector from goal to current state
        e = self.goals - states

        # Normalize the vectors to unit vector
        e_norm = norm(e, axis=-1)
        e_norm.shape = (self.N, 1)
        e /= e_norm

        return e      

    def UpdateVelocity(self):
        # Compute norm of desired velocity
        norm_des = norm(self.desired_vel, axis=-1)
        norm_des.shape = (self.N, 1)

        # Check with the maximum acceptable speed
        g = np.where(self.max_vel >= norm_des, 1, self.max_vel / norm_des)

        # Update the velocity
        self.velocities = self.desired_vel * g
        self.PublishToOmnibots(self.velocities)
        

    def UpdateState(self, t, F):
        # Update desired velocity, w(t + dt)
        self.desired_vel = self.desired_vel + (self.dt * F)

        # Use the desired velocity to compute actual velocity, v(t + dt)
        self.UpdateVelocity()

        # Use the new velocity to copute next state
        #self.state = self.state + (self.dt * self.velocities)
        self.GetRobotPositions()


    def main(self):
        # For each time iteration
        for t in range(self.T):
            rospy.loginfo("Iteration %d out of %d" % (t, self.T-1))
            # Initialize array for storing sum of forces
            F = np.zeros((self.N, 2))

            # Compute the forces acting on each person
            # Compute acceleration forces
            directions = self.ComputeDirections(self.state)
            F += self.af.ComputeForces(directions, self.velocities)

            # Compute pedestrian forces
            F += self.pf.ComputeForces(self.state, self.velocities, directions)

            # Compute the boundary forces
            F += self.bf.ComputeForces(self.state)

            # Update the states of each person
            self.UpdateState(t, F)

            self.rate.sleep()

        return self.state

    def stop(self):
        vel = np.zeros((self.N, 2))
        self.PublishToOmnibots(vel)



if __name__ == "__main__":
    T = 100
    sf = SocialForce(T)
    states = sf.main()
    sf.stop()
