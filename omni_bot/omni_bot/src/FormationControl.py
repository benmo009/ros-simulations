#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import Twist, Pose
import numpy as np
import networkx as nx
from networkx.linalg.graphmatrix import incidence_matrix
from gazebo_msgs.srv import GetModelState

class FormationControl:
    def __init__(self, form_rad, rate=10, Kp=0.6, Kp1=-0.25, qrange=1.0):
        self.form_rad = form_rad

        # Initialize ros node
        rospy.init_node("formation_control")
        rospy.wait_for_service("gazebo/get_model_state")
        self.model_state = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)

        self.rate = rospy.Rate(rate)
        self.dt = 1/rate

        # Get the initial positions of each robot from parameter server
        while not rospy.has_param('omni_bot/origins'):
            continue

        self.q = rospy.get_param('omni_bot/origins')
        self.q = np.array(self.q)
        (self.N, self.num_states) = self.q.shape
        self.flat_dim = self.N * self.num_states
        self.q.shape = (self.flat_dim, )

        # Initialize gains and parameters
        self.Kp = Kp * np.eye(self.flat_dim)
        self.Kp1 = Kp1 * np.eye(self.flat_dim)
        self.qrange = qrange

        rospy.loginfo("Formation Control - Got origins! There are %d robots" % self.N)

        self.linear_vel = Twist()
        self.angular_vel = 0

        # Set up publisher to each /omni_bot_#/cmd_vel topic
        self.omni_pubs = []
        for n in range(self.N):
            topic = ( "/omni_bot_%d/cmd_vel" % n )
            self.omni_pubs.append( rospy.Publisher(topic, Twist, queue_size=0))

        # Set up graphs
        self.InitGraphs()

        # Move to formation
        self.MoveToFormation(150)

        rospy.loginfo("Formation Control - In Formation!")

        # Start taking keyboard prompts - set up /cmd_vel subscriber
        self.vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)

        self.main()


    def vel_callback(self, data):
        self.linear_vel = data
        self.angular_vel = 0

        if data.angular.z > 0:
            self.angular_vel = 2
        elif data.angular.z < 0:
            self.angular_vel = -2


    def PublishToOmnibots(self, vel):
        for n in range(self.N):
            twist_msg = Twist()
            twist_msg.linear.x = vel[n,0]
            twist_msg.linear.y = vel[n,1]

            self.omni_pubs[n].publish(twist_msg)

    
    # Initializes the information graphs
    def InitGraphs(self):
        # Initialize links
        n1 = list(range(0, self.N))
        n2 = list(range(1, self.N))
        n2.append(0)

        # Initialize graphs
        self.G = nx.Graph()
        self.G_full = nx.Graph()

        # Construct the graph links
        n1_full = np.array([])
        for i in range(self.N):
            self.G.add_edge(n1[i], n2[i])

            full_edges = np.zeros((self.N-1-i, 2))
            full_edges[:,0] = i
            full_edges[:,1] = n1[i+1::]
        
            self.G_full.add_edges_from(full_edges)

        # Get the incidence matrices
        self.D = incidence_matrix(self.G, oriented=True)
        self.D = self.D.toarray()

        self.D_full = incidence_matrix(self.G_full, oriented=True)
        self.D_full = self.D_full.toarray()
        
        # Number of links
        (_, self.links) = self.D.shape
        (_, self.links_full) = self.D_full.shape

        # Kronecker Products
        self.K = np.kron( np.transpose(self.D), np.eye(self.num_states) )
        self.K_full = np.kron( np.transpose(self.D_full), np.eye(self.num_states) )


    def PositionPotential(self, z_des):
        z = self.K @ self.q 
        return z - z_des # Should I make this a member variable????


    def PhiFunc(self, x, dx):
        return (x < dx) * ( (1/x) + (1/dx) )

    def CollisionPotential(self):
        z_full = self.K_full @ self.q
        z_flat = np.reshape(z_full, (self.links_full, self.num_states))
        norm_z = np.linalg.norm(z_flat, axis=-1)
        norm_z = np.reshape( norm_z, (self.links_full, 1) )

        phi = z_flat * self.PhiFunc(norm_z, self.qrange) / norm_z
        phi = np.reshape( phi, (self.num_states*self.links_full, ) )

        return phi


    def GetRobotPositions(self):
        positions = np.zeros((self.N, self.num_states))

        for i in range(self.N):
            robot_name = ( "omni_bot_%d" % i )
            state = self.model_state(robot_name, 'world')
            p = state.pose.position
            positions[i] = [p.x, p.y]
        
        self.q = np.reshape(positions, (self.flat_dim, ))


    def MoveToFormation(self, T):
        # Compute desired positions
        theta_res = 2*np.pi / self.N
        theta = np.linspace(0, 2*np.pi - theta_res, self.N)
        q_des = np.zeros((self.N, self.num_states))
        q_des[:,0] = np.cos(theta)
        q_des[:,1] = np.sin(theta)
        q_des *= self.form_rad
        q_des.shape = (self.flat_dim, )

        # Compute desired z difference variable
        z_des  = np.kron( np.transpose(self.D), np.eye(self.num_states) ) @ q_des

        for t in range(T):
            self.GetRobotPositions()
            psi = self.PositionPotential(z_des)
            phi = self.CollisionPotential()

            u = -self.Kp @ np.kron( self.D, np.eye(self.num_states) ) @ psi 
            u += -self.Kp1 @ np.kron(self.D_full, np.eye(self.num_states)) @ phi

            self.PublishToOmnibots( np.reshape(u, (self.N, self.num_states)) )
            
            self.rate.sleep()

        z_final = self.K @ self.q 
        rospy.loginfo("Final Relative Error")
        rospy.loginfo(z_final)


    def main(self):
        while not rospy.is_shutdown():
            vel = np.zeros((self.N, self.num_states))
            vel[:,0] = self.linear_vel.linear.x
            vel[:,1] = self.linear_vel.linear.y
            vel.shape = (self.flat_dim, )

            th = self.angular_vel * self.dt
            R = np.array([ [np.cos(th), -np.sin(th)],
                            [np.sin(th), np.cos(th)] ])

            self.GetRobotPositions()
            
            q_des = np.reshape(self.q, (self.N, self.num_states)) @ np.transpose(R)
            q_des.shape = (self.flat_dim, )
            z_des = self.K @ q_des

            psi = self.PositionPotential(z_des)
            phi = self.CollisionPotential()

            u = -self.Kp @ np.kron( self.D, np.eye(self.num_states) ) @ psi 
            u += -self.Kp1 @ np.kron(self.D_full, np.eye(self.num_states)) @ phi
            u += vel

            self.PublishToOmnibots( np.reshape(u, (self.N, self.num_states)) )

            self.rate.sleep()


    def stop(self):
        for n in range(self.N):
            twist_msg = Twist()
            self.omni_pubs[n].publish(twist_msg)

if __name__ == "__main__":
    fc = FormationControl(2)
