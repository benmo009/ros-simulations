import numpy as np
import matplotlib.pyplot as plt
from numpy.random import normal
import networkx as nx 
import scipy
import math
import matplotlib.animation as am
import pickle

def plot_points(origins):
    plt.close('all')
    fig, ax = plt.subplots(1)

    (N, _) = origins.shape

    for i in range(N):
        ax.scatter(origins[i,0], origins[i,1], c='black')
        ax.add_patch(plt.Circle(origins[i,:], radius=0.2, 
                                fc='red', ec='black', alpha=0.5))

    # Set axis limits
    x_max = np.max( origins[:,0] ) + 0.2
    x_min = np.min( origins[:,0] ) - 0.2
    y_max = np.max( origins[:,1] ) + 0.2
    y_min = np.min( origins[:,1] ) - 0.2

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
def generate_origins(N, variance):
    origins = normal(0, variance, (N, 2))
    plot_points(origins)

    # Check for overlap
    reroll = input("Is there overlap? (y/n) ")
    if reroll.lower() == 'y' or reroll.lower() == 'yes':
        print("Need to reroll")
        origins = generate_origins(N, variance)

    return origins

# Potential function
def s(x, d):
    return np.log(x / d)

def psifun(x, dx):
    return (x < dx) * ( (1 / x) + (1 / dx) )
    

class Animator:
    def __init__(self, points, rad, xlim, ylim):
        self.points = points
        self.rad = rad

        (self.N, self.T) = points.shape
        self.N = self.N // 2

        self.q0 = np.reshape(points[:,0], (N, 2))
        self.qf = np.reshape(points[:,-1], (N, 2))

        self.xlim = xlim
        self.ylim = ylim


    def AnimatePoints(self):
        fig, ax = plt.subplots()
        ax.set_aspect('equal', adjustable='box')

        self.scat = ax.scatter(self.q0[:,0], self.q0[:,1], c='black')
        self.circles = []
        for i in range(self.N):
            circ = ax.add_patch( plt.Circle(self.q0[i], radius=self.rad, 
                                            fc='red', ec='black', alpha=0.5) )
            
            self.circles.append(circ)

            ax.add_patch( plt.Circle(self.qf[i], radius=self.rad, 
                                            fc='red', ec='black', alpha=0) )
    
    
        ax.set(xlim=self.xlim, ylim=self.ylim)

        animation = am.FuncAnimation(fig, self.animate, frames=self.T)

        animation.save("simulation.gif", writer='imagemagick', fps=30)
        print("Done!")

        plt.show()

    def animate(self, i):
        q = np.reshape(self.points[:,i], (self.N, 2))
        
        self.scat.set_offsets(q)
        for j in range(self.N):
            self.circles[j].set_center(q[j])

        return self.scat, self.circles
       



if __name__ == "__main__":
    # Number of robots
    N = 5

    # Generate origins
    origins = generate_origins(N, 1)

    # Initialize graph
    n1 = list(range(0, N))
    n2 = list(range(1, N))
    n2.append(0)

    # Construct the graph
    G = nx.Graph()
    G_full = nx.Graph()

    n1_full = np.array([])
    for i in range(N):
        G.add_edge(n1[i], n2[i])

        full_edges = np.zeros((N-1-i, 2))
        full_edges[:,0] = i
        full_edges[:,1] = n1[i+1::]
    
        G_full.add_edges_from(full_edges)

    D = nx.linalg.graphmatrix.incidence_matrix(G, oriented=True)
    D = D.toarray()

    D_full = nx.linalg.graphmatrix.incidence_matrix(G_full, oriented=True)
    D_full = D_full.toarray()
    
    # Number of links
    (_, links) = D.shape
    (_, links_full) = D_full.shape

    # Number of states for each agent
    n = 2
    # Number of time steps
    steps = 360
    
    q = np.zeros((n*N, steps+1))
    u = np.zeros((n*N, steps))

    # Initial Condition
    # get the initial robot poses
    q0 = np.copy(origins)
    q0.shape = (n*N,)

    q[:,0] = q0

    # Desired distance from center
    r_des = 2

    # Desired relative posiion and distance for formation
    theta_res = 2*math.pi / N
    theta = np.linspace(0, 2*math.pi - theta_res, N)

    q_des = np.zeros((N, 2))
    q_des[:,0] = np.cos(theta)
    q_des[:,1] = np.sin(theta)
    q_des *= r_des
    q_des.shape = (n*N,)

    z_des = np.kron( np.transpose(D), np.eye(n) ) @ q_des
    print(z_des)
    zdis_des = np.linalg.norm( np.reshape(z_des, (links, n)), axis=-1)
    
    # Time
    ts = 0.05
    t = ts * np.arange(0, steps)
    
    # Gains 
    Kp = 1 * np.eye(n*N)  # 0.3
    Kp1 = 0.2 * np.eye(n*N)  # 0.2  0.5  0.2

    for k in range(steps):

        zk = np.kron( np.transpose(D), np.eye(n,n) ) @ q[:,k]

        psi = zk - z_des
        qrange = 0.8 # to avoid collision  2.5  5  0.5

        # Collision Avoidance
        zkfull = np.kron( np.transpose(D_full), np.eye(n,n) ) @ q[:,k]
        zkflat = np.reshape(zkfull, (links_full, n))
        distzk = np.linalg.norm(zkflat, axis=-1)
        distzk.shape = (links_full, 1)

        phiflat = zkflat * psifun(distzk, qrange) / distzk
        phi = np.reshape(phiflat, n*links_full,)

        u[:,k] = -Kp@np.kron( D, np.eye(n) ) @ psi + Kp1 @ np.kron(D_full, np.eye(n)) @ phi
        
        q[:,k+1] = q[:,k] + ts * u[:,k]


    zf = np.kron( np.transpose(D), np.eye(n) ) @ q[:,-1]
    print("Final Relative Error")
    print(np.reshape(zf - z_des, (links, n)))

    fig, ax = plt.subplots()
    scat = ax.scatter(origins[:,0], origins[:,1], c='red', label="q0")

    qf = np.reshape(q[:,-1], (N,2))
    ax.scatter(qf[:,0], qf[:,1], c='green', label="qf")

    circles = []
    for i in range(N):
        circ = ax.add_patch(plt.Circle((origins[i,0],origins[i,1]), radius=0.2, 
                                    fc='red', ec='black', alpha=0.5))

        ax.add_patch(plt.Circle((qf[i,0],qf[i,1]), radius=0.2, 
                                    fc='green', ec='black', alpha=0.5))


    

    # ax.add_patch(plt.Circle((qf[:,0],qf[:,1]), radius=0.2, 
    #                                 fc='red', ec='black', alpha=0.5))



    for i in range(N):
        plt.plot(q[i*n, :], q[i*n+1, :], c='blue')
    
    plt.legend()

    ax.set_aspect('equal', adjustable='box')
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()

    plt.show()
    
    animator = Animator(q, 0.2, xlim, ylim)
    animator.AnimatePoints()

    # ===================================================================================
    # Rotate the circle

    ang_vel = 1  # 1 rad / s
    th = ang_vel * ts

    R = np.array([ [np.cos(th), -np.sin(th)],
                    [np.sin(th), np.cos(th)] ])

    q0 = np.reshape(q[:,-1], (N,n))
    q_old = np.copy(q)
    q[:,0] = q[:,-1]
        
    # Time
    ts = 0.05
    t = ts * np.arange(0, steps)

    for k in range(steps):
        current_q = np.reshape(q[:,k], (N,n))
        q_des = current_q @ np.transpose(R)
        q_des.shape = (n*N,)
        z_des = np.kron( np.transpose(D), np.eye(n) ) @ q_des

        zk = np.kron( np.transpose(D), np.eye(n,n) ) @ q[:,k]

        psi = zk - z_des
        qrange = 0.8 # to avoid collision  2.5  5  0.5

        # Collision Avoidance
        zkfull = np.kron( np.transpose(D_full), np.eye(n,n) ) @ q[:,k]
        zkflat = np.reshape(zkfull, (links_full, n))
        distzk = np.linalg.norm(zkflat, axis=-1)
        distzk.shape = (links_full, 1)

        phiflat = zkflat * psifun(distzk, qrange) / distzk
        phi = np.reshape(phiflat, n*links_full,)

        u[:,k] = -Kp @ np.kron( D, np.eye(n) ) @ psi + Kp1 @ np.kron(D_full, np.eye(n)) @ phi + 2*ts
        
        q[:,k+1] = q[:,k] + ts * u[:,k]


    zf = np.kron( np.transpose(D), np.eye(n) ) @ q[:,-1]
    print("Final Relative Error")
    print(np.reshape(zf - z_des, (links, n)))

    q = np.concatenate((q_old, q), axis=1)

    fig, ax = plt.subplots()
    origins = np.reshape(q[:,0], (N,2))
    scat = ax.scatter(origins[:,0], origins[:,1], c='red', label="q0")

    qf = np.reshape(q[:,-1], (N,2))
    ax.scatter(qf[:,0], qf[:,1], c='green', label="qf")

    circles = []
    qf = np.reshape(q[:,-1], (N,2))
    for i in range(N):
        circ = ax.add_patch(plt.Circle((origins[i,0],origins[i,1]), radius=0.2, 
                                    fc='red', ec='black', alpha=0.5))

        ax.add_patch(plt.Circle((qf[i,0],qf[i,1]), radius=0.2, 
                                    fc='green', ec='black', alpha=0.5))


    

    # ax.add_patch(plt.Circle((qf[:,0],qf[:,1]), radius=0.2, 
    #                                 fc='red', ec='black', alpha=0.5))



    for i in range(N):
        plt.plot(q[i*n, :], q[i*n+1, :], c='blue')
    
    plt.legend()

    ax.set_aspect('equal', adjustable='box')
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()

    plt.show()

    
    animator = Animator(q, 0.2, xlim, ylim)
    animator.AnimatePoints()

    
    
