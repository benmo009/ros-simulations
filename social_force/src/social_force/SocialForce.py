import numpy as np 
import math 
import matplotlib.pyplot as plt
from numpy.linalg import norm
import matplotlib.animation as am

from AccelerationForce import AccelerationForce
from PedestrianForce import PedestrianForce
from BoundaryForce import BoundaryForce
from Boundary import Boundary

class SocialForce:
    def __init__(self, origins, goals, boundaries, dt, T):
        # Store number of pedestrians and number of boundaries
        (self.N, _) = origins.shape
        self.dt = dt
        self.T = T

        # Initialize states matrix
        self.states = np.zeros((self.T+1, self.N, 2))
        self.states[0] = origins

        self.goals = goals

        # Initialize force classes
        self.af = AccelerationForce(self.N)
        self.pf = PedestrianForce(self.dt)
        self.bf = BoundaryForce(boundaries)

        # Initialize velocity matrix
        self.velocities = np.zeros((self.N, 2))
        self.desired_vel = np.zeros((self.N, 2))
        self.max_vel = 1.3 * self.af.v_pref 


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
        

    def UpdateState(self, t, F):
        # Update desired velocity, w(t + dt)
        self.desired_vel = self.desired_vel + (self.dt * F)

        # Use the desired velocity to compute actual velocity, v(t + dt)
        self.UpdateVelocity()

        # Use the new velocity to copute next state
        self.states[t+1] = self.states[t] + (self.dt * self.velocities)


    def main(self):
        # For each time iteration
        for t in range(self.T):
            print("Iteration %d out of %d" % (t, self.T-1))
            # Initialize array for storing sum of forces
            F = np.zeros((self.N, 2))

            # Compute the forces acting on each person
            # Compute acceleration forces
            directions = self.ComputeDirections(self.states[t])
            F += self.af.ComputeForces(directions, self.velocities)

            # Compute pedestrian forces
            F += self.pf.ComputeForces(self.states[t], self.velocities, directions)

            # Compute the boundary forces
            F += self.bf.ComputeForces(self.states[t])

            # Update the states of each person
            self.UpdateState(t, F)

        return self.states
                
    def AnimateSimulation(self):
        print("Animating Simulation Results...", end="")
        # Plot the scene
        fig, ax = plt.subplots()

        boundaries[0].plot(fig, ax)
        boundaries[1].plot(fig, ax)

        self.scat = ax.scatter(self.states[0][:,0], self.states[0][:,1])
        ax.scatter(self.goals[:,0], self.goals[:,1], alpha=0)

        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        ax.set(xlim=xlim, ylim=ylim)

        animation = am.FuncAnimation(fig, self.animate, frames=self.T)

        animation.save("simulation.gif", fps=30)
        print("Done!")

    def animate(self, i):
        self.scat.set_offsets(self.states[i])
        return self.scat
        



if __name__ == "__main__":
    # 3 pedestrians
    # 2 boundaries

    # Initialize boundaries
    door_width = 2
    n = 1000

    boundary1 = np.zeros((n, 2))
    boundary1[:,1] = np.linspace(door_width/2, 10, 1000)

    boundary2 = np.zeros((n,2))
    boundary2[:,1] = np.linspace(-10, -door_width/2, 1000)

    boundaries = []
    boundaries.append(Boundary(boundary1))
    boundaries.append(Boundary(boundary2))

    # Initialize pedestrians
    N = 10
    origin = np.zeros((N, 2))
    origin[:, 0] = 3 + np.random.normal(0, 0.25, (N,))
    origin[0:N//2, :] *= -1
    origin[:,1] = np.random.normal(0, 0.5, (N,))

    goals = np.copy(origin)
    goals[:,0] *= -1
    goals[:,1] = np.random.normal(0, 1, (N,))

    dt = 0.1
    T = 200

    sf = SocialForce(origin, goals, boundaries, dt, T)
    states = sf.main()

    sf.AnimateSimulation()