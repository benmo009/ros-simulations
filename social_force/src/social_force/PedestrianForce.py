import numpy as np
from numpy.linalg import norm 

class PedestrianForce:
    def __init__(self, dt, V0=2.1, sigma=0.3, c=0.5, psi=100):
        self.V0 = V0
        self.sigma = sigma
        self.c = c
        self.psi = psi * (np.pi/180)  # Convert to radians
        self.dt = dt
    
    # Gets vector from specified person to everyone else
    def VecToOthers(self, states, person):
        others = np.concatenate((states[0:person], states[person+1::]))
        return (states[person] - others)

    # Gets velocities of every other person besides the specified person
    def OthersVelocity(self, vel, person):
        return np.concatenate( (vel[0:person], vel[person+1::]) )

    def OthersDirection(self, directions, person):
        return np.concatenate( (directions[0:person], directions[person+1::]) )

    # Computes the semiminor axis for the potential (the exponent term)
    def SemiminorAxis(self, vec, vel, direct):
        # vec - vector from current person to everyone else ( M-1 x 2 )
        # vel - velocities of every other person ( M-1 x 2 )
        (M,_) = vel.shape

        # Compute the speeds from velocities, row-wise norm
        speeds = norm(vel, axis=-1)
        speeds.shape = (M, 1)
        norm_vec = norm(vec, axis=-1)
        other_norm = norm(vec - self.dt*speeds*direct, axis=-1)

        # Compute the left and right side of the square root
        left = (norm_vec + other_norm)**2
        right = (speeds*self.dt)**2
        right.shape = (M, )

        b = 0.5 * np.sqrt( left - right )
        return b

    # Compute potential from one person to everyone else
    def PedestrianPotential(self, vec, vel, direct):
        axis = self.SemiminorAxis(vec, vel, direct)
        V = self.V0 * np.exp(-axis / self.sigma)
        return V

    # Computes the gradient of the potential
    def gradPedestrianPotential(self, vec, vel, direct):
        (N,_) = vec.shape

        V = self.PedestrianPotential(vec, vel, direct)

        # Increment rab by dx and dy
        dx = np.array([1e-3, 0])
        dy = np.array([0, 1e-3])

        V_dx = self.PedestrianPotential(vec+dx, vel, direct)
        V_dy = self.PedestrianPotential(vec+dy, vel, direct)

        dV = np.zeros((N,2))
        dV[:,0] = (V_dx - V)/dx[0]
        dV[:,1] = (V_dy - V)/dy[1]
        return dV

    def CheckSight(self, direction, forces):
        (M, _) = forces.shape
        dot_prod = forces @ direction
        ang = norm(forces, axis=-1) * np.cos(self.psi)

        w = np.where(dot_prod >= ang, 1, self.c)
        w.shape = (M,1)
        forces = forces * w
        return forces

    # Computes the total forces on each person from every other person
    def ComputeForces(self, states, velocities, directions):
        # Get number of people
        (N,_) = states.shape

        # Initialize force array
        F = np.zeros((N,2))

        # For each person, compute the sum of pedestrian forces on person n
        for n in range(N):
            # Compute the force from every other person and add it up
            e = directions[n]
            vec = self.VecToOthers(states, n)
            vel = self.OthersVelocity(velocities, n)
            direct = self.OthersDirection(directions, n)
            
            dV = self.gradPedestrianPotential(vec, vel, direct)

            # Check line of sight
            dV = self.CheckSight(e, dV)

            # Add up the forces
            F[n] = -1*np.sum(dV, axis=0)

        return F


        

        