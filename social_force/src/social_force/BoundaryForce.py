import numpy as np
from Boundary import Boundary
from numpy.linalg import norm

class BoundaryForce:
    def __init__(self, boundaries, U0=10, R=0.2):
        # parameters
        self.U0 = U0
        self.R = R

        # Initialize list of boundaries in the scene
        self.boundaries = boundaries
        self.B = len(boundaries)

    def BoundaryPotential(self, vec):
        vec_norm = norm(vec, axis=-1)
        return self.U0 * np.exp(-vec_norm / self.R)

    
    def gradBoundaryPotential(self, vec):
        (N, _) = vec.shape

        U = self.BoundaryPotential(vec)

        dx = np.array([1e-3, 0])
        dy = np.array([0, 1e-3])

        U_dx = self.BoundaryPotential(vec+dx)
        U_dy = self.BoundaryPotential(vec+dy)

        dU = np.zeros((N,2))
        dU[:,0] = (U_dx - U) / dx[0]
        dU[:,1] = (U_dy - U) / dy[1]
        return dU

    def ComputeForces(self, states):
        (N, _) = states.shape

        F = np.zeros((N, 2))

        # For each person, compute the total force from all boundaries 
        for n in range(N):
            
            # Find the nearest point for each boundary
            nearest_points = np.zeros((self.B, 2))
            for b in range(self.B):
                nearest_points[b] = self.boundaries[b].ClosestPoint(states[n])

            # Compute the potentials
            dU = self.gradBoundaryPotential(nearest_points)
            F[n] = -1*np.sum(dU, axis=0)

        return F
