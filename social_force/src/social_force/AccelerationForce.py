import numpy as np
from numpy.linalg import norm

class AccelerationForce:
    def __init__(self, N, tau=0.5, mu=1.34, sigma=0.26):
        self.N = N  # Number of people
        self.tau = tau  # relaxation time for acceleration
        self.v_pref = np.random.normal(mu, sigma, (self.N, 1))

    def ComputeForces(self, directions, velocities):
        F = (1/self.tau) * (self.v_pref*directions - velocities)
        return F