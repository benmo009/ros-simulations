# Kalman Filter implementation for mobile robot

import math
import numpy as np
from numpy.linalg import inv, det

class ExtendedKalmanFilter:
    # Extended Kalman Filter Constructor
    def __init__(self, A, B, C, f, h, V, W, P0, q0):
        # Functions needed for Kalman Filter
        self.A = A  # Function that takes inputs (q_hat, u), returns Ak matrix
        self.B = B  # Function that takes input q_hat and returns Bk matrix
        self.C = C  # Function that takes input q_hat and returns Ck matrix
        self.f = f  # Function that takes inputs (q_hat, u), returns q_bar
        self.h = h  # Function that takes input q_bar, returns predicted output

        # Variables needed for Kalman Filter
        self.V = V  # Covariance for output noise, (1 x 1)
        self.W = W  # Covariance matrix for input command u, (2 x 2)
        self.P = P0  # Covariance matrix for initial state q, (3 x 3)
        self.q_hat = q0  # Initial state guess, (3 x 1)
  
    # Takes the current input and output and returns the new estimated state
    def estimate(self, y, u):
        # Get q_bar, the open loop estimate
        self.q_bar = self.f(self.q_hat, u)

        # Compute linearized matrices for the current step
        A_mat = self.A(self.q_hat, u)
        B_mat = self.B(self.q_hat)
        C_mat = self.C(self.q_hat)

        A_tran = np.transpose(A_mat)
        B_tran = np.transpose(B_mat)
        C_tran = np.transpose(C_mat)

        if det(self.V) != 0:
            V_inv = inv(self.V)
        else:
            V_inv = np.zeros((1,1))

        # Compute K
        self.Sigma = A_mat @ self.P @ A_tran + B_mat @ self.W @ B_tran
        self.P = inv( inv(self.Sigma) + C_tran @ V_inv @ C_mat )
        self.K = self.P @ C_tran @ V_inv

        # Compute next q_hat
        self.q_hat = self.q_bar + self.K * (y - self.h(self.q_bar))
        
        return self.q_hat

        