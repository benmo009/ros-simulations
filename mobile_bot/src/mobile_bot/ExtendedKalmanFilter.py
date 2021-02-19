# Initialization
#   Get initial q from gazebo service
#   Add some noise to get initial q_hat and initial P




import rospy
import numpy as np
import math

class ExtendedKalmanFilter:
    def __init__(self, A, B, C, f, h, V, W, P0, q0):
        # Functions needed for Kalman Filter
        self.A = A  # Function that takes inputs (q_hat, u), returns Ak matrix
        self.B = B  # Function that takes input q_hat and returns Bk matrix
        self.C = C  # Function that takes input q_hat and returns Ck matrix
        self.f = f  # Function that takes inputs (q_hat, u), returns open state q_bar
        self.h = h  # Function that takes input q_bar, returns predicted output

        # Variables needed for Kalman Filter
        self.V = V  # Covariance for output noise, (1 x 1)
        self.W = W  # Covariance matrix for input command u, (2 x 2)
        self.P = P0  # Covariance matrix for initial state q, (3 x 3)
        self.q_hat = q0  # Initial state guess, (3 x 1)

        
    # Takes the current input and output of the state and returns the new
    # estimated state
    def estimate(self, y, u):
        # Get q_bar, the open loop estimate
        self.q_bar = self.f(self.q_hat, u)

        # Compute linearized matrices for the current step
        A_matrix = self.A(self.q_hat, u)
        B_matrix = self.B(self.q_hat)
        C_matrix = self.C(self.q_hat)

        A_transpose = np.transpose(A_matrix)
        B_transpose = np.transpose(B_matrix)
        C_transpose = np.transpose(C_matrix)

        if np.linalg.det(self.V) != 0:
            V_inv = np.linalg.inv(self.V)
        else:
            V_inv = np.zeros((1,1))

        # Compute K
        self.Sigma = A_matrix @ self.P @ A_transpose + B_matrix @ self.W @ B_transpose
        self.P = np.linalg.inv( np.linalg.inv(self.Sigma) + C_transpose @ V_inv @ C_matrix )
        self.K = self.P @ C_transpose @ V_inv

        # Compute next q_hat
        self.q_hat = self.q_bar + self.K * (y - self.h(self.q_bar))
        
        return self.q_hat

        