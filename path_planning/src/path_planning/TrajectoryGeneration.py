#!/usr/bin/env python3 

import numpy as np
import rospy
from nav_msgs.msg import Path
import pickle

# Function to solve the quadratic equation
def quadreg(a, b, c):
    d = b**2 - 4*a*c
    x1 = (-b + np.sqrt(d)) / (2*a)
    x2 = (-b - np.sqrt(d)) / (2*a)

    return x1, x2

# Checks if a value x is within a limit x = [low, high]
def in_range(x, lim, inclusive=True):
    if inclusive:
        return x <= lim[1] and x >= lim[0]
    else:
        return x < lim[1] and x > lim[0]

# Converts euler angles to quaternions
def euler_to_quaternion(euler):
    N = euler.shape[0]
    c = np.cos(euler/2)
    s = np.sin(euler/2)

    qx = (s[:,0] * c[:,1] * c[:,2]) - (c[:,0] * s[:,1] * s[:,1])
    qy = (c[:,0] * s[:,1] * c[:,2]) + (s[:,0] * c[:,1] * s[:,2])
    qz = (c[:,0] * c[:,1] * s[:,2]) - (s[:,0] * s[:,1] * c[:,2])
    qw = (c[:,0] * c[:,1] * c[:,2]) + (s[:,0] * s[:,1] * s[:,2])

    q = np.zeros((N, 4))
    q[:,0] = qx
    q[:,1] = qy
    q[:,2] = qz
    q[:,3] = qw

    return q

# Generate a trajectory from an array of (x,y) positions
def generate_trajectory(path, v0=0, vf=0, vmax=1.0, amax=5.0, dmax=-5.0):
    # Compute path lengths
    N = path.shape[0]
    diff = path[1::] - path[0:-1]
    theta = np.zeros(N)
    theta[1::] = np.arctan2(diff[:,1], diff[:,0])

    path_len = np.zeros(N)
    path_len[1::] = np.linalg.norm(diff, axis=-1)
    for i in range(1, N):
        path_len[i] = path_len[i] + path_len[i-1]

    # Get starting and final distances
    x0 = path_len[0]
    xf = path_len[-1]

    # Generate trajectory
    # ======================
    # Acceleration and deceleration times
    ta = (vmax - v0) / amax  # Time to finish accelerating
    td = (vf - vmax) / dmax
    # Total trajectory time
    tf = ( xf - x0 + (amax/2)*ta**2 - (dmax/2)*td**2 ) / (amax*ta + v0)
    td = tf - td  # Time to start decelerating

    # Compute integration constants
    Cd = xf - (dmax/2)*tf**2 - (vmax - dmax*td)*tf
    Cv = (-dmax/2)*td**2 + Cd
    Ca = x0

    const_lim = [vmax*ta + Cv, vmax*td + Cv]
    acc_lim = [x0, const_lim[0]]
    dec_lim = [const_lim[1], xf]

    # Index path length with time
    times = np.zeros(N)
    times[0] = 0
    times[-1] = tf
    for i in range(1, N-1):
        d = path_len[i]
        # Point falls in constant velocity time block
        if in_range(d, const_lim):
            times[i] = (d - Cv) / vmax
        # Point falls in accelerating time block
        elif in_range(d, acc_lim):
            t1, t2 = quadreg(amax/2, 0, Ca - d)
            if abs(t1 - t2) <= 1e-6:
                times[i] = t1
            elif in_range(t1, acc_lim) and not in_range(t2, acc_lim):
                times[i] = t1
            elif in_range(t2, acc_lim) and not in_range(t1, acc_lim):
                times[i] = t2
            else:
                print("Both times are in the trajectory?")
                print("({}, {})".format(t1, t2))
                times[i] = t1
        elif in_range(d, dec_lim):
            t1, t2 = quadreg(dmax/2, (vmax-dmax*td), Cd - d)
            if abs(t1 - t2) <= 1e-6:
                times[i] = t1
            elif in_range(t1, dec_lim) and not in_range(t2, dec_lim):
                times[i] = t1
            elif in_range(t2, dec_lim) and not in_range(t1, dec_lim):
                times[i] = t2
            else:
                print("Both times are in the trajectory?")
                print("({}, {})".format(t1, t2))
                times[i] = t1

    # Compile the times, (x,y) position and orientation into one array
    trajectory = np.zeros((N, 4))
    trajectory[:,0] = times
    trajectory[:,1:3] = path
    trajectory[:,3] = theta

    return trajectory


class PathSub:
    def __init__(self):
        self.sub = rospy.Subscriber('robot_path', Path, self.callback)
        rospy.wait_for_message('robot_path', Path)

    def callback(self, data):
        self.path = data.poses 


if __name__ == "__main__":
    rospy.init_node('trajectory_generator')
    #tf_pub = rospy.Publisher('/tf', TFMessage)
    ps = PathSub()
    

    path = ps.path 
    P = np.zeros((len(path), 2))
    for i in range(len(path)):
        P[i,0] = path[i].pose.position.x 
        P[i,1] = path[i].pose.position.y

    fid = open("/home/ben/path_points.txt", "wb")
    pickle.dump(P, fid)
    fid.close()

    