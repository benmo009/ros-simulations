#!\usr\bin\env python

import numpy as np
import math
import rospy
from gazebo_msgs.srv import GetModelState
from numpy.random import normal
from tf.transformations import euler_from_quaternion

# A matrix for linearized mobile robot model
def A(q, u):
    Ak = np.eye(3)
    Ak[0,2] = u[0] * math.sin(q[2])
    Ak[1,2] = u[0] * math.cos(q[2])
    return Ak

# B matrix for linearized  mobile robot model
def B(q):
    Bk = np.zeros((3,2))
    Bk[0,0] = math.cos(q[2])
    Bk[1,0] = math.sin(q[2]) 
    Bk[2,1] = 1
    return Bk

# Open loop model for mobile robot
def f(q, u):
    F = np.zeros((3,2))
    F[0,0] = math.cos(q[2])
    F[1,0] = math.sin(q[2])
    F[2,1] = 1
    return q + F @ u

# Gets the initial state while adding noise
def get_initial_state(noise_cov=0.1*np.eye(3)):
    # Call the model state service
    # rospy.wait_for_service("/gazebo/get_model_state")
    # #rospy.sleep(5)  # Sleep for 5 seconds to let the model spawn in
    # get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    # model_state = get_model_state("mobile_bot", "world")
    # model_pose = model_state.pose

    # Create state vector
    state = np.zeros((3,1))

    noise_std_dev = np.sqrt(noise_cov)

    # Add x and y positions with noise
    # state[0] = model_pose.position.x + normal(0, noise_std_dev[0,0])
    state[0] = 1 + normal(0, noise_std_dev[0,0])
    state[1] = 1 + normal(0, noise_std_dev[1,1])
    
    # Get the orientation in euler angle
    # q = model_pose.orientation
    # euler_angles = euler_from_quaternion([q.x, q.y, q.z, q.w])
    # # Add the orientation with noise
    # state[2] = euler_angles[2] + normal(0, noise_std_dev[2,2])
    state[2] = 0 + normal(0, noise_std_dev[2,2])

    return state