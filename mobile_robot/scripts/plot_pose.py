#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose
        
if __name__ == "__main__":

    # Initialize lists for storing data
    timestamps = []
    x_pos = []
    y_pos = []
    theta_pos = []

    # Initialize ros node
    rospy.init_node("model_state_client")

    # Set rate of calling service to 10 Hz
    rate = rospy.Rate(10)

    # Wait for service to start
    rospy.wait_for_service("/gazebo/get_model_state")

    start_time = rospy.get_time()
    t = start_time

    while not rospy.is_shutdown() and (t - start_time) < 60:
        try:
            # Call the model state service
            get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            model_state = get_model_state("mobile_bot", "world")

            # Get the position information
            x = model_state.pose.position.x
            y = model_state.pose.position.y
            q = model_state.pose.orientation

            # Get the current simulation time
            t = rospy.get_time()

            # Convert from quaternion to theta


            timestamps.append(t)
            x_pos.append(x)
            y_pos.append(y)
            rospy.loginfo("x: %.3f  y: %.3f" % (x, y))
            rospy.loginfo("q: %.2fi + %.2fj + %.2fk + %.2f\n" % (q.x, q.y, q.z, q.w))
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
    

    timestamps = np.array(timestamps)
    x_pos = np.array(x_pos)
    y_pos = np.array(y_pos)

    plt.plot(timestamps, x_pos, label="x")
    plt.plot(timestamps, y_pos, label="y")
    plt.legend()
    plt.xlabel("Time (s)")
    plt.ylabel("Position")

    plt.show()

