#!/usr/bin/env python

import rospy 
import numpy as np

if __name__ == "__main__":
    rospy.init_node('load_boundary_param')
    rospy.loginfo("Loading boundary parameters for doorway model")

    wall_width = 0.25
    wall_length = 10.0

    door_width = 2.0
    
    wall1 = np.zeros((1000, 2))
    wall1[0:450, 0] = np.linspace(door_width/2, (door_width/2)+wall_length, 450)
    
    wall1[450:900, 0] = np.linspace(door_width/2, (door_width/2)+wall_length, 450)
    wall1[450:900, 1] = -wall_width

    wall1[900:950, 1] = np.linspace(0, -wall_width, 50)
    wall1[900:950, 0] = door_width/2

    wall1[950:1000, 1] = np.linspace(0, -wall_width, 50)
    wall1[950:1000, 0] = (door_width/2) + wall_length

    wall2 = np.copy(wall1)
    wall2[:,0] *= -1

    boundaries = [wall1.tolist(), wall2.tolist()]

    rospy.set_param('social_force/boundaries', boundaries)
    rospy.loginfo("Done loading boundary parameters for doorway model")