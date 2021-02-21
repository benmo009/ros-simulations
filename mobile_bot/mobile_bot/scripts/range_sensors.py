#!/usr/bin/env python
# Subscribe to noisy gps topic

# compute robots relative pose to room cornders

# Do range sensor math (subproblems)

# Publish the estimated pose


import rospy
import numpy as np
from mobile_bot import RangeSensor

if __name__ == "__main__":
    try:
        # Get sensor location from parameter server
        r = RangeSensor.RangeSensor("range_sensor", [10,0], 0.01)
        while not rospy.is_shutdown():
            r.estimate_and_publish_state()
    except rospy.ROSInterruptException:
        pass