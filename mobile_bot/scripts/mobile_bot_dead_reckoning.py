#!/usr/bin/env python

import rospy
import numpy as np
from mobile_bot import DeadReckoning

if __name__ == "__main__":
    try:
        P = 0.1 * np.eye(3)
        dr = DeadReckoning.DeadReckoning("mobile_bot_dead_reckoning", P)
        while not rospy.is_shutdown():
            dr.estimate_and_publish_state()
            print("here")
    except rospy.ROSInterruptException:
        pass