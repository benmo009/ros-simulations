#!/usr/bin/env python

import rospy
from mobile_bot_ekf import ProximitySensor

if __name__ == "__main__":
    try:
        proximity_sensor = ProximitySensor.ProximitySensor("mobile_bot_proximity_sensor")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
