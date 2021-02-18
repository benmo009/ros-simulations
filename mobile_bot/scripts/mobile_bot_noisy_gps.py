#!/usr/bin/env python

import rospy
from mobile_bot import NoisyGPS


if __name__ == "__main__":
    try:
        # Get GPS Parameters from parameter server
        if rospy.has_param("mobile_bot/gps/publish_rate"):
            # Publish rate
            pub_rate = rospy.get_param("mobile_bot/gps/publish_rate")
        else:
            pub_rate = 10
        
        if rospy.has_param("mobile_bot/gps/noise_covariance"):
            # GPS noise covariance
            noise_cov = rospy.get_param("mobile_bot/gps/noise_covariance")
        else:
            noise_cov = 0.1
        
        # Run the GPS node
        gps = NoisyGPS.NoisyGPS("mobile_bot_gps", pub_rate, noise_cov)
        while not rospy.is_shutdown():
            gps.get_pose_and_pub()
            gps.sleep()

    except rospy.ROSInterruptException:
        pass