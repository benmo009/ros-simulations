#!/usr/bin/env python

# Subscribe to /mobile_bot/diff_drive_controller/odom
# Get initial state from ros service
# Use odometry twist data to estimate input command
# Compute predicted next state using the unicycle model

# Publish estimated state to /mobile_bot/estimated_pose

import rospy
from mobile_bot import OdometrySubscriber


if __name__ == "__main__":
    try:
        odom_sub = OdometrySubscriber.OdometrySubscriber("mobile_bot_odom_subscriber")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass