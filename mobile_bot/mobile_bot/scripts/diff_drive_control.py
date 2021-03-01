#!/usr/bin/env python

import rospy
from mobile_bot import DiffDriveControl

if __name__ == "__main__":
    name = "mobile_bot_diff_drive_controller"
    noise_cov = 0

    diff = DiffDriveControl.DiffDriveControl(name, noise_cov)
    rospy.spin()