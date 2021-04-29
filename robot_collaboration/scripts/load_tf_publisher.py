#!/usr/bin/env python3

import rospy 
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path

class PathSub:
    def __init__(self):
        self.sub = rospy.Subscriber('robot_path', Path, self.callback)
        rospy.wait_for_message('robot_path', Path)

    def callback(self, data):
        self.path = data.poses 

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


if __name__ == "__main__":

    rospy.init_node('load_tf_br')
    tf_pub = rospy.Publisher('/tf', TFMessage)
    ps = PathSub()
    

    path = ps.path 
    P = np.zeros((len(path), 3))
    for i in range(len(path)):
        P[i,0] = path[i].pose.position.x 
        P[i,1] = path[i].pose.position.y

    diff = P[1::, 0:2] - P[0:-1, 0:2]

    P[1::, 2] = np.arctan2(diff[:,1], diff[:,0])
    euler = np.zeros((len(path), 3))
    euler[:,-1] = P[:, 2]

    q = euler_to_quaternion(euler)


    

    rate = rospy.Rate(10)
    for i in range(len(path)):
        tf_msg = TFMessage()
        tf_stamp = TransformStamped()
        tf_stamp.header.stamp = rospy.Time.now()
        tf_stamp.header.frame_id = 'map'
        tf_stamp.child_frame_id = 'load_base_link'
        tf_stamp.transform.rotation.w = 1
        tf_stamp.transform.translation = path[i].pose.position
        tf_stamp.transform.rotation.x = q[i,0]
        tf_stamp.transform.rotation.y = q[i,1]
        tf_stamp.transform.rotation.z = q[i,2]
        tf_stamp.transform.rotation.w = q[i,3]





        tf_msg.transforms = [tf_stamp]

        tf_pub.publish(tf_msg)


        rate.sleep()
