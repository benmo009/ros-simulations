#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
from matplotlib import transforms
from scipy import ndimage
import numpy as np
from matplotlib.patches import Circle
from Tree import Tree
import math


class RapidlyExploringRandomTree:
    def __init__(self):
        rospy.init_node("rrt_node")
        rospy.Subscriber("/costmap", OccupancyGrid, self.map_callback)
        rospy.wait_for_message("/costmap", OccupancyGrid)
        

    def map_callback(self, data):
        print(data.info)
        self.map_resolution = data.info.resolution

        self.map_origin = data.info.origin.position
        self.map_origin = np.array([self.map_origin.x, self.map_origin.y])

        self.map_origin = (-self.map_origin / self.map_resolution) - 1975
        self.map_origin = np.round(self.map_origin).astype(int)
        

        self.costmap = data.data
        self.costmap = np.array(self.costmap)
        self.costmap.shape = (data.info.width, data.info.height)
        self.costmap = self.costmap[1975:2185, 1975:2185]

        self.freespace = np.argwhere(self.costmap == 0)
        

        

    def set_goal(self, x, y):
        self.goal = np.array([x, y])
        self.goal_on_map = self.goal / self.map_resolution 
        self.goal_on_map += self.map_origin
        self.goal_on_map = np.round(self.goal_on_map).astype(int)


    def draw_tree(self, T_init, T_goal):
        costmap_with_tree = np.copy(self.costmap)    

        tree_init_points = np.array(T_init.V).astype(int)
        costmap_with_tree[tree_init_points[:,0], tree_init_points[:,1]] = 50
        
        tree_goal_points = np.array(T_goal.V).astype(int)
        costmap_with_tree[tree_goal_points[:,0], tree_goal_points[:,1]] = 80
        im = plt.imshow(costmap_with_tree)

        plt.scatter(self.map_origin[0], self.map_origin[1], c='orange')
        plt.scatter(self.goal_on_map[0], self.goal_on_map[1], c='orange')
        
        plt.show()
    
    def draw_path(self, path):
        costmap_with_tree = np.copy(self.costmap)    

        path = np.array(path)
        costmap_with_tree[path[:,0], path[:,1]] = 50
        
        im = plt.imshow(costmap_with_tree)

        plt.scatter(self.map_origin[0], self.map_origin[1], c='orange')
        plt.scatter(self.goal_on_map[0], self.goal_on_map[1], c='orange')
        
        plt.show()

    def build_rrt(self, n, q0):
        T = Tree()
        T.V = [tuple(q0)]
        
        for i in range(n):
            q_rand = np.random.randint(0, self.freespace.shape[0])
            q_rand = self.freespace[q_rand,:]
            self.extend_rrt(T, q_rand)

        return T
    
    def extend_rrt(self, T, q):
        q_near = self.closest_neighbor(T, q)
        q_new = self.find_q_new(q_near, q)

        if self.costmap[ tuple(q_new) ] == 0:
            T.V.append(q_new)
            T.E.append( (tuple(q_near), tuple(q_new)) )
            return q_new
        
        return np.array([None, None])


    def merge_rrt(self, T_init, T_goal, n):
        T1 = T_init
        T2 = T_goal
        for i in range(n):
            q_rand = np.random.randint(0, self.freespace.shape[0])
            q_rand = self.freespace[q_rand,:]

            q_new1 = self.extend_rrt(T1, q_rand)

            if (q_new1 != None).all():
                q_new2 = self.extend_rrt(T2, q_new1)

                if (q_new1 == q_new2).all():
                    return True, q_new1
                
                T_temp = T1
                T1 = T2
                T2 = T_temp

        return False, np.array([None, None])


    def tree_path_search(self, T, q_start, q_end):
        edge = T.E[-1]
        next_point = edge[0]
        points = [q_start, next_point]
        for i in range(-2, -1*len(T.E), -1):
            edge = T.E[i]
            if edge[1] == next_point:
                next_point = edge[0]
                points.append(next_point)

        return points


    def closest_neighbor(self, T, q):
        q_near = np.zeros_like(q)
        closest_dist = np.inf
        for point in T.V:
            point = np.array(point)
            distance = np.linalg.norm( point - q )
            if distance <= closest_dist:
                closest_dist = distance
                q_near = point

        return q_near

    def find_q_new(self, q_near, q_rand):
        q_new = np.zeros_like(q_rand)
        for i in range(2):
            if q_rand[i] > q_near[i]:
                q_new[i] = q_near[i] + 1
            elif q_rand[i] < q_near[i]:
                q_new[i] = q_near[i] - 1
            else:
                q_new[i] = q_near[i]
        
        return q_new
    

if __name__ == "__main__":    

    rrt = RapidlyExploringRandomTree()
    rrt.set_goal(6,6)
    T_init = rrt.build_rrt(300, rrt.map_origin)
    T_goal = rrt.build_rrt(300, rrt.goal_on_map)

    rrt.draw_tree(T_init, T_goal)

    success, q_common = rrt.merge_rrt(T_init, T_goal, 300)

    if success:
        print("Trees Merged at point", tuple(q_common))

    rrt.draw_tree(T_init, T_goal)

    start_points = rrt.tree_path_search(T_init, q_common, tuple(rrt.map_origin))
    goal_points = rrt.tree_path_search(T_goal, q_common, tuple(rrt.goal_on_map))
    start_points = np.flip(start_points, axis=0)
    path = np.concatenate((start_points, goal_points[1::]), axis=0)

    rrt.draw_path(path)

    # Publish path for rviz
    path_msg = Path()
    path_msg.header.frame_id = 'map'
    for point in path:
        pose_obj = PoseStamped()
        pose_obj.pose.position.y = (point[0] - 25) * rrt.map_resolution
        pose_obj.pose.position.x = (point[1] - 25)* rrt.map_resolution
        pose_obj.pose.orientation.w = 1
        path_msg.poses.append(pose_obj)

    pub = rospy.Publisher("robot_path", Path, queue_size=0)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(path_msg)
        rate.sleep()


    
