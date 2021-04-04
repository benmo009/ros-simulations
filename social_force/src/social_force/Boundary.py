import numpy as np 
import matplotlib.pyplot as plt 

class Boundary:
    def __init__(self, points):
        # Array of points describing the boundary
        self.points = points
        (self.n, _) = points.shape  # n x 2 array for n points


    # Find the point on boundary that is closest to person pose
    def ClosestPoint(self, r):
        # r - pedestrian pose

        # Find p - point on boundary closest to person
        min_dist = np.inf
        for i in range(self.n):
            dist = np.linalg.norm(r - self.points[i])
            if dist < min_dist:
                min_dist = dist
                p = self.points[i]

        return (r - p)
    
    
    def plot(self, fig, ax):
        ax.plot(self.points[:,0], self.points[:,1])





    