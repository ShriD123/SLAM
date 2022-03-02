#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

class Map:
    def __init__(self, m, s):
        self.m = m
        self.s = s
        self.grid_offset = np.array([-m, -m])
        self.grid_spacing = np.array([s,s])
        self.height = self.width = 2*m
        self.n_rows = self.n_cols = self.height // s
        self.resolution = s
        self.data = np.zeros((self.n_rows, self.n_cols), dtype=int)
    
    def get_occupany_grid(self):
        og = OccupancyGrid()
        og.info.resolution = self.resolution
        og.info.width = self.width
        og.info.height = self.height

        # To fill grid from left to right and top to bottom
        og.info.origin.orientation.y = -1
        og.info.origin.orientation.x = 1

        og.data = list(self.data.flatten())
        return og
    
    def update_map(self, coords):
        if np.any(np.abs(coords) > self.m):
            raise IndexError('Coordinates out of bounds')
        coords = coords[:, :2]

        index = np.round((coords - self.grid_offset) / self.grid_spacing)
        index = index.astype(int)
        print(index)
        self.data[index[:, 0], index[:, 1]] = 100
            
if __name__ == 'main':
    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
    rospy.init_node('map_test', anonymous=True)
    m = Map(10, 1)

    m.data[0, 0] = 100
    pts = np.array([[0, 0], [-1, 1], [3, 3]])
    m.update_map(pts)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        map_pub.publish(m.get_occupany_grid())
        rate.sleep()
        # rospy.spin()


