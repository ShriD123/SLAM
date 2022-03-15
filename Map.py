#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

class Map:
    def __init__(self, m, s):
        self.m = m
        self.s = s
        self.grid_offset = np.array([-m, -m])
        self.grid_spacing = np.array([s,s])
        self.height = self.width = int(2*m // s)
        self.n_rows = self.n_cols = int(2*m // s)
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
        self.data[index[:, 0], index[:, 1]] = 100

    def update_map_from_listener(self, msg):
        data = msg.data
        s = data.shape[0]
        coords = np.c_[data[s//2:], data[:s//2]]
        # remove all nan rows, which come from when
        # then walls is not in range of the sensor
        coords = coords[~np.isnan(coords).any(axis=1), :]
        self.update_map(coords)

map_pub = rospy.Publisher('/custom_map', OccupancyGrid, queue_size=1)
rospy.init_node('map_test', anonymous=True)
m = Map(10, 0.1)    

wall_listener = rospy.Subscriber('/scan_info', numpy_msg(Floats), m.update_map_from_listener)
# rospy.spin()

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    print(m.data.sum())
    map_pub.publish(m.get_occupany_grid())
    rate.sleep()

# map_pub = rospy.Publisher('/custom_map', OccupancyGrid, queue_size=1)
# rospy.init_node('map_test', anonymous=True)
# m = Map(10, 1)

# m.data[0, 0] = 100
# pts = np.array([[0, 0], [-1, 1], [3, 3]])
# m.update_map(pts)

# rate = rospy.Rate(10.0)
# while not rospy.is_shutdown():
#     map_pub.publish(m.get_occupany_grid())
#     rate.sleep()
#     # rospy.spin()


