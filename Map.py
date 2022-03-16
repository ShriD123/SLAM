#!/usr/bin/env python
import rospy
import numpy as np
from bresenham import bresenham
from nav_msgs.msg import OccupancyGrid, Odometry
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
        self.robot_loc = None
    
    def get_occupany_grid(self):
        og = OccupancyGrid()
        og.info.resolution = self.resolution
        og.info.width = self.width
        og.info.height = self.height

        # To fill grid from left to right and top to bottom
        og.info.origin.position.x = -self.m
        og.info.origin.position.y = -self.m 
        og.info.origin.orientation.y = -1
        og.info.origin.orientation.x = -1

        og.data = list(self.data.flatten())
        return og
    
    def update_map(self, coords, wall=True):
        if np.any(np.abs(coords) > self.m):
            raise IndexError('Coordinates out of bounds')
        coords = coords[:, :2]

        index = np.round((coords - self.grid_offset) / self.grid_spacing)
        index = index.astype(int)
        if self.robot_loc is not None:
            self.update_free_points(self.robot_loc, index)

        if wall:
            self.data[index[:, 0], index[:, 1]] = 100

    def update_map_from_listener(self, msg, wall=True):
        data = msg.data
        s = data.shape[0]
        coords = np.c_[data[:s//2], data[s//2:]]
        # remove all nan rows, which come from when
        # then walls is not in range of the sensor
        coords = coords[~np.isnan(coords).any(axis=1), :]
        self.update_map(coords, wall=wall)

    def update_free_points(self, origin, wall_pts):
        free_pts = []
        x0, y0 = origin
        for x1, y1 in wall_pts:
            line_pts = bresenham(x0, y0, x1, y1)
            free_pts.extend(list(line_pts))
        free_pts = np.array(free_pts, dtype=int)
        self.data[free_pts[:, 0], free_pts[:, 1]] = 25

    def update_robot_loc(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        loc = np.array([x, y])
        self.robot_loc = np.round((loc - self.grid_offset) / self.grid_spacing)
        self.robot_loc = self.robot_loc.astype(int)

map_pub = rospy.Publisher('/custom_map', OccupancyGrid, queue_size=1)
rospy.init_node('map_test', anonymous=True)
m = Map(10, 0.05)    
wall_listener = rospy.Subscriber('/wall_info', numpy_msg(Floats), m.update_map_from_listener)
free_listener = rospy.Subscriber('/free_info', numpy_msg(Floats), lambda x: m.update_map_from_listener(x, wall=False))
odom_listener = rospy.Subscriber('/odom', Odometry, m.update_robot_loc)
# rospy.spin()

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    # print(m.data.sum())
    map_pub.publish(m.get_occupany_grid())
    rate.sleep()

# map_pub = rospy.Publisher('/custom_map', OccupancyGrid, queue_size=1)
# rospy.init_node('map_test', anonymous=True)
# m = Map(10, 1)

# m.data[0, 0] = 100


# rate = rospy.Rate(10.0)
# while not rospy.is_shutdown():
#     map_pub.publish(m.get_occupany_grid())
#     rate.sleep()


