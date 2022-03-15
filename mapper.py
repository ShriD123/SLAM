#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import tf
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from nav_msgs.msg import OccupancyGrid

from Map import Map 

class ScanHandler:
    """
    This class listens for laser scans, and transforms
    them into the odom frame.
    """
    def __init__(self):
        # rospy.init_node('scan_handler')

        self.tl = tf.TransformListener()
        self.rate = rospy.Rate(10.0)

        m = 10
        s = 0.1
        self.map = Map(m, s)

        self.scan_listener = rospy.Subscriber('/scan', LaserScan, self.handle_scan)
        self.wall_location_publisher = rospy.Publisher('scan_handler', numpy_msg(Floats), queue_size=10)
        self.map_publisher = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

        # rospy.spin()
        
    def handle_scan(self, msg):
        """
        Subscriber callback for laser scan messages.

        Reads in the scan, and translates the angle
        and distance data into x, y, z points in the
        odom frame.
        """
        angle_min = msg.angle_min
        angle_max = msg.angle_max

        theta = msg.ranges
        r = np.linspace(angle_min, angle_max, len(theta))
        
        x, y = self.get_coordinates_from_scan(r, theta)
        z = np.zeros_like(x)
        ones = np.ones_like(x)

        # Need the ones in order to multiply by the 4x4 rotation/translation
        # matrix
        scan_points = np.vstack([x, y, z, ones])
        
        odom_points = self.scan_to_odom(scan_points).T
        self.map.update_map(odom_points)
        self.map_publisher.publish(self.map.get_occupany_grid())
        # self.wall_location_publisher.publish(odom_points.T)

    def get_coordinates_from_scan(self, r: np.ndarray, theta: np.ndarray):
        """
        Takes in r, theta coordinates from the raw laser scan
        and transforms them into x, y coordinates.
        """
        # Note: angles are measured counter-clockwise from forward
        # facing orientation of the laser scanner
        x = -r*np.sin(theta)
        y = r*np.cos(theta)
        return x, y

    def scan_to_odom(self, scan_points: np.ndarray):
        """
        Given points in the laser scanner frame, transform
        them into the odom frame.

        scan_points: shape (3 x # laser points)
        """
        pts = self.transform_points('base_link', 'base_scan', scan_points)
        pts = self.transform_points('base_footprint', 'base_link', pts)
        pts = self.transform_points('odom', 'base_footprint', pts)
        return pts

    def transform_points(self, target_frame, source_frame, points):
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.tl.lookupTransform(target_frame, source_frame, rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            self.rate.sleep()

        transformation_matrix = self.tl.fromTranslationRotation(trans, rot)

        return transformation_matrix @ points

rospy.init_node('mapper')
sh = ScanHandler()
rospy.spin()

# class Mapper:
#     def __init__(self, m, s):
#         rospy.init_node('mapper', anonymous=True)
#         self.scan_listener = rospy.Subscriber('scan_handler', numpy_msg(Floats), self.process_scan_coords)
#         self.map_pub = rospy.Publisher('/custom_map', OccupancyGrid, queue_size=10)
#         self.map = Map(m, s)

#     def process_scan_coords(self, msg):
#         print(msg)
#         self.map.update_map(msg)
#         self.map_pub.publish(self.map.get_occupany_grid())

    