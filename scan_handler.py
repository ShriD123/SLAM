#!/usr/bin/env python
import numpy as np

import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import tf

class ScanHandler:
    """
    This class listens for laser scans, and transforms
    them into the odom frame.
    """
    def __init__(self):
        rospy.init_node('scan_handler')

        self.tl = tf.TransformListener()
        self.rate = rospy.Rate(10.0)

        self.scan_listener = rospy.Subscriber('/scan', LaserScan, self.handle_scan)
        self.wall_location_publisher = rospy.Publisher('/scan_info', numpy_msg(Floats), queue_size=10)

        rospy.spin()
        
    def handle_scan(self, msg):
        """
        Subscriber callback for laser scan messages.

        Reads in the scan, and translates the angle
        and distance data into x, y, z points in the
        odom frame.
        """
        angle_min = msg.angle_min
        angle_max = msg.angle_max

        r = np.array(msg.ranges)
        theta = np.linspace(angle_min, angle_max, len(r))
        
        x, y = self.get_coordinates_from_scan(r, theta)
        z = np.zeros_like(x)
        ones = np.ones_like(x)

        # Need the ones in order to multiply by the 4x4 rotation/translation
        # matrix
        scan_points = np.vstack([x, y, z, ones])
        scan_points[np.isinf(scan_points)] = np.nan

        # print(scan_points.T[:, :2])
        
        odom_points = self.scan_to_odom(scan_points)
        to_publish = odom_points.T

        self.wall_location_publisher.publish(np.r_[to_publish[:, 0], to_publish[:, 1]].astype('float32'))

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

ScanHandler()

# def process_scan(data):
#     angle_min = data.angle_min
#     angle_max = data.angle_max
#     angle_inc = data.angle_increment
#     time_inc = data.time_increment
#     scan_time = data.scan_time

#     ranges = data.ranges
#     intensities = data.intensities

#     print(ranges)
#     print(ranges[0])
#     print(ranges[90])
#     print(ranges[180])
#     print(ranges[270])
#     print("-----")

# def process_odom(data):
#     print(data.pose)
#     print("-----")
   
# frames = set()

# def process_tf(data):
#     print(data.transforms)
#     print(len(data.transforms))
#     print(data.transforms[0].header, data.transforms[0].child_frame_id)
#     print('-----')
#     # for transform in data.transforms:
#     #     frames.add(str(transform.child_frame_id))
#     #     frames.add(str(transform.header.frame_id))

#     print(frames)

# def listener():
#     rospy.init_node('listener')

#     tflistener = tf.TransformListener()
#     rate = rospy.Rate(10.0)
#     while not rospy.is_shutdown():
#         print(tflistener.frameExists('base_link'), tflistener.frameExists('base_scan'))
#         try:
#             (trans,rot) = tflistener.lookupTransform('base_link', 'base_scan', rospy.Time(0))
#             print(trans)
#             print(tflistener.fromTranslationRotation(trans, rot))
#         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#             continue

#         rate.sleep()


#     # l = tf.TransformListener()
#     # t = l.lookupTransform('/base_link', '/map', rospy.Time(0))
#     # print(type(t))
#     # print(t)

#     # listener = tf.TransformListener()
#     # rospy.Subscriber('/odom', Odometry, process_odom)
#     # rospy.Subscriber('/map', OccupancyGrid, process_map)
#     # rospy.Subscriber('/scan', LaserScan, process_scan)
#     # rospy.Subscriber('/tf_static', TFMessage, process_tf)

#     # rospy.spin()

