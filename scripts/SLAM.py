#!/usr/bin/env python3
#
#   SLAM.py
#
#
import rospy
import math
import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from urdf_parser_py.urdf import Robot


#
#   Class
#
class TurtleBot:
    # Initialize.
    def __init__(self):
        # Create publishers and subscribers for moving the robot
        # self.pub_robot = rospy.Publisher('/mobile_base/command/velocity')
        self.pub_odom = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.sub_odom = rospy.Subscriber('/ground_truth/state', Odometry, self.callback_odom)
        # self.sub_odom = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.callback_sense)
    
    # Update every 10ms!
    def update(self, t, stop=False):
        twist_obj = Twist()
        twist_obj.linear.x = 0.5
        twist_obj.linear.y = 0
        twist_obj.linear.z = 0
        twist_obj.angular.x = 0
        twist_obj.angular.y = 0
        twist_obj.angular.z = 0
        if stop:
            twist_obj.linear.x = 0

        self.pub_odom.publish(twist_obj)

    # Main Code ??
    def main(self):
        # TODO Implement later I guess lmao
        pass

    # Callback Function 
    def callback_odom(self, msg):
        print(msg.pose)
        pass

    # Callback Function 
    def callback_sense(self, msg):
        # rospy.loginfo('I heard %s', msg)
        # print(msg.ranges[0])
        pass

#
#  Main Code
#
if __name__ == "__main__":
    ''' If we are going to make a servo loop, do this.'''
    # Prepare/initialize this node
    rospy.init_node('SLAM')

    # Instantiate the trajectory generator object, encapsulating all
    # the computation and local variables.
    slam = TurtleBot()

    # Prepare a servo loop at 100Hz.
    rate  = 100;
    servo = rospy.Rate(rate)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt of %f seconds (%fHz)" %
                  (dt, rate))


    # Run the servo loop until shutdown (killed or ctrl-C'ed).
    starttime = rospy.Time.now()
    # while not rospy.is_shutdown():
    for i in range(50):

        # Current time (since start)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()

        # Update the controller.
        slam.update(t, stop=True)

        # Wait for the next turn.  The timing is determined by the
        # above definition of servo.
        servo.sleep()
    
    for i in range(50):
        slam.update(0, stop=True)
        servo.sleep()
