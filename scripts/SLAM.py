#!/usr/bin/env python3
#
#   SLAM.py
#
#
import rospy
import math
import numpy as np

from sensor_msgs.msg   import JointState
from urdf_parser_py.urdf import Robot


#
#  Generator Class
#
class Generator:
    # Initialize.
    def __init__(self):
        # Create a publisher to send the joint commands.  Add some time
        # for the subscriber to connect.  This isn't necessary, but means
        # we don't start sending messages until someone is listening.
        self.pub = rospy.Publisher("/joint_states", JointState)
        rospy.sleep(0.25)
       
        # Grab the robot's URDF from the parameter server.
        robot = Robot.from_parameter_server()

        # Instantiate the Kinematics
        self.kin = i_kin.Kinematics(robot, 'world', 'tip')            
        
        # Initialize the current segment index and starting time t0.
        # q_init = np.array([0.9, 0.0, 1.8]).reshape((3,1))
        q_init = np.array([0.0, 0.0, np.pi/2]).reshape((3,1))
        self.q_prev = q_init
        self.index = 0
        self.t0    = 0.0
        

    def calculatePos(self, t):
        z_pos = 0.4 * (1-math.cos(t))
        return np.array([0.0, 0.5, z_pos])

    def calculateVel(self, t):
        z_vel = 0.4 * math.sin(t)
        return np.array([0.0, 0.0, z_vel])
        
    def calculateXTilda(self, t, q, dt):
        x_val = self.calculatePos(t-dt).reshape((3,1))
        (T, J) = self.kin.fkin(q)
        xq_val = i_kin.p_from_T(T)
        return x_val - xq_val
    
    def calculateQDot(self, t, q, dt):  
        # lambda_kin = 20
        lambda_kin = 1
        x_vec = self.calculateVel(t).reshape((3,1)) + lambda_kin*self.calculateXTilda(t,q,dt)
        (T,J) = self.kin.fkin(q)
        qdot = np.linalg.inv(J[0:3,0:3]) @ x_vec
        return qdot
    
    # Update every 10ms!
    def update(self, t):
        # Create an empty joint state message.
        cmdmsg = JointState()
        cmdmsg.name = ['theta1', 'theta2', 'theta3']
        
        # Check the end time and just run one cycle
        if (t >= 2*np.pi):
            rospy.signal_shutdown("The motion of one cycle is done.")
            return

        # If the current segment is done, shift to the next.
        #if (t - self.t0 >= self.segments[self.index].duration()):
            #self.index = (self.index+1)
            # If the list were cyclic, you could go back to the start with
            # self.index = (self.index+1) % len(self.splines)
            
        # Compute the next iteration of q
        dt = t-self.t0
        qdot = self.calculateQDot(t, self.q_prev, dt)
        q_now = self.q_prev + dt*qdot
        (cmdmsg.position, cmdmsg.velocity) = (q_now, qdot)
        self.t0 = t
        self.q_prev = q_now
        
        # Send the command (with the current time).
        cmdmsg.header.stamp = rospy.Time.now()
        self.pub.publish(cmdmsg)
#
#  Main Code
#
if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('generator')

    # Instantiate the trajectory generator object, encapsulating all
    # the computation and local variables.
    generator = Generator()

    # Prepare a servo loop at 100Hz.
    rate  = 100;
    servo = rospy.Rate(rate)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt of %f seconds (%fHz)" %
                  (dt, rate))


    # Run the servo loop until shutdown (killed or ctrl-C'ed).
    starttime = rospy.Time.now()
    while not rospy.is_shutdown():

        # Current time (since start)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()

        # Update the controller.
        generator.update(t)

        # Wait for the next turn.  The timing is determined by the
        # above definition of servo.
        servo.sleep()
