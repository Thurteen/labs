#!/usr/bin/env python2

"""ROS Node for controlling the ARDrone 2.0 using the ardrone_autonomy package.

This ROS node subscribes to the following topics:
/vicon/ARDroneCarre/ARDroneCarre

This ROS node publishes to the following topics:
/cmd_vel

"""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist
from position_controller import PositionController#position_controller.py


# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from aer1217_ardrone_simulator.msg import GazeboState
from aer1217_ardrone_simulator.msg import DesiredStateMsg


class ROSControllerNode(object):
    """ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""
    # write code here to define node publishers and subscribers
    # publish to /cmd_vel topic
    # subscribe to /vicon/ARDroneCarre/ARDroneCarre for position and attitude feedback

    def __init__(self):
        self.position_controller = PositionController()

        self.sub_desired_state = rospy.Subscriber('/aer1217_ardrone/desired_state',
                                              DesiredStateMsg,
                                              self.update_quadrotor_desired_state)
        #######################################################################
        # input is the estimated state x,y,z,roll,pitch,yaw from VICON
        self.sub_estimated_state_vicon = rospy.Subscriber('/vicon/ARDroneCarre/ARDroneCarre',
                                                           TransformStamped,
                                                           self.update_quadrotor_estimated_state_from_vicon)
        #######################################################################
        # output roll,pitch,yaw_rate,climb_rate commands
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=30)
        self.pub_cmd_vel_RHC = rospy.Publisher('cmd_vel_RHC', Twist, queue_size=30)
        #######################################################################
        self.onboard_loop_frequency = 200.
        self.pub_cmd_vel_timer = rospy.Timer(rospy.Duration(1. / self.onboard_loop_frequency), self.send_cmd_vel)



    def update_quadrotor_desired_state(self, desired_state_msg):
        self.position_controller.set_desired_state(desired_state_msg.x[0],
                                                   desired_state_msg.x[1],
                                                   desired_state_msg.x[2],
                                                   desired_state_msg.x_dot[0],
                                                   desired_state_msg.x_dot[1],
                                                   desired_state_msg.x_dot[2],
                                                   desired_state_msg.x_2dot[0],
                                                   desired_state_msg.x_2dot[1],
                                                   desired_state_msg.x_2dot[2],
                                                   desired_state_msg.yaw,
                                                   desired_state_msg.yaw_rate)


    def update_quadrotor_estimated_state_from_vicon(self, estimated_state_msg):
        self.estimated_state = estimated_state_msg
        x = estimated_state_msg.transform.translation.x
        y = estimated_state_msg.transform.translation.y
        z = estimated_state_msg.transform.translation.z
        a = estimated_state_msg.transform.rotation.x
        b = estimated_state_msg.transform.rotation.y
        c = estimated_state_msg.transform.rotation.z
        d = estimated_state_msg.transform.rotation.w
        (roll, pitch, yaw) = euler_from_quaternion([a, b, c, d])
        self.position_controller.set_estimated_state(x,y,z,roll,pitch,yaw)


    def send_cmd_vel(self, event):
        # output roll,pitch,yaw_rate,climb_rate commands
        (pitch, roll, yaw_rate, climb_rate) = self.position_controller.get_pitch_roll_yawrate_climbrate()
        cmd_vel = Twist()
        cmd_vel.linear.x = roll  #roll angle in radian
        cmd_vel.linear.y = pitch  #pitch angle in radian
        cmd_vel.linear.z = climb_rate  #climb rate
        cmd_vel.angular.x = 0
        cmd_vel.angular.y = 0
        cmd_vel.angular.z = yaw_rate #raw rate in radian per second
        self.pub_cmd_vel.publish(cmd_vel)
        self.pub_cmd_vel_RHC.publish(cmd_vel)

if __name__ == '__main__':
    rospy.init_node('aer1217_roscontroller')
    ROSControllerNode()
    rospy.spin()
