#!/usr/bin/env python2

"""ROS Node for publishing desired positions."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from aer1217_ardrone_simulator.msg import DesiredStateMsg
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from ad import adnumber
from ad.admath import *

from math import floor


class ROSDesiredPositionGenerator(object):
    """ROS interface for publishing desired positions."""
    # write code here for desired position trajectory generator

    def __init__(self):
        # publisher for new way point
        self.pub_desired_state = rospy.Publisher('/aer1217_ardrone/desired_state', DesiredStateMsg, queue_size=30)
        self.pub_desired_state_rviz = rospy.Publisher('/aer1217_ardrone/desired_state_rviz', PoseStamped, queue_size=30)
        self.pub_task_state = rospy.Publisher('/aer1217_ardrone/if_task_finished', Bool, queue_size=300)
        rospy.wait_for_service('/gazebo/reset_world')
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        # publish new way point every 1.0 second
        self.pub_prop_vel = rospy.Timer(rospy.Duration(.1), self.update_desired_state)

        # value 0 for circular path, 1 for linear path
        self.shape = 0
        # trajectory global setting
        self.speed = 0.05
        self.time_offset = 0
        # for rviz
        self.seq = 0
        self.ResetWorld()

    def ResetWorld(self):
        # straight line trajectory definition
        if (self.shape == 1):
            self.x0 = -1
            self.y0 = 2
            self.z0 = 1
            self.x1 = 1
            self.y1 = 0
            self.z1 = 2
            self.segment_index = 0
            self.segment_index_max = 1
            self.norm = sqrt((self.x1 - self.x0)**2 + (self.y1 - self.y0)**2 + (self.z1 - self.z0)**2)
            self.t_max = self.norm / self.speed
            self.yaw_initial = atan2((self.y1 - self.y0), (self.x1 - self.x0))
        # circular trajectory defition
        elif(self.shape == 0):
            self.R = 1
            self.xc = 0
            self.yc = 1
            self.z0 = 1.5
            self.z1 = 1.5
            self.norm = abs(self.z1 - self.z0)
            self.omega = self.speed / self.R
            self.initial_phase = 0  # -pi * 0.5
            self.t_max = 2 * 2 * pi / self.omega
        self.reset_world()

    def update_desired_state(self, event):
        system_time = rospy.get_time()
        t = adnumber(system_time)
        if (self.shape == 1):
            if(t > (self.t_max + self.time_offset) and self.segment_index < self.segment_index_max):
                self.x0, self.x1 = self.x1, self.x0
                self.y0, self.y1 = self.y1, self.y0
                self.z0, self.z1 = self.z1, self.z0
                self.segment_index += 1
                self.time_offset = system_time
                self.norm = sqrt((self.x1 - self.x0) ** 2 + (self.y1 - self.y0) ** 2 + (self.z1 - self.z0) ** 2)
                self.t_max = self.norm / self.speed
            x = self.x0 + (self.x1 - self.x0) * self.speed * (t - self.time_offset) / self.norm
            y = self.y0 + (self.y1 - self.y0) * self.speed * (t - self.time_offset) / self.norm
            z = self.z0 + (self.z1 - self.z0) * self.speed * (t - self.time_offset) / self.norm
            yaw_dot = 0
        elif (self.shape == 0):
            x = self.xc + self.R * cos(self.omega * (t - self.time_offset) + self.initial_phase)
            y = self.yc + self.R * sin(self.omega * (t - self.time_offset) + self.initial_phase)
            z = adnumber(self.z0)  # self.z0 + (self.z1 - self.z0) * self.speed * (t-self.time_offset)/self.normw            yaw_dot =self.omega
        # calculate derivative of xyz wrt time
        dx = x.d(t)
        dy = y.d(t)
        dz = z.d(t)
        # second order derivative
        ddx = x.d2(t)
        ddy = y.d2(t)
        ddz = z.d2(t)
        if(self.shape == 0):
            yaw = atan2(dy, dx)
        if (self.shape == 1):
            yaw = self.yaw_initial
        # pack the data
        waypoint = DesiredStateMsg()
        waypoint.x = [x, y, z]
        waypoint.x_dot = [dx, dy, dz]
        waypoint.x_2dot = [ddx, ddy, ddz]
        waypoint.yaw = yaw
        waypoint.yaw_rate = yaw_dot
        if(0):
            waypoint.x = [0, 0, 1]
            waypoint.x_dot = [0, 0, 0]
            waypoint.x_2dot = [0, 0, 0]
            waypoint.yaw = 0
            waypoint.yaw_rate = 0
        #rospy.logerr("get the first derivative wrt t:%f", dx)
        self.pub_desired_state.publish(waypoint)
        # publish for rviz
        pose_stamped = PoseStamped()
        pose_stamped.header.seq = self.seq
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "/map"
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        quat = quaternion_from_euler(0, 0, yaw)
        pose_stamped.pose.orientation.x = quat[0]
        pose_stamped.pose.orientation.y = quat[1]
        pose_stamped.pose.orientation.z = quat[2]
        pose_stamped.pose.orientation.w = quat[3]
        self.pub_desired_state_rviz.publish(pose_stamped)
        self.seq += 1
        task_state = Bool()
        if(t > self.t_max + self.time_offset):
            task_state.data = True
            self.time_offset = system_time
            self.ResetWorld()
        else:
            task_state.data = False
        self.pub_task_state.publish(task_state)


if __name__ == '__main__':
    rospy.init_node('desired_positions')
    ROSDesiredPositionGenerator()
    rospy.spin()
