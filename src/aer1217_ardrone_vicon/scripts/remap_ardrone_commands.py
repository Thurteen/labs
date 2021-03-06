#!/usr/bin/env python2

"""ROS Node for interfacing with ARDrone Autonomy package.

Optional: boundary breach can be detected and remap will be adjusted
accordingly.

2017-02-22 -- Rikky Duivenvoorden

This ROS node subscribes to the following topics:
/cmd_vel_RHC
/vicon/ARDroneCarre/ARDroneCarre

/cmd_vel_RHC conventions:
+linear.x: roll right
+linear.y: pitch down
+linear.z: move up
+angular.z: yaw CCW from above

This ROS node publishes to the following topics:
/cmd_vel

ARDrone_autonomy conventions:
+linear.x: pitch down
+linear.y: roll left
+linear.z: move up
+angular.z: yaw CCW from above
"""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

# Import class that computes the desired positions
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Empty

__all__ = ['ROSARDroneAutonomyInterface']

class ROSARDroneAutonomyInterface(object):
    """ROS interface for emulating the lab equipment."""

    def __init__(self):
        """Initialize the ROSLabInterface class."""
        super(ROSARDroneAutonomyInterface, self).__init__()

        # Publisher
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=50)

        # for landing command when boundary is breached
        self.pubLand    = rospy.Publisher('/ardrone/land',Empty)

        # Subscribers
        self.sub_cmd_vel_RHC = rospy.Subscriber('cmd_vel_RHC', Twist,
                                            self.remap_ideal_command)

        self.sub_vicon = rospy.Subscriber('/vicon/ARDroneCarre/ARDroneCarre',
                                          TransformStamped,
                                          self.check_boundary_breach,
                                          queue_size=1)

        # Initialize messages for publishing
        self.cmd_vel_msg = Twist()
        
        # Parameters for remapping
        self.euler_angle_limit = 1.
        self.climb_rate_limit = 2.
        self.yaw_rate_limit = np.pi / 2
        
        self._remap_command_is_busy = False
        
        # Boundary breach detect parameters
        self.land_on_breach = True
        self.breach_detected = False
        self._override_land = False
        self.boundary_x = [-2.5, 2.0]
        self.boundary_y = [-2.0, 3.0]
        self.boundary_z = [-1.0, 2.5]
        self.land_breach_pos_tolerance = 0.1
    
    def check_boundary_breach(self, vicon_msg):
        # Extract vicon positions
        pos = [vicon_msg.transform.translation.x,
               vicon_msg.transform.translation.y,
               vicon_msg.transform.translation.z]
        
        if (pos[0] < self.boundary_x[0]
            or pos[0] > self.boundary_x[1]):
            print("Breached x direction boundary: x=", pos[0])
            self.breach_detected = True
            if (pos[0]
                < self.boundary_x[0] * (1 + self.land_breach_pos_tolerance)
                or pos[0]
                > self.boundary_x[1] * (1 + self.land_breach_pos_tolerance)):

                # Then override land
                self._override_land = True

        elif (pos[1] < self.boundary_y[0]
              or pos[1] > self.boundary_y[1]):
            print("Breached y direction boundary: y=", pos[1])
            self.breach_detected = True
            if (pos[1]
                < self.boundary_y[0] * (1 + self.land_breach_pos_tolerance)
                or pos[1]
                > self.boundary_y[1] * (1 + self.land_breach_pos_tolerance)):

                # Then override land
                self._override_land = True

        elif (pos[2] < self.boundary_z[0]
              or pos[2] > self.boundary_z[1]):
            print("Breached z direction boundary: z=", pos[2])
            self.breach_detected = True
            if (pos[2]
                < self.boundary_z[0] * (1 + self.land_breach_pos_tolerance)
                or pos[2]
                > self.boundary_z[1] * (1 + self.land_breach_pos_tolerance)):

                # Then override land
                self._override_land = True
                
        # Send landing
        if self._override_land and self.land_on_breach:
            print("Boundary breach - landing...")
            
            # First go to hover
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.linear.y = 0.0
            self.cmd_vel_msg.linear.z = 0.0
            self.cmd_vel_msg.angular.x = 0.0
            self.cmd_vel_msg.angular.y = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            
            # Publish the hover command in case it is not flying automatically
            self.pub_cmd_vel.publish(self.cmd_vel_msg)
            
            # Then publish land command
            self.pubLand.publish(Empty())

    def remap_ideal_command(self, cmd_vel_RHC_msg):
        """Get the ideal roll-pitch-yaw_rate-climb_rate commands, normalize
        them, remap them to ARDrone Autonomy convention, and publish to
        /cmd_vel for ardrone_autonomy.
        """
        if not self._remap_command_is_busy:
            self._remap_command_is_busy = True
        
            # Get the ideal commands from the nonlinear controller and
            # normalize and clip between -1 and 1
            self.cmd_vel_msg.linear.x = np.clip(cmd_vel_RHC_msg.linear.y /
                                                self.euler_angle_limit, -1.,
                                                1.)
            self.cmd_vel_msg.linear.y = -np.clip(cmd_vel_RHC_msg.linear.x /
                                                self.euler_angle_limit, -1.,
                                                1.)
            self.cmd_vel_msg.angular.z = np.clip(cmd_vel_RHC_msg.angular.z /
                                                 self.climb_rate_limit, -1.,
                                                 1.)
            self.cmd_vel_msg.linear.z = np.clip(cmd_vel_RHC_msg.linear.z /
                                                self.yaw_rate_limit, -1., 1.)
            
            # Pass through unused commands in the event hover is requested
            self.cmd_vel_msg.angular.x = cmd_vel_RHC_msg.angular.x
            self.cmd_vel_msg.angular.y = cmd_vel_RHC_msg.angular.y
            
            
            # Publish the remapped cmd_vel
            self.pub_cmd_vel.publish(self.cmd_vel_msg)
            self._remap_command_is_busy = False


if __name__ == '__main__':
    rospy.init_node('ardrone_autonomy_interface')
    ROSARDroneAutonomyInterface()
    
    rospy.spin()
