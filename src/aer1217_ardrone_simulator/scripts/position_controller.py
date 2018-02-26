#!/usr/bin/env python2

"""Class for writing position controller."""

from __future__ import division, print_function, absolute_import

import roslib
import rospy
import numpy as np
from math import asin,cos,sin,fmod,pi,fabs,sqrt
from numpy.core.umath import deg2rad,rad2deg
from scipy.interpolate import UnivariateSpline
import sys
from std_msgs.msg import Bool

from time import gmtime, strftime
import csv
import os

class PositionController(object):
    """ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""
    # write code here for position controller
    def __init__(self):
        self.old_time_sampling = rospy.get_time()
        self.old_time_control = rospy.get_time()
        self.dt=0
        self.initialized = False
        #input is the desired x,y,z and yaw
        self.desired_x = 0
        self.desired_y = 0
        self.desired_z = 0
        self.desired_yaw = 0
        self.desired_yaw_rate = 0

        self.desired_dx = 0
        self.desired_dy = 0
        self.desired_dz = 0

        self.desired_ddx = 0
        self.desired_ddy = 0
        self.desired_ddz = 0

        self.estimated_x = 0
        self.estimated_y = 0
        self.estimated_z = 0
        self.estimated_roll = 0
        self.estimated_pitch = 0
        self.estimated_yaw = 0

        self.estimation_seq=0 #Leave this to zero
        self.if_use_ddz = True
        self.array_size = 8
        self.estimated_x_array = np.empty(self.array_size)
        self.estimated_y_array = np.empty(self.array_size)
        self.estimated_z_array = np.empty(self.array_size)
        self.estimated_t_array = np.empty(self.array_size)

        self.g=9.81

        #value 0 is finite difference derivative
        #value 1 is spline interpolated derivative (should be smoother)
        self.deriv_scheme = 1

        self.pitch=0
        self.roll=0
        self.yaw_rate=0
        self.climb_rate=0

        #The following are the limit of the ouput
        #any value beyond the limit will be clipped
        self.max_f = 8*abs(self.g)
        self.max_roll_angle=deg2rad(60)
        self.max_pitch_angle = deg2rad(60)
        self.roll_limit = sin(self.max_roll_angle)#precalculated value
        self.pitch_limit = sin(self.max_pitch_angle)#precalculated value

        self.int_delta_x = 0
        self.int_delta_y = 0
        self.ddxc_accu_max = self.max_f*np.fmin(self.roll_limit,self.pitch_limit)
        self.ddyc_accu_max = self.max_f*np.fmin(self.roll_limit, self.pitch_limit)

        self.sub_desired_state = rospy.Subscriber('/aer1217_ardrone/if_trajectory_finished',
                                                  Bool,self.Update_PID)

        # Forwardfeed, P,I,D, 1/Tau_yaw, 1/Tau_z parameters(six in total)
        #[1.0, 0.2070, 0.00081, 3.2700, 1.69, 4.00] #candidate 1
        #[1.0, 0.2438, -0.0081, 3.0017, 1.69, 4.00] #candidate 2
        #[1.000, 0.2497, -0.01671, 3.0017, 1.690, 4.059] #candidate 3
        #[1.000, 0.2391, -0.01671, 3.0495, 1.702, 4.001]#best for linear path E=0.021
        #F:1.000  P:0.2368  I:-0.01669  D:3.0495  y:1.702  Z:4.001 #with dz feedforward--bad
        #self.FPIDyZ =      [1.000, 0.4206, 0.00587, 4.6831, 1.680, 4.204]
        self.running = True
        self.bDisplayLogInCsvFormat = False
        self.bWarining = False
        self.bShowPidState=False
        self.FPIDyZ =      [1.000, 0.4306, 0.00686, 4.4413, 4.643, 4.204]
        if(self.running==False):
            self.dFPIDyZ = [0.000, 0.0050, 0.00010, 0.0261, 0.003, 0.010]
        else:
            self.dFPIDyZ = [0.0, 0.00, 0.000, 0.0, 0.0, 0.0]
        self.coeff_min_idx = 1
        self.coeff_max_idx = len(self.FPIDyZ) - 1
        self.coeff_idx = self.coeff_min_idx

        self.state = 0
        self.best_control_err = float("inf")
        self.control_err = 0
        self.control_seq = 1
        self.t_accu_for_disp=0

        self.current_time = 0

        self.flight_record=[]


    def RecordCurrentState(self):
        self.flight_record.append([self.current_time, self.desired_x, self.desired_y, self.desired_z, self.desired_yaw,
                                   self.estimated_x, self.estimated_y, self.estimated_z, self.estimated_yaw])


    def Update_PID(self, if_trajectory_finished):
        if(if_trajectory_finished.data==True):
            filename = strftime("%Y-%m-%d--%H-%M-%S.csv", gmtime())
            fullpath = os.path.join(os.path.expanduser('~'), 'aer1217', filename)
            rospy.logwarn("Writing record:%s",fullpath)
            with open(fullpath, 'wb') as csvfile:
                spamwriter = csv.writer(csvfile, delimiter=' ',quotechar='|', quoting=csv.QUOTE_MINIMAL)
                for entry in self.flight_record:
                    spamwriter.writerow(entry)
            del self.flight_record[:]
        #Not used
        if(self.running == True):
            if(if_trajectory_finished.data==False):
                return

            if(if_trajectory_finished.data==True):
                self.reset_controller()
                return
        else:
            if(if_trajectory_finished.data==False):
                return

        if(self.state==0):
            self.control_err /= self.control_seq
            self.best_control_err =self.control_err
            self.control_err=0
            self.FPIDyZ[self.coeff_idx]+=self.dFPIDyZ[self.coeff_idx]
            self.state=1
        elif(self.state==1):
            self.control_err /= self.control_seq
            if(self.control_err<self.best_control_err):
                self.best_control_err=self.control_err
                self.dFPIDyZ[self.coeff_idx] *= 1.1
                self.coeff_idx+=1
                if (self.coeff_idx > self.coeff_max_idx): self.coeff_idx = self.coeff_min_idx
                self.FPIDyZ[self.coeff_idx] += self.dFPIDyZ[self.coeff_idx]
            else:
                self.FPIDyZ[self.coeff_idx] -= 2 * self.dFPIDyZ[self.coeff_idx]
                self.state=2
        elif(self.state==2):
            self.control_err /= self.control_seq
            if(self.control_err<self.best_control_err):
                self.best_control_err = self.control_err
                self.dFPIDyZ[self.coeff_idx] *= 1.1
                self.coeff_idx += 1
                if (self.coeff_idx > self.coeff_max_idx): self.coeff_idx = self.coeff_min_idx
                self.FPIDyZ[self.coeff_idx] += self.dFPIDyZ[self.coeff_idx]
            else:
                self.FPIDyZ[self.coeff_idx] += self.dFPIDyZ[self.coeff_idx]
                self.dFPIDyZ[self.coeff_idx] *= 0.9
                self.coeff_idx += 1
                if (self.coeff_idx > self.coeff_max_idx): self.coeff_idx = self.coeff_min_idx
                self.FPIDyZ[self.coeff_idx] += self.dFPIDyZ[self.coeff_idx]
            self.state = 1
        self.reset_controller()
        return

    def reset_controller(self):
        #reinitialize the controller
        self.initialized = False
        self.estimation_seq=0
        self.control_err = 0
        self.control_seq = 1
        self.int_delta_x = 0
        self.int_delta_y = 0

    def set_desired_state(self,x,y,z,dx,dy,dz,ddx,ddy,ddz,yaw,yaw_rate):
        #get new waypoint for the trajectory planner
        self.desired_x = x
        self.desired_y = y
        self.desired_z = z
        self.desired_dx = dx
        self.desired_dy = dy
        self.desired_dz = dz
        self.desired_ddx = ddx
        self.desired_ddy = ddy
        self.desired_ddz = ddz
        self.desired_yaw = yaw
        self.desired_yaw_rate = yaw_rate

    def set_estimated_state(self,x,y,z,roll,pitch,yaw):
        #get measurement for VICON
        self.current_time = rospy.get_time()
        dt = self.current_time - self.old_time_sampling

        if(dt<0.001):
            return
        self.dt_sampling = dt
        self.estimated_x_array =np.roll(self.estimated_x_array, -1)
        self.estimated_y_array =np.roll(self.estimated_y_array, -1)
        self.estimated_z_array =np.roll(self.estimated_z_array, -1)
        self.estimated_t_array =np.roll(self.estimated_t_array, -1)
        self.estimated_x_array[-1] = x
        self.estimated_y_array[-1] = y
        self.estimated_z_array[-1] = z
        if(self.deriv_scheme==0):
            self.estimated_t_array[-1] = self.dt_sampling
        elif(self.deriv_scheme==1):
            self.estimated_t_array[-1] = self.current_time
        self.estimation_seq+=1

        #Get enough data before starts
        if(self.estimation_seq>self.array_size):
            self.initialized=True

        self.estimated_x = x
        self.estimated_y = y
        self.estimated_z = z
        self.estimated_roll = roll
        self.estimated_pitch = pitch
        self.estimated_yaw = yaw
        self.old_time_sampling=self.current_time

    def get_pitch_roll_yawrate_climbrate(self):
        #The actual PID controll loop
        self.current_time = rospy.get_time()
        dt = self.current_time - self.old_time_control

        if(self.initialized == False or dt < 0.001):
            return (0.0, 0.0, 0.0, 0.0)

        self.RecordCurrentState()

        self.dt_control=dt
        self.old_time_control = self.current_time
        self.t_accu_for_disp += dt
        self.control_seq+=1
        #####################################################################################
        #compute 2nd order derivative of Z
        if(self.if_use_ddz):
            if (self.deriv_scheme == 0):
                estimated_dz_array = np.gradient(self.estimated_z_array,self.estimated_t_array)
                estimated_ddz_array = np.gradient(estimated_dz_array, self.estimated_t_array)
            elif(self.deriv_scheme == 1):
                z_spl = UnivariateSpline(self.estimated_t_array, self.estimated_z_array, s=self.array_size, k=3)
                z_spl_2d = z_spl.derivative(n=2)
                estimated_ddz_array = z_spl_2d(self.estimated_t_array)
            ddz=estimated_ddz_array[-1]
        else:
            ddz=0#set to zero as the handnote suggested
        ##################################################################################
        #compute first order derivative of X
        if (self.deriv_scheme == 0):
            estimated_dx_array = np.gradient(self.estimated_x_array, self.estimated_t_array)
        elif (self.deriv_scheme == 1):
            x_spl = UnivariateSpline(self.estimated_t_array, self.estimated_x_array, s=self.array_size, k=2)
            x_spl_d = x_spl.derivative(n=1)
            estimated_dx_array = x_spl_d(self.estimated_t_array)
        dx = estimated_dx_array[-1]
        #################################################################################
        #compute first order derivative of Y
        if (self.deriv_scheme == 0):
            estimated_dy_array = np.gradient(self.estimated_y_array, self.estimated_t_array)
        elif (self.deriv_scheme == 1):
            y_spl = UnivariateSpline(self.estimated_t_array, self.estimated_y_array, s=self.array_size, k=2)
            y_spl_d = y_spl.derivative(n=1)
            estimated_dy_array = y_spl_d(self.estimated_t_array)
        dy = estimated_dy_array[-1]
        ##################################################################################
        #compute the thrust
        f = (ddz+self.g)/(cos(self.estimated_roll)*cos(self.estimated_pitch))
        if(f > self.max_f):
            if(self.bWarining):rospy.logwarn("f saturated:%0.3f", f)
            #f = self.max_f
        elif(f < (-self.max_f)):
            if (self.bWarining):rospy.logwarn("f saturated:%0.3f", f)
            #f = -self.max_f
        #############################################################
        #Compute yaw rate using P control with Feedforward term
        delta_yaw = fmod(self.desired_yaw - self.estimated_yaw,2*pi)
        if(delta_yaw<-pi):
            delta_yaw+=2*pi
        elif(delta_yaw>pi):
            delta_yaw-=2*pi
        self.yaw_rate = delta_yaw * self.FPIDyZ[4] + self.desired_yaw_rate
        #############################################################
        # Compute climb rate using P control with Feedforward term
        delta_z= (self.desired_z - self.estimated_z)
        self.climb_rate = delta_z * self.FPIDyZ[5] + self.desired_dz
        #############################################################
        #PID controller of the acceleration in X and Y with Feedforward terms
        delta_x = self.desired_x - self.estimated_x
        delta_y = self.desired_y - self.estimated_y
        self.int_delta_x += delta_x * self.dt
        self.int_delta_y += delta_y * self.dt
        d_delta_x = self.desired_dx - dx
        d_delta_y = self.desired_dy - dy
        self.control_err += delta_x**2+delta_y**2+delta_z**2+delta_yaw**2
        if(self.int_delta_x > self.ddxc_accu_max):
            if (self.bWarining): rospy.logwarn("Ix saturated:%0.3f", self.int_delta_x)
            self.int_delta_x = self.ddxc_accu_max
        elif(self.int_delta_x < -self.ddxc_accu_max):
            if (self.bWarining): rospy.logwarn("Ix saturated:%0.3f", self.int_delta_x)
            self.int_delta_x = -self.ddxc_accu_max

        if (self.int_delta_y > self.ddyc_accu_max):
            if (self.bWarining): rospy.logwarn("Iy saturated:%0.3f", self.int_delta_y)
            self.int_delta_y = self.ddyc_accu_max
        elif (self.int_delta_y < -self.ddyc_accu_max):
            if (self.bWarining): rospy.logwarn("Iy saturated:%0.3f", self.int_delta_y)
            self.int_delta_y = -self.ddyc_accu_max

        ddxc = self.FPIDyZ[0]*self.desired_ddx + self.FPIDyZ[1] * delta_x + self.FPIDyZ[2] * self.int_delta_x + self.FPIDyZ[3] * d_delta_x
        ddyc = self.FPIDyZ[0]*self.desired_ddy + self.FPIDyZ[1] * delta_y + self.FPIDyZ[2] * self.int_delta_y + self.FPIDyZ[3] * d_delta_y
        ##################################################################################
        #Normalize the acceleration using the thrust (As the note described)
        ddxc/=f
        ddyc/=f
        ############################################################
        # Apply rotation matrix to compute the acceleration in roll direction
        sin_roll = ddxc*sin(self.estimated_yaw) - ddyc*cos(self.estimated_yaw)

        if(sin_roll>self.roll_limit):
            self.roll = self.max_roll_angle
            if (self.bWarining):rospy.logwarn("roll saturated, sin_val:%0.3f", sin_roll)
        elif(sin_roll<-self.roll_limit):
            self.roll = -self.max_roll_angle
            if (self.bWarining):rospy.logwarn("roll saturated, sin_val:%0.3f", sin_roll)
        else:
            self.roll = asin(sin_roll)
        ##################################################################################
        #Apply rotation matrix to compute the acceleration in pitch direction
        sin_pitch =  (ddxc*cos(self.estimated_yaw) + ddyc*sin(self.estimated_yaw))/(cos(self.roll))
        if(sin_pitch>self.pitch_limit):
            self.pitch = self.max_pitch_angle
            if (self.bWarining):rospy.logwarn("pitch saturated, sin_val:%0.3f", sin_pitch)
        elif(sin_pitch<-self.pitch_limit):
            self.pitch = -self.max_pitch_angle
            if (self.bWarining):rospy.logwarn("pitch saturated, sin_val:%0.3f", sin_pitch)
        else:
            self.pitch = asin(sin_pitch)
        #############################################################
        #Display the result
        if(self.t_accu_for_disp>1):
            self.t_accu_for_disp=0
            if(self.state==1):sign='+'
            elif(self.state==2):sign='-'
            else:sign='='
            if(self.pitch>=0):pm='Forward '
            else:             pm='Backward'
            if (self.roll >= 0):rm = 'Right'
            else:               rm = 'Left '

            if(self.running == True):
                if(self.bDisplayLogInCsvFormat==True):
                    rospy.loginfo(",%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f",
                                  self.current_time,self.desired_x,self.desired_y,self.desired_z,self.desired_yaw,
                                self.estimated_x,self.estimated_y,self.estimated_z,self.estimated_yaw)
                else:
                    if(self.bShowPidState==True):
                        rospy.loginfo("=====================================================")
                        rospy.loginfo(" F:%0.3f  P:%0.4f  I:%0.5f  D:%0.4f  y:%0.2f  Z:%0.2f", self.FPIDyZ[0], self.FPIDyZ[1],
                                      self.FPIDyZ[2], self.FPIDyZ[3], self.FPIDyZ[4], self.FPIDyZ[5])
                        rospy.loginfo("Exy:%0.2f Ez:%0.2f E:%0.3f Fs:%0.0fHz, Fc:%0.0fHz", sqrt(delta_x**2+delta_y**2), delta_z,
                                      self.control_err / self.control_seq, 1 / self.dt_sampling, 1 / self.dt_control)
                        rospy.loginfo("xest:%0.5f yest:%0.5f zest:%0.5f", self.estimated_x, self.estimated_y, self.estimated_z)
                        rospy.loginfo("dx:%0.5f dy:%0.5f dz:%0.5f", self.desired_dx, self.desired_dy, self.desired_dz)
                        rospy.loginfo("ddx:%0.5f ddy:%0.5f ddz:%0.5f", self.desired_ddx, self.desired_ddy, self.desired_ddz)
                        rospy.loginfo("p:%0.3f r:%0.3f dy:%0.3f dz:%0.3f",
                                      rad2deg(self.pitch), rad2deg(self.roll),
                                      rad2deg(self.yaw_rate), self.climb_rate)
                        rospy.loginfo("Acceleration: %s   %s", pm, rm)
            else:
                if (self.bShowPidState == True):
                    rospy.loginfo("========== index:%d/%d(%s) ==========",self.coeff_idx,self.coeff_max_idx,sign)
                    rospy.loginfo(" F:%0.3f  P:%0.4f  I:%0.5f  D:%0.4f  y:%0.3f  Z:%0.3f", self.FPIDyZ[0], self.FPIDyZ[1], self.FPIDyZ[2], self.FPIDyZ[3], self.FPIDyZ[4], self.FPIDyZ[5])
                    rospy.loginfo("dF:%0.3f dP:%0.4f dI:%0.5f dD:%0.4f dy:%0.3f dZ:%0.3f", self.dFPIDyZ[0], self.dFPIDyZ[1], self.dFPIDyZ[2], self.dFPIDyZ[3], self.dFPIDyZ[4], self.dFPIDyZ[5])
                    rospy.loginfo("E:%0.3f Ez:%0.2f cE:%0.3f Fs:%0.0fHz, Fc:%0.0fHz", self.best_control_err, delta_z, self.control_err / self.control_seq,1/self.dt_sampling, 1/self.dt_control)
                    rospy.loginfo("dx:%0.5f dy:%0.5f dz:%0.5f dyaw:%0.5f", self.desired_dx, self.desired_dy, self.desired_dz,self.desired_yaw_rate)
                    rospy.loginfo("ddx:%0.5f ddy:%0.5f ddz:%0.5f",self.desired_ddx,self.desired_ddy,self.desired_ddz)
                    rospy.loginfo("p:%0.3f r:%0.3f dy:%0.3f dz:%0.3f",
                                  rad2deg(self.pitch),rad2deg(self.roll),
                                  rad2deg(self.yaw_rate),self.climb_rate)
                    rospy.loginfo("Acceleration: %s   %s",pm,rm)

        #send the command to the attitude and motor controller
        return (self.pitch,self.roll,self.yaw_rate,self.climb_rate)

