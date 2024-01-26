#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8

# Copyright 2023 Ozer Ozkahraman (ozero@kth.se)
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


from __future__ import division, print_function
import numpy as np
import tf
import rospy
import math

import geometry as geom

class SimpleRPMGoal(object):
    def __init__(self,
                 x,
                 y,
                 depth,
                 rpm,
                 tolerance):
        self.x = x
        self.y = y
        self.depth = depth
        self.rpm = rpm
        self.tolerance = tolerance

    @property
    def pos(self):
        return np.array([self.x, self.y, self.depth])

class Lolo(object):
    IDLE = "IDLE"
    DRIVE = "DRIVE"

    def __init__(self,
                 max_rpm = 100):
        """
        A container object that abstracts away ros-related stuff for a nice abstract vehicle
        pose is in NED, x = north, y = east, z = down/depth
        """

        self.max_rpm = max_rpm

        self.goal = None
        self.target_altitude = None
        self.control_mode = Lolo.IDLE

        #Vehicle state values
        self.pos_x = 0
        self.pos_y = 0
        self.pos_depth = 0
        self.altitude = 0
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.pitchRate = 0  # rotation rate around Y axis
        self.rollRate = 0   # rotation rate around X axis
        self.yawRate = 0    # rotation rate around Z axis
        self.vx = 0         # speed in X direcion
        self.vy = 0         # speed in Y direcion
        self.vz = 0         # speed in Z direcrion
        
        #vehicle desired states
        self.desired_yaw = 0
        self.desired_pitch = 0
        self.desired_roll = 0
        
        self.yaw_updated = False
        self.roll_updated = False
        self.depth_updated = False
        self.speed_updated = False

        #Vehicle actuator vaues
        self.thruster_rpms = np.zeros(2)
        self.desired_rpms = np.zeros(2)
        self.desired_rpm = 0

        self.elevon_angles = np.zeros(2)
        self.desired_elevon_angles = np.zeros(2)

        self.rudder_angle = 0
        self.desired_rudder_angle = 0

        self.elevator_angle = 0
        self.desired_elevator_angle = 0

    def _reset_desires(self):
        #print("Reset setpoints to 0 and reset controllers")
        print("_reset_desires no longer used")

    def _change_mode(self, new_mode):
        if new_mode == self.control_mode:
            return
        print("Changing mode: {} -> {}".format(self.control_mode, new_mode))
        self._reset_desires()
        self.control_mode = new_mode

    ######################################
    # Call this when you want lolo to actually control something
    ######################################
    def update(self):
        if self.goal is None:
            self._change_mode(Lolo.IDLE)
            #self._reset_desires()
            return

        ####
        # Get all the diffs towards the goal
        ####
        depth_diff = self.goal.depth - self.depth
        pitch_diff = np.arctan2(depth_diff, self.xy_dist_to_goal) * geom.RADTODEG
        xy_diff = self.position_error[:2]
        yaw_diff = geom.vec2_directed_angle(self.yaw_vec, xy_diff) * geom.RADTODEG

        ####
        # Change mode according to the vehicle state and goal
        ####

        #TODO make special modes for diving and surface
        self._change_mode(Lolo.DRIVE)

        if self.control_mode == Lolo.DRIVE:
            #High level
            #set yaw setpoint
            self.control_wp()

            #set RPM / surge setpoint
            self.control_speed()

            #set roll setpoint
            self.control_roll()

            #set depth+altutude setpoint (only depth for now)
            self.control_depth()
            return
    
    #High level Control
    def control_wp(self):
        #set setpoint for yaw
        self.desired_yaw = np.arctan2(self.position_error[1], self.position_error[0])
        self.yaw_updated = True

    def control_depth(self):
        #set setpoint for depth based on depth setpoint or altitude
        target_depth = min(self.goal.depth, (self.depth+self.altitude) - self.target_altitude) if self.target_altitude is not None and self.altitude is not None else self.goal.depth
        self.desired_depth = target_depth
        self.depth_updated = True

    def control_speed(self):
        #set setpoints for RPM based on speed setpoint
        self.desired_rpm = self.goal.rpm
        self.speed_updated = True

    def control_roll(self):
        #set setpoint for rollrate
        self.desired_roll = 0 #Hard coded for now
        self.roll_updated = True


    ###############################
    ### Outward facing stuff, mostly automated away from this object
    ###############################
    def set_goal(self,x,y,depth,rpm,tolerance, altitude = None):
        self.goal = SimpleRPMGoal(x,y,depth,rpm,tolerance)
        if altitude is not None: self.target_altitude = altitude

    def reset_goal(self):
        self.goal = None
        self.target_altitude = None
        self.update()

    def update_pos(self, x=None, y=None, depth=None):
        if x is not None: self.pos_x = x
        if y is not None: self.pos_y = y
        if depth is not None: self.pos_depth = depth
        #print("\tlolo pos: (" + str(self.x) + ", " + str(self.y) + ")")

    def update_altitude(self, alt=None):
        if alt is not None and alt > 0: self.altitude = alt
        else: self.altitude = None

    def update_ori(self, r=None, p=None, y=None):
        if r is not None: self.roll = r
        if p is not None: self.pitch = p
        if y is not None: self.yaw = y
        #print("Lolo updated attitudes: (rpy) = " + str(np.rad2deg(self.roll)) + ", " + str(np.rad2deg(self.pitch)) + ", " + str(np.rad2deg(self.yaw)))

    def update_angular_vel(self, pitchrate=None, yawrate=None, rollrate=None):
        if pitchrate is not None: self.pitchRate = pitchrate
        if yawrate is not None: self.yawRate = yawrate
        if rollrate is not None: self.rollRate = rollrate
        #print("Angular vel updated" + str(yawrate) + " " + str(self.yawRate))

    def update_linear_vel(self, vx=None, vy=None, vz=None):
        if vx is not None: self.vx = vx
        if vy is not None: self.vy = vy
        if vz is not None: self.vz = vz

    def update_thruster_rpms(self, port=None, strb=None):
        if port is not None: self.thruster_rpms[0] = port
        if strb is not None: self.thruster_rpms[1] = strb

    def update_elevator_angle(self, a):
        self.elevator_angle = a

    def update_elevon_angles(self, port=None, strb=None):
        if port is not None: self.elevon_angles[0] = port
        if strb is not None: self.elevon_angles[1] = strb

    def update_rudder_angle(self, a):
        self.rudder_angle = a

    def update_elevator_angle(self, a):
        self.elevator_angle = a

    def blarg(self):
        self.desired_elevator_angle = np.random.standard_normal()*0.6
        self.desired_rudder_angle = np.random.standard_normal()*0.6
        self.desired_elevon_angles[0] = np.random.standard_normal()*0.6
        self.desired_elevon_angles[1] = np.random.standard_normal()*0.6
        self.desired_rpms[0] = np.random.standard_normal()*250
        self.desired_rpms[1] = np.random.standard_normal()*250


    ###############################
    ### Properties for convenience
    ###############################
    @property
    def x(self):
        return self.pos_x
    @property
    def y(self):
        return self.pos_y
    @property
    def depth(self):
        return self.pos_depth
    @property
    def pos(self):
        return np.array([self.x, self.y, self.depth])
    #@property
    #def roll(self):
        return self.ori_rpy[0]
    #@property
    #def pitch(self):
        return self.ori_rpy[1]
    #@property
    #def yaw(self):
        return self.ori_rpy[2]
    @property
    def yaw_vec(self):
        return np.array([np.cos(self.yaw), np.sin(self.yaw)])
    @property
    def ori_quat(self):
        return tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
    @property
    def port_rpm(self):
        return self.thruster_rpms[0]
    @property
    def strb_rpm(self):
        return self.thruster_rpms[1]
    @property
    def port_elevon_angle(self):
        return self.elevon_angles[0]
    @property
    def strb_elevon_angle(self):
        return self.elevon_angles[1]
    @property
    def position_error(self):
        return self.goal.pos - self.pos
    @property
    def xy_dist_to_goal(self):
        return geom.euclid_distance(self.goal.pos[:2], self.pos[:2])
    @property
    def depth_to_goal(self):
        return self.goal.depth - self.depth