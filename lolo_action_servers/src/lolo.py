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

class PID(object):

    def __init__(self,kP=None,kI=None,kD=None,max_output = None):
        self.kP = kP if kP is not None else 0
        self.kI = kI if kI is not None else 0
        self.kD = kD if kD is not None else 0
        self.max_output = max_output
        self.integral = 0
        self.last_update = None
        self.last_error = None
    
    def reset(self):
        self.integral = 0
        self.last_update = None
        self.last_meassurement = None

    def update_error(self, error):
        current_time_s = rospy.get_time()
        dt = current_time_s - self.last_update if self.last_update is not None else None

        proportional = self.kP*error
        if dt is not None: self.integral +=  dt*self.kI*error 
        derivative = self.kD*(error - self.last_error) / dt if dt is not None else 0

        #prevent integral windup
        if self.max_output is not None:
            if(self.integral > self.max_output): self.integral = self.max_output
            if(self.integral < -self.max_output): self.integral = -self.max_output

        output = proportional + self.integral + derivative

        self.last_update = current_time_s
        self.last_error = error

        return max(-self.max_output, min(self.max_output, output)) if self.max_output is not None else output
    
    def update(self, meassurement, setpoint):
        error = setpoint - meassurement
        return self.update_error(error)



class Lolo(object):
    IDLE = "IDLE"
    DRIVE = "DRIVE"

    def __init__(self,
                 max_rpm = 100,
                 useless_rudder_depth = 0.8):
        """
        A container object that abstracts away ros-related stuff for a nice abstract vehicle
        pose is in NED, x = north, y = east, z = down/depth
        """

        self.max_rpm = max_rpm
        self.useless_rudder_depth = useless_rudder_depth

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
        self.desired_yawRate = 0
        self.desired_pitchRate = 0
        self.desired_rollRate = 0

        #Vehicle actuator vaues
        self.thruster_rpms = np.zeros(2)
        self.desired_rpms = np.zeros(2)

        self.elevon_angles = np.zeros(2)
        self.desired_elevon_angles = np.zeros(2)

        self.rudder_angle = 0
        self.desired_rudder_angle = 0

        self.elevator_angle = 0
        self.desired_elevator_angle = 0

        #Pid controllers
        self.depth_PID = PID(0.1,0,0, np.radians(20)) # max 20 deg pitch
        self.speed_PID = PID(10,0,0, 1000) #1000 RPM max output
        self.pitch_PID = PID(1,0.1,0, np.radians(2)) #Max 10 deg/s pitch
        self.roll_PID = PID(0.1,0,0, np.radians(30))   #Max 10 deg/s roll
        self.yaw_PID = PID(2,0,0, np.radians(5))  #Max 10 deg/s yaw

        #Rate PIDs not used at the moment. 
        self.pitch_rate_PID = PID(2,0,0, np.radians(30))  #max 30 deg elevator angle
        self.roll_rate_PID = PID(1,0,0, np.radians(30))  #max 30 deg elevon angle
        self.yaw_rate_PID = PID(2,0.1,0, np.radians(30)) #max 30 deg rudder angle

    def _reset_desires(self):
        print("Reset setpoints to 0 and reset controllers")
        self.desired_yaw = 0
        self.desired_pitch = 0
        self.desired_roll = 0
        self.desired_yawRate = 0
        self.desired_pitchRate = 0
        self.desired_rollRate = 0
        self.desired_elevator_angle = 0
        self.desired_rudder_angle = 0
        self.desired_elevon_angles[0] = 0
        self.desired_elevon_angles[1] = 0
        self.desired_rpms[0] = 0
        self.desired_rpms[1] = 0

        #Reset PID since mode changed. Is this needed?
        self.depth_PID.reset()
        self.speed_PID.reset()
        self.pitch_PID.reset()
        self.roll_PID.reset()
        self.yaw_PID.reset()
        self.pitch_rate_PID.reset()
        self.roll_rate_PID.reset()
        self.yaw_rate_PID.reset()

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
            self.control_wp()
            self.control_depth()
            self.control_speed()
            #Low level
            self.control_pitch()
            self.control_roll()
            self.control_yaw()
            self.control_pitchRate()
            self.control_rollRate()
            self.control_yawRate()
            return
    
    #High level
    def control_wp(self):
        #set setpoint for yaw
        self.desired_yaw = np.arctan2(self.position_error[1], self.position_error[0])
        #self.desired_depth = 
        
        #self.desired_yaw = geom.vec2_directed_angle(self.yaw_vec, np.array([np,cos(self.desired_yaw) , np.sin(self.desired_yaw)]))
        #print("desired yaw: " + str(np.rad2deg(self.desired_yaw)) +  " current yaw: " + str(np.rad2deg(self.yaw)))
        

    def control_depth(self):
        #set setpoint for pitch based on depth setpoint or altitude
        desired_depth = self.goal.depth
        if(self.target_altitude is not None): #We have a target altitude
            altitude_target_depth = self.depth + 10 #Default value if no seafloor is detected
            if self.altitude is not None:
                total_depth = self.depth +self.altitude
                altitude_target_depth = total_depth - self.target_altitude
            desired_depth = min(self.goal.depth, altitude_target_depth)
            print("Altitude control!")
            if self.altitude is not None : print("total depth : " + str(self.depth +self.altitude))
            print("target altitude: " + str(self.target_altitude))
            print("current altitude: " + str(self.altitude))
            print("max depth: " + str(self.goal.depth))

        self.desired_pitch = self.depth_PID.update(self.depth, desired_depth)
        print("desired depth : " + str(desired_depth))
        print("current depth : " + str(self.depth))
        print("desired pitch : " + str(np.rad2deg(self.desired_pitch)))

    def control_speed(self):
        #set setpoints for RPM based on speed setpoint
        self.desired_rpms[0] = self.goal.rpm
        self.desired_rpms[1] = self.goal.rpm

    #low level level
    def control_pitch(self):
        #set setpoint for pitchrate
        self.desired_pitchRate = self.pitch_PID.update(self.pitch, self.desired_pitch)
        print("\tdesired pitch : " + str(np.rad2deg(self.desired_pitch)))
        print("\tcurrent pitch : " + str(np.rad2deg(self.pitch)))
        print("\tdesired pitchrate angle : " + str(np.rad2deg(self.desired_pitchRate)))

    def control_roll(self):
        #set setpoint for rollrate
        #self.desired_roll = np.deg2rad(5)
        self.desired_rollRate = self.roll_PID.update(self.roll, self.desired_roll)

    def control_yaw(self):
        #set setpoint for yawrate
        yaw_diff = geom.vec2_directed_angle(self.yaw_vec, np.array([np.cos(self.desired_yaw) , np.sin(self.desired_yaw)]))
        self.desired_yawRate = self.yaw_PID.update_error(yaw_diff)
        #print("Desired Yaw: " + str(np.rad2deg(self.desired_yaw)))
        #print("current Yaw: " + str(np.rad2deg(self.yaw)))
        #print("Yaw diff: " + str(np.rad2deg(yaw_diff)))
        #print("Desired rudder angle: " + str(np.rad2deg(self.desired_yawRate)))

    def control_pitchRate(self):
        #set setpoint for elevator and elevons
        self.desired_elevator_angle = -self.pitch_rate_PID.update(self.pitchRate, self.desired_pitchRate)
        print("\t\tdesired pitchRate : " + str(np.rad2deg(self.desired_pitchRate)))
        print("\t\tcurrent pitchRate : " + str(np.rad2deg(self.pitchRate)))
        print("\t\tdesired elevator angle : " + str(np.rad2deg(self.desired_elevator_angle)))

    def control_rollRate(self):
        #set setpoint for elevons
        actuation = self.roll_rate_PID.update(self.rollRate, self.desired_rollRate)
        self.desired_elevon_angles[0] = actuation + self.desired_elevator_angle
        self.desired_elevon_angles[1] = -actuation + self.desired_elevator_angle

    def control_yawRate(self):
        #set setpoint for rudders (and thrusters based on speed)
        self.desired_rudder_angle = self.yaw_rate_PID.update(self.yawRate, self.desired_yawRate)

        rpm_fade_out = max(0, 0.8 - abs(self.vx)) if self.depth > 1 else 1
        rpm_actuation = max(-500, min( 500, 1000*self.desired_rudder_angle)) * rpm_fade_out
        self.desired_rpms[0] += rpm_actuation
        self.desired_rpms[1] -= rpm_actuation
        #print("\t\tDesired yawrate: " + str(np.rad2deg(self.desired_yawRate)) + " deg/s")
        #print("\t\tCurrent yawrate: " + str(np.rad2deg(self.yawRate)) + " deg/s")
        #print("\t\tDesired rudder_angle = " + str(np.rad2deg(self.desired_rudder_angle)))
        
    
    


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
        print("\tlolo pos: (" + str(self.x) + ", " + str(self.y) + ")")

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