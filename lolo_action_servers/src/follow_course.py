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



import numpy as np
import math
import geometry as geom
import rospy, tf, actionlib
import time

from lolo import Lolo
from ros_lolo import ROSLolo


#from smarc_bt.msg import GotoWaypoint
from smarc_bt.msg import FollowCourseFeedback, FollowCourseResult, FollowCourseAction, FollowCourseGoal


def get_param(name, default=None):
    v = rospy.get_param(rospy.search_param(name), default)
    print("got rosparam name:{}, val:{}".format(name, v))
    if type(v) == type({}):
        print("{} returned a dict! Defaulting to {}".format(name, default))
        v = default
    return v

class LoloFollowCourse(object):
    def __init__(self):
        
        self.start_time = None
        # see launch/config.yaml
        self.lolo = Lolo(max_rpm = get_param("max_rpm", 500))

        self.ros_lolo = ROSLolo(lolo = self.lolo,
                                robot_name = get_param("robot_name", "lolo"),
                                update_freq = get_param("controller_update_freq", 10),
                                max_rpm = get_param("max_rpm", 500))

        self.update_freq = get_param("action_update_freq", 10)

        self.tf_listener = tf.TransformListener()

        self.name = rospy.get_name()
        self.reset_fb_result()
        self.action_server = actionlib.SimpleActionServer(self.name,
                                                          FollowCourseAction,
                                                          execute_cb = self.run,
                                                          auto_start = False)

    ###################################################
    # do something every tick here
    # ideally you shouldnt need to think about ros at all
    # inside this funtion
    ###################################################
    def update(self):
        self.lolo.update()

    ###################################################
    # action server piping, shouldnt need modification most of the time
    ###################################################
    def feedback(self, msg):
        s = "{} [{}]".format(msg, self.name)
        self.fb.feedback_message = s
        rospy.loginfo_throttle(5, s)
        self.action_server.publish_feedback(self.fb)

    def start(self):
        self.action_server.start()
        rospy.loginfo("Started!")

    def on_preempt(self):
        self.ros_lolo.stop()
        self.feedback("Pre-empted!")
        self.action_server.set_preempted(self.result, "[{}] preempted!".format(self.name))
        self.reset_fb_result()

    def on_done(self):
        self.ros_lolo.stop()
        self.feedback("Completed!")
        self.result.done = True
        self.action_server.set_succeeded(self.result, "[{}] Succeeded!")
        self.reset_fb_result()

    def on_new_goal(self):
        print("")
        self.feedback("Got goal!")
        self.reset_fb_result()
        self.ros_lolo.start()

    def reset_fb_result(self):
        self.fb = FollowCourseFeedback()
        self.result = FollowCourseResult()

    def run(self, goal : FollowCourseGoal):
        self.on_new_goal()
        
        start_time = time.time()
        rospy.loginfo("start time set!")

        target_course = math.atan2( math.cos( math.radians(goal.targetheading_deg)), math.sin( math.radians(goal.targetheading_deg)) )
        rospy.loginfo("Target heading: " + str(goal.targetheading_deg) + ", target course: " + str(math.degrees(target_course)))
        
        # acquire the depth
        altitude = None
        
        depth = goal.targetDepth
        altitude = goal.targetAltitude
        rpm = goal.rpm
        
        # set internal goal from message params
        self.lolo.set_course_goal(depth = depth,
                           rpm = rpm,
                           course = target_course,
                           altitude = altitude)
        


        # and finally, we start spinning and controlling things
        rate = rospy.Rate(self.update_freq)
        while not rospy.is_shutdown():
            if self.action_server.is_preempt_requested():
                self.on_preempt()
                # return, not break!
                return
            
            runtime = time.time() - start_time
            time_left = goal.runtime_s - runtime

            cm = self.lolo.control_mode
            if cm == Lolo.DRIVE:
                self.feedback("target course: "+ str(math.degrees(target_course)) + ", runtime:" + str(runtime ) + ", time left=" + str(time_left))
            else:
                self.feedback(cm)

            #Check if it's time to stop
            if time_left < 0:
                # success~
                break

            self.update()
            rate.sleep()

        # finished running
        self.on_done()

if __name__ == "__main__":
    rospy.init_node("follow_course")
    s = LoloFollowCourse()
    s.start()
    rospy.loginfo("Spinning~")
    rospy.spin()
