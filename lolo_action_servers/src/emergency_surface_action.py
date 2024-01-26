#! /usr/bin/env python3
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

#Subscribe to the emergency surface topic. Send commands to the abort flag if there is an issue.

from __future__ import division, print_function

import actionlib
import rospy
from smarc_msgs.msg import ThrusterRPM
from std_msgs.msg import Bool
from std_msgs.msg import Float64, Float32
#from move_base_msgs.msg import MoveBaseFeedback, MoveBaseResult, MoveBaseAction
from smarc_bt.msg import GotoWaypointActionFeedback, GotoWaypointResult, GotoWaypointAction

class EmergencySurface(object):

    def execute_cb(self, goal):

        rospy.loginfo("Emergency action initiated")

        r = rospy.Rate(20.) # 10hz
        while not rospy.is_shutdown():

            # Preempted
            if self._as.is_preempt_requested():
                # Publish emergency command
                #self.emergency_pub.publish(False)

                #Enable controllers
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted(GotoWaypointResult(), "Preempted EmergencySurface action")
                return

            # Publish emergency command
            #self.emergency_pub.publish(True)

            
            # set all actuation to idle and float up to the surface
            thruster_msg = ThrusterRPM()
            thruster_msg.rpm = 0

            angle_msg = Float32()
            angle_msg.data = 0

            self.elevator_pub.publish(angle_msg)
            self.rudder_pub.publish(angle_msg)
            self.elevon_port_pub.publish(angle_msg)
            self.elevon_strb_pub.publish(angle_msg)
            self.thruster_port_pub.publish(thruster_msg)
            self.thruster_strb_pub.publish(thruster_msg)

            r.sleep()

        rospy.loginfo('%s: Completed' % self._action_name)

    #def timer_callback(self, event):

    def __init__(self, name):

        """Publish 0 to VBS and disable all controllers"""
        self._action_name = name


        #rospy.Timer(rospy.Duration(2), self.timer_callback)
        #self.emergency_pub = rospy.Publisher(emergency_topic, Bool, queue_size=10)
        
        self.rudder_pub = rospy.Publisher("/lolo/core/rudder_cmd", Float32, queue_size=1)
        self.elevon_port_pub = rospy.Publisher("/lolo/core/elevon_port_cmd", Float32, queue_size=1)
        self.elevon_strb_pub = rospy.Publisher("/lolo/core/elevon_strb_cmd", Float32, queue_size=1)
        self.thruster_port_pub = rospy.Publisher("/lolo/core/thruster1_cmd", ThrusterRPM, queue_size=1)
        self.thruster_strb_pub = rospy.Publisher("/lolo/core/thruster2_cmd", ThrusterRPM, queue_size=1)
        self.elevator_pub = rospy.Publisher("/lolo/core/elevator_cmd", Float32, queue_size=1)
       
        self._as = actionlib.SimpleActionServer(self._action_name, GotoWaypointAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        rospy.loginfo("Announced action server with name: %s", self._action_name)

        rospy.spin()

if __name__ == '__main__':

    rospy.init_node('emergency_surface_action')
    planner = EmergencySurface(rospy.get_name())
