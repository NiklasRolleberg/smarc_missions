#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

import py_trees as pt
import py_trees_ros as ptr

import time
import math
import numpy as np

import rospy
import tf
import actionlib

from smarc_msgs.msg import FloatStamped
from smarc_bt.msg import GotoWaypointAction, GotoWaypointGoal, GotoWaypoint, MissionControl, Maneuver
from smarc_bt.msg import FollowCourseAction, FollowCourseGoal
from smarc_msgs.srv import UTMToLatLon, LatLonToUTM
import actionlib_msgs.msg as actionlib_msgs
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Header, Bool, Empty
from visualization_msgs.msg import MarkerArray
from geographic_msgs.msg import GeoPoint
# from sensor_msgs.msg import NavSatFix
from mission_plan import Maneuver as Mission_Plan_Maneuver

from std_srvs.srv import SetBool


import bb_enums
import common_globals

from mission_plan import MissionPlan, Waypoint


class A_AbortPlan(pt.behaviour.Behaviour):
    def __init__(self):
        self.bb = pt.blackboard.Blackboard()
        super(A_AbortPlan, self).__init__(name="A_AbortPlan")

    def update(self):
        plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if plan is None:
            self.feedback_message = "No plan"
            return pt.Status.FAILURE

        if plan.state == MissionControl.FB_EMERGENCY:
            self.feedback_message = "Aborted"
            return pt.Status.SUCCESS

        plan.emergency()
        return pt.Status.SUCCESS


class A_PublishFinalize(pt.behaviour.Behaviour):
    def __init__(self, topic):
        super(A_PublishFinalize, self).__init__(name="A_PublishFinalize")
        self.bb = pt.blackboard.Blackboard()
        self.topic = topic

        self.last_published_time = None

        self.message_object = Empty()


    def setup(self, timeout):
        self.pub = rospy.Publisher(self.topic, Empty, queue_size=1)
        return True


    def update(self):
        if self.last_published_time is not None:
            time_since = time.time() - self.last_published_time
            self.feedback_message = "Last pub'd:{:.2f}s ago".format(time_since)
        else:
            self.feedback_message = "Never published!"

        finalized = self.bb.get(bb_enums.MISSION_FINALIZED)
        if not finalized:
            try:
                self.pub.publish(self.message_object)
                self.last_published_time = time.time()
                self.bb.set(bb_enums.MISSION_FINALIZED, True)
                mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
                mission_plan.complete_mission()
                self.feedback_message = "Mission finalized, plan is go<-False"
                return pt.Status.SUCCESS
            except:
                msg = "Couldn't publish"
                rospy.logwarn_throttle(1, msg)
                self.feedback_message = msg
                return pt.Status.FAILURE

        return pt.Status.SUCCESS


class A_SetNextPlanAction(pt.behaviour.Behaviour):
    def __init__(self, do_not_visit=False):
        """
        Sets the current plan action to the next one
        SUCCESS if it can set it to something that is not None
        FAILURE otherwise

        if do_not_visit=True, then this action will only get the current wp
        and set it and wont actually advance the plan forward.
        This is useful for when you want to set the current wp right after
        you created a plan.
        """
        self.bb = pt.blackboard.Blackboard()
        super(A_SetNextPlanAction, self).__init__('A_SetNextPlanAction')
        self.do_not_visit = do_not_visit

    def update(self):
        mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mission_plan is None:
            rospy.logwarn_throttle(5, "Mission plan was None!")
            return pt.Status.FAILURE

        if not self.do_not_visit:
            #mission_plan.visit_wp()
            mission_plan.complete_maneuver()

        #next_action = mission_plan.get_current_wp("SetNextPlanAction")
        next_action = mission_plan.get_current_maneuver("SetNextPlanAction")
        rospy.logwarn("Next action: " + str(next_action))
        if next_action is None:
            self.feedback_message = "Next action was None"
            rospy.logwarn_throttle(1, self.feedback_message)
            return pt.Status.FAILURE

        rospy.loginfo_throttle_identical(5, "Set CURRENT_PLAN_ACTION {} to: {}".format(self.do_not_visit, str(next_action)))
        self.bb.set(bb_enums.CURRENT_PLAN_ACTION, next_action)
        return pt.Status.SUCCESS


class A_ExecuteManeuver(ptr.actions.ActionClient):

    #################################################################################################
    class WP_ActionClient(ptr.actions.ActionClient):
        def __init__(self, auv_config, vehicle, node_name='wp_actionclient', action_namespace="/lolo/actions"):
            self.node_name = node_name
            self.action_namespace = action_namespace + "/goto_waypoint"
            self.vehicle = vehicle

            # become action client
            ptr.actions.ActionClient.__init__(self,
                name = self.node_name,
                action_spec = GotoWaypointAction,
                action_goal = None,
                action_namespace = self.action_namespace,
                override_feedback_message_on_running = "Moving to waypoint"
            )

            self.action_goal_handle = None

            self.server_feedback_msg = None
            self.action_server_ok = False
            self.goal_tf_frame = auv_config.UTM_LINK

            # every X seconds, try to reconnect to the action server
            # if the server wasnt up and ready when the BT was started
            self.last_reconnect_attempt_time = 0
            self.reconnect_attempt_period = 5

        def setup(self, timeout):
            """
            Overwriting the normal ptr action setup to stop it from failiing the setup step
            and instead handling this failure in the tree.
            """
            self.logger.debug("%s.setup()" % self.__class__.__name__)
            self.action_client = actionlib.SimpleActionClient(
                self.action_namespace,
                self.action_spec
            )

            if not self.action_client.wait_for_server(rospy.Duration(timeout)):
                self.logger.error("{0}.setup() could not connect to the action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
                self.action_client = None
            else:
                self.action_server_ok = True
            return True
        
        def make_goal_from_maneuver(self, maneuver):
            # construct the message
            goal = GotoWaypointGoal()
            goal.waypoint.pose.header
            goal.waypoint.pose.header.frame_id

            goal.waypoint.pose = maneuver.utm_wp
            
            #rospy.loginfo("Goal: " + goal.waypoint.pose.header.frame_id)
            goal.waypoint.goal_tolerance = maneuver.maneuver.wp_goal_tolerance

            goal.waypoint.travel_altitude = maneuver.maneuver.wp_targetAltitude
            goal.waypoint.travel_depth = maneuver.maneuver.wp_targetDepth

            goal.waypoint.speed_control_mode = GotoWaypoint.SPEED_CONTROL_RPM
            goal.waypoint.z_control_mode = GotoWaypoint.Z_CONTROL_DEPTH

            goal.waypoint.travel_rpm = maneuver.maneuver.wp_rpm
            goal.waypoint.speed_control_mode = GotoWaypoint.SPEED_CONTROL_RPM
            goal.waypoint.name = maneuver.name
            return goal
        
        def feedback_cb(self, msg):
            self.server_feedback_msg = msg

        def send_goal(self):
            self.server_feedback_msg = None
            self.action_goal_handle = self.action_client.send_goal(self.action_goal, feedback_cb=self.feedback_cb)
            self.sent_goal = True
            #self.vehicle.last_goto_wp = self.action_goal.waypoint

        def update(self):
            """
            Check only to see whether the underlying action server has
            succeeded, is running, or has cancelled/aborted for some reason and
            map these to the usual behaviour return states.
            """

            if not self.action_server_ok:
                self.feedback_message = "Action Server not available!"
                rospy.logerr_throttle_identical(5, self.feedback_message)
                t = time.time()
                diff = t - self.last_reconnect_attempt_time
                if diff < self.reconnect_attempt_period:
                    self.feedback_message = "Re-trying to connect in {}s".format(diff)
                else:
                    self.setup(self.reconnect_attempt_period-1)

                return pt.Status.FAILURE

            # if your action client is not valid
            if not self.action_client:
                self.feedback_message = "ActionClient is invalid! Client:"+str(self.action_client)
                rospy.logerr(self.feedback_message)
                return pt.Status.FAILURE

            # if the action_goal is invalid
            if not self.action_goal:
                self.feedback_message = "No action_goal (WP)!"
                rospy.logwarn(self.feedback_message)
                return pt.Status.FAILURE

            # if goal hasn't been sent yet
            if not self.sent_goal:
                self.send_goal()
                rospy.loginfo("Sent goal to action server:"+str(self.action_goal))
                self.feedback_message = "Goal sent"
                return pt.Status.RUNNING

            # if the goal was aborted or preempted
            if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                                actionlib_msgs.GoalStatus.PREEMPTED]:
                self.feedback_message = "Aborted goal"
                rospy.loginfo(self.feedback_message)
                self.action_goal = None
                return pt.Status.FAILURE

            result = self.action_client.get_result()

            # if the goal was accomplished
            if result is not None and result.reached_waypoint:
                self.feedback_message = "Completed goal"
                rospy.loginfo(self.feedback_message)
                self.action_goal = None
                return pt.Status.SUCCESS

            # no live updates, just report distance to planned wp
            # still running, set our feedback message to distance left
            #current_loc = self.vehicle.position_utm
            #mplan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
            #if mplan is not None and current_loc is not None:
            #    wp = mplan.get_current_wp()
            #    x,y = current_loc
            #    h_dist = math.sqrt( (x-wp.x)**2 + (y-wp.y)**2 )
            #    v_dist = wp.depth - self.vehicle.depth
            #    self.feedback_message = "HDist:{:.2f}, VDist:{:.2f} towards {}".format(h_dist, v_dist, wp.wp.name)
            self.feedback_message = "Waypoint actionserver client feedback." + str(rospy.Time.now())

            if self.server_feedback_msg is not None and self.server_feedback_msg.feedback_message != "":
                self.feedback_message = "[S:{}]  [C:{}]".format(self.server_feedback_msg.feedback_message, self.feedback_message)

            return pt.Status.RUNNING

        def initialise(self, maneuver = None):
            if(maneuver is None):
                return
            
            self.action_goal = self.make_goal_from_maneuver(maneuver)
            rospy.loginfo("Maneuver goal initialized:"+str(self.action_goal.waypoint.name))

            # ensure that we still need to send the goal
            self.sent_goal = False

#-----------------------------------------------------------------------------------------------#
    class Course_ActionClient(ptr.actions.ActionClient):
        def __init__(self, auv_config, vehicle, node_name='course_actionclient', action_namespace="/lolo/actions"):
            self.node_name = node_name
            self.action_namespace = action_namespace + "/follow_course"
            self.vehicle = vehicle

            # become action client
            ptr.actions.ActionClient.__init__(self,
                name = self.node_name,
                action_spec = FollowCourseAction,
                action_goal = None,
                action_namespace = self.action_namespace,
                override_feedback_message_on_running = "Following course"
            )

            self.action_goal_handle = None

            self.server_feedback_msg = None
            self.action_server_ok = False

            # every X seconds, try to reconnect to the action server
            # if the server wasnt up and ready when the BT was started
            self.last_reconnect_attempt_time = 0
            self.reconnect_attempt_period = 5

        def setup(self, timeout):
            """
            Overwriting the normal ptr action setup to stop it from failiing the setup step
            and instead handling this failure in the tree.
            """
            self.logger.debug("%s.setup()" % self.__class__.__name__)
            self.action_client = actionlib.SimpleActionClient(
                self.action_namespace,
                self.action_spec
            )

            if not self.action_client.wait_for_server(rospy.Duration(timeout)):
                self.logger.error("{0}.setup() could not connect to the action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
                self.action_client = None
            else:
                self.action_server_ok = True
            return True
        
        def make_goal_from_maneuver(self, maneuver):
            
            # construct the message
            goal = FollowCourseGoal()
            goal.targetheading_deg = maneuver.maneuver.course_targetheading
            goal.rpm = maneuver.maneuver.course_rpm
            goal.runtime_s = maneuver.maneuver.course_runtime_s
            goal.targetAltitude = maneuver.maneuver.course_targetAltitude
            goal.targetDepth = maneuver.maneuver.course_targetDepth
            return goal
        
        def feedback_cb(self, msg):
            self.server_feedback_msg = msg

        def send_goal(self):
            self.server_feedback_msg = None
            
            self.action_goal_handle = self.action_client.send_goal(self.action_goal, feedback_cb=self.feedback_cb)
            self.sent_goal = True
            #self.vehicle.last_goto_wp = self.action_goal.waypoint

        def update(self):
            """
            Check only to see whether the underlying action server has
            succeeded, is running, or has cancelled/aborted for some reason and
            map these to the usual behaviour return states.
            """

            if not self.action_server_ok:
                self.feedback_message = "Action Server not available!"
                rospy.logerr_throttle_identical(5, self.feedback_message)
                t = time.time()
                diff = t - self.last_reconnect_attempt_time
                if diff < self.reconnect_attempt_period:
                    self.feedback_message = "Re-trying to connect in {}s".format(diff)
                else:
                    self.setup(self.reconnect_attempt_period-1)

                return pt.Status.FAILURE

            # if your action client is not valid
            if not self.action_client:
                self.feedback_message = "ActionClient is invalid! Client:"+str(self.action_client)
                rospy.logerr(self.feedback_message)
                return pt.Status.FAILURE

            # if the action_goal is invalid
            if not self.action_goal:
                self.feedback_message = "No action_goal! (course)"
                rospy.logwarn(self.feedback_message)
                return pt.Status.FAILURE

            # if goal hasn't been sent yet
            if not self.sent_goal:
                self.send_goal()
                rospy.loginfo("Sent goal to action server:"+str(self.action_goal))
                self.feedback_message = "Goal sent"
                return pt.Status.RUNNING

            # if the goal was aborted or preempted
            if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                                actionlib_msgs.GoalStatus.PREEMPTED]:
                self.feedback_message = "Aborted goal"
                rospy.loginfo(self.feedback_message)
                self.action_goal = None
                return pt.Status.FAILURE

            result = self.action_client.get_result()

            # if the goal was accomplished
            if result is not None and result.done:
                self.feedback_message = "Completed goal"
                rospy.loginfo(self.feedback_message)
                self.action_goal = None
                return pt.Status.SUCCESS

            # no live updates, just report distance to planned wp
            # still running, set our feedback message to distance left
            #current_loc = self.vehicle.position_utm
            #mplan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
            #if mplan is not None and current_loc is not None:
            #    wp = mplan.get_current_wp()
            #    x,y = current_loc
            #    h_dist = math.sqrt( (x-wp.x)**2 + (y-wp.y)**2 )
            #    v_dist = wp.depth - self.vehicle.depth
            #    self.feedback_message = "HDist:{:.2f}, VDist:{:.2f} towards {}".format(h_dist, v_dist, wp.wp.name)
            self.feedback_message = "follow course actionserver client feedback." + str(rospy.Time.now())

            if self.server_feedback_msg is not None and self.server_feedback_msg.feedback_message != "":
                self.feedback_message = "[S:{}]  [C:{}]".format(self.server_feedback_msg.feedback_message, self.feedback_message)

            return pt.Status.RUNNING

        def initialise(self, maneuver = None):
            if(maneuver is None):
                return
            
            self.action_goal = self.make_goal_from_maneuver(maneuver)
            rospy.loginfo("Maneuver goal initialized:")

            # ensure that we still need to send the goal
            self.sent_goal = False
                
    #################################################################################################

    def __init__(self,
                 auv_config,
                 action_namespace = None,
                 node_name = "A_ExecuteManeuver"):
        """
        Runs an action server that will move the robot to the given waypoint

        action_namespace -> if given, will send the goal to that server instead of
        the default goto_waypoint
        wp_from_bb -> if given, the waypoint will be taken from the given bb variable
        live_mode_enabled -> if True, the waypoint will be re-submitted every tick to the server, wp_from_bb must be given
        goalless -> if True, only an empty goal will be sent to the sever, useful as a "signal to start"
        """

        super(A_ExecuteManeuver, self).__init__(name="A_ExcecuteManeuver")

        self.bb = pt.blackboard.Blackboard()
        self.vehicle = self.bb.get(bb_enums.VEHICLE_STATE)
        self.node_name = node_name

        
        #Action namespaces
        action_namespace = auv_config.GOTO_ACTION_NAMESPACE


        #Create Action clients
        self.wp_actionclient = self.WP_ActionClient(auv_config=auv_config, vehicle=self.vehicle, node_name="wp_actionclient", action_namespace=action_namespace)
        self.course_actionclient = self.Course_ActionClient(auv_config=auv_config, vehicle=self.vehicle, node_name="course_actionclient", action_namespace=action_namespace)


    def setup(self, timeout):
        r1 = self.wp_actionclient.setup(timeout)
        r2 = self.course_actionclient.setup(timeout)
        return r1 or r2

    def initialise(self):
        if not self.wp_actionclient.action_server_ok:
            self.feedback_message = "No WP action server found for {}!".format(self.action_namespace)
            rospy.logwarn_throttle(5, self.feedback_message)
            return

        if not self.course_actionclient.action_server_ok:
            self.feedback_message = "No Course action server found for {}!".format(self.action_namespace)
            rospy.logwarn_throttle(5, self.feedback_message)
            return

        mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        
        if mission_plan is None:
            self.feedback_message = "No mission plan found!"
            rospy.logwarn(self.feedback_message)
            return

        #Get current maneuver
        #maneuver = mission_plan.get_current_wp("GotoWaypointAction")
        maneuver = mission_plan.get_current_maneuver("GotoWaypointAction")

        if maneuver is None:
            self.feedback_message = "Unplanned waypoint not found but it is enabled!"
            rospy.loginfo_throttle(3, self.feedback_message)
            return

        #if maneuver.frame_id != self.goal_tf_frame:
        #    self.feedback_message = 'The frame of the waypoint({0}) does not match the expected frame({1}) of the action client!'.format(wp.frame_id, self.goal_tf_frame)
        #    rospy.logerr_throttle(5, self.feedback_message)
        #    return
        
        if(maneuver.maneuver.maneuver_type == Maneuver.MANEUVER_TYPE_WP):
            #Waypoint
            self.wp_actionclient.initialise(maneuver)
            self.course_actionclient.terminate(pt.common.Status.INVALID)
        elif (maneuver.maneuver.maneuver_type == Maneuver.MANEUVER_TYPE_COURSE):
            #Coscos
            self.course_actionclient.initialise(maneuver)
            self.wp_actionclient.terminate(pt.common.Status.INVALID)
        else:
            rospy.logerr("This is not right!")        

    def update(self):
        """
        Check only to see whether the underlying action server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.
        """ 
        #return self.course_actionclient.update() 
        r1 = self.wp_actionclient.update() 
        r2 = self.course_actionclient.update()

        if r1  == pt.Status.SUCCESS or r2 == pt.Status.SUCCESS:
            return pt.Status.SUCCESS
        if r1  == pt.Status.RUNNING or r2 == pt.Status.RUNNING:
            return pt.Status.RUNNING
        
        rospy.logerr("WP or course action client failed. Calling stop mission")
        mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        mission_plan.stop_mission()
        
        return pt.Status.FAILURE
    
    def terminate(self, new_status):
        self.wp_actionclient.terminate(new_status)
        self.wp_actionclient.action_goal = None
        self.course_actionclient.terminate(new_status)
        self.course_actionclient.action_goal = None

class A_GotoWaypoint(ptr.actions.ActionClient):
    def __init__(self,
                 auv_config,
                 action_namespace = None,
                 node_name = "A_GotoWaypoint"):
        """
        Runs an action server that will move the robot to the given waypoint

        action_namespace -> if given, will send the goal to that server instead of
        the default goto_waypoint
        """

        self.bb = pt.blackboard.Blackboard()
        self.vehicle = self.bb.get(bb_enums.VEHICLE_STATE)
        self.node_name = node_name


        self.action_goal_handle = None

        if action_namespace is None:
            action_namespace = auv_config.GOTO_ACTION_NAMESPACE

        action_namespace = action_namespace + "/goto_waypoint"

        # become action client
        ptr.actions.ActionClient.__init__(
            self,
            name = self.node_name,
            action_spec = GotoWaypointAction,
            action_goal = None,
            action_namespace = action_namespace,
            override_feedback_message_on_running = "Moving to waypoint"
        )
        self.server_feedback_msg = None

        self.action_server_ok = False

        self.goal_tf_frame = auv_config.UTM_LINK

        # every X seconds, try to reconnect to the action server
        # if the server wasnt up and ready when the BT was started
        self.last_reconnect_attempt_time = 0
        self.reconnect_attempt_period = 5

    def setup(self, timeout):
        """
        Overwriting the normal ptr action setup to stop it from failiing the setup step
        and instead handling this failure in the tree.
        """
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.action_client = actionlib.SimpleActionClient(
            self.action_namespace,
            self.action_spec
        )

        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
            self.action_client = None
        else:
            self.action_server_ok = True

        return True

    def make_goal_from_maneuver(self, maneuver):
        # construct the message
        goal = GotoWaypointGoal()
        goal.waypoint.pose.header
        goal.waypoint.pose.header.frame_id

        goal.waypoint.pose = maneuver.utm_wp
        
        #rospy.loginfo("Goal: " + goal.waypoint.pose.header.frame_id)
        goal.waypoint.goal_tolerance = maneuver.maneuver.wp_goal_tolerance

        goal.waypoint.travel_altitude = maneuver.maneuver.wp_targetAltitude
        goal.waypoint.travel_depth = maneuver.maneuver.wp_targetDepth

        goal.waypoint.speed_control_mode = GotoWaypoint.SPEED_CONTROL_RPM
        goal.waypoint.z_control_mode = GotoWaypoint.Z_CONTROL_DEPTH

        goal.waypoint.travel_rpm = maneuver.maneuver.wp_rpm
        goal.waypoint.speed_control_mode = GotoWaypoint.SPEED_CONTROL_RPM
        goal.waypoint.name = maneuver.name
        return goal

    def feedback_cb(self, msg):
        self.server_feedback_msg = msg

    def send_goal(self):
        self.server_feedback_msg = None
        self.action_goal_handle = self.action_client.send_goal(self.action_goal, feedback_cb=self.feedback_cb)
        self.sent_goal = True
        #self.vehicle.last_goto_wp = self.action_goal.waypoint

    def initialise(self):
        if not self.action_server_ok:
            self.feedback_message = "No action server found for {}!".format(self.action_namespace)
            rospy.logwarn_throttle(5, self.feedback_message)
            return

 
        mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mission_plan is None:
            self.feedback_message = "No mission plan found!"
            rospy.logwarn(self.feedback_message)
            return

        #Get current maneuver
        maneuver = mission_plan.get_current_maneuver("GotoWaypointAction")

        if maneuver is None:
            self.feedback_message = "Unplanned waypoint not found but it is enabled!"
            rospy.loginfo_throttle(3, self.feedback_message)
            return

        #if wp.frame_id != self.goal_tf_frame:
        #    self.feedback_message = 'The frame of the waypoint({0}) does not match the expected frame({1}) of the action client!'.format(wp.frame_id, self.goal_tf_frame)
        #    rospy.logerr_throttle(5, self.feedback_message)
        #    return


        self.action_goal = self.make_goal_from_maneuver(maneuver)
        rospy.loginfo("Goto waypoint goal initialized:"+str(self.action_goal.waypoint.name))
        # ensure that we still need to send the goal
        self.sent_goal = False

    def update(self):
        """
        Check only to see whether the underlying action server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.
        """

        if not self.action_server_ok:
            self.feedback_message = "Action Server not available!"
            rospy.logerr_throttle_identical(5, self.feedback_message)
            t = time.time()
            diff = t - self.last_reconnect_attempt_time
            if diff < self.reconnect_attempt_period:
                self.feedback_message = "Re-trying to connect in {}s".format(diff)
            else:
                self.setup(self.reconnect_attempt_period-1)

            return pt.Status.FAILURE

        # if your action client is not valid
        if not self.action_client:
            self.feedback_message = "ActionClient is invalid! Client:"+str(self.action_client)
            rospy.logerr(self.feedback_message)
            return pt.Status.FAILURE

        # if the action_goal is invalid
        if not self.action_goal:
            self.feedback_message = "No action_goal!"
            rospy.logwarn(self.feedback_message)
            return pt.Status.FAILURE

        # if goal hasn't been sent yet
        if not self.sent_goal:
            self.send_goal()
            rospy.loginfo("Sent goal to action server:"+str(self.action_goal))
            self.feedback_message = "Goal sent"
            return pt.Status.RUNNING

        # if the goal was aborted or preempted
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                              actionlib_msgs.GoalStatus.PREEMPTED]:
            self.feedback_message = "Aborted goal"
            rospy.loginfo(self.feedback_message)
            return pt.Status.FAILURE

        result = self.action_client.get_result()

        # if the goal was accomplished
        if result is not None and result.reached_waypoint:
            self.feedback_message = "Completed goal"
            rospy.loginfo(self.feedback_message)
            return pt.Status.SUCCESS
        
        self.feedback_message = "Waypoint actionserver client feedback." + str(rospy.Time.now())

        if self.server_feedback_msg is not None and self.server_feedback_msg.feedback_message != "":
            self.feedback_message = "[S:{}]  [C:{}]".format(self.server_feedback_msg.feedback_message, self.feedback_message)

        return pt.Status.RUNNING

class A_Followcourse(ptr.actions.ActionClient):
    def __init__(self,
                 auv_config,
                 action_namespace = None,
                 node_name = "A_Followcourse"):
        """
        Runs an action server that will move the robot to the given waypoint

        action_namespace -> if given, will send the goal to that server instead of
        the default goto_waypoint
        """

        self.bb = pt.blackboard.Blackboard()
        self.vehicle = self.bb.get(bb_enums.VEHICLE_STATE)
        self.node_name = node_name
        
        if action_namespace is None:
            action_namespace = auv_config.GOTO_ACTION_NAMESPACE
        action_namespace = action_namespace + "/follow_course"

        # become action client
        ptr.actions.ActionClient.__init__(self,
            name = self.node_name,
            action_spec = FollowCourseAction,
            action_goal = None,
            action_namespace = action_namespace,
            override_feedback_message_on_running = "Following course"
        )

        self.action_goal_handle = None

        self.server_feedback_msg = None
        self.action_server_ok = False

        # every X seconds, try to reconnect to the action server
        # if the server wasnt up and ready when the BT was started
        self.last_reconnect_attempt_time = 0
        self.reconnect_attempt_period = 5


    def setup(self, timeout):
        """
        Overwriting the normal ptr action setup to stop it from failiing the setup step
        and instead handling this failure in the tree.
        """
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.action_client = actionlib.SimpleActionClient(
            self.action_namespace,
            self.action_spec
        )

        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
            self.action_client = None
        else:
            self.action_server_ok = True
        return True

    def make_goal_from_maneuver(self, maneuver):
            # construct the message
        goal = FollowCourseGoal()
        goal.targetheading_deg = maneuver.maneuver.course_targetheading
        goal.rpm = maneuver.maneuver.course_rpm
        goal.runtime_s = maneuver.maneuver.course_runtime_s
        goal.targetAltitude = maneuver.maneuver.course_targetAltitude
        goal.targetDepth = maneuver.maneuver.course_targetDepth
        return goal

    def feedback_cb(self, msg):
        self.server_feedback_msg = msg

    def send_goal(self):
        self.server_feedback_msg = None
        self.action_goal_handle = self.action_client.send_goal(self.action_goal, feedback_cb=self.feedback_cb)
        self.sent_goal = True
        #self.vehicle.last_goto_wp = self.action_goal.waypoint

    def initialise(self):
        if not self.action_server_ok:
            self.feedback_message = "No action server found for {}!".format(self.action_namespace)
            rospy.logwarn_throttle(5, self.feedback_message)
            return

        mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mission_plan is None:
            self.feedback_message = "No mission plan found!"
            rospy.logwarn(self.feedback_message)
            return

        #Get current maneuver
        maneuver = mission_plan.get_current_maneuver("GotoWaypointAction")

        if(maneuver is None):
            return
        
        self.action_goal = self.make_goal_from_maneuver(maneuver)
        rospy.loginfo("Maneuver goal initialized:")

        # ensure that we still need to send the goal
        self.sent_goal = False

    def update(self):
        """
        Check only to see whether the underlying action server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.
        """

        if not self.action_server_ok:
            self.feedback_message = "Action Server not available!"
            rospy.logerr_throttle_identical(5, self.feedback_message)
            t = time.time()
            diff = t - self.last_reconnect_attempt_time
            if diff < self.reconnect_attempt_period:
                self.feedback_message = "Re-trying to connect in {}s".format(diff)
            else:
                self.setup(self.reconnect_attempt_period-1)

            return pt.Status.FAILURE

        # if your action client is not valid
        if not self.action_client:
            self.feedback_message = "ActionClient is invalid! Client:"+str(self.action_client)
            rospy.logerr(self.feedback_message)
            return pt.Status.FAILURE

        # if the action_goal is invalid
        if not self.action_goal:
            self.feedback_message = "No action_goal! (course)"
            rospy.logwarn(self.feedback_message)
            return pt.Status.FAILURE

        # if goal hasn't been sent yet
        if not self.sent_goal:
            self.send_goal()
            rospy.loginfo("Sent goal to action server:"+str(self.action_goal))
            self.feedback_message = "Goal sent"
            return pt.Status.RUNNING

        # if the goal was aborted or preempted
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                            actionlib_msgs.GoalStatus.PREEMPTED]:
            self.feedback_message = "Aborted goal"
            rospy.loginfo(self.feedback_message)
            self.action_goal = None
            return pt.Status.FAILURE

        result = self.action_client.get_result()

        # if the goal was accomplished
        if result is not None and result.done:
            self.feedback_message = "Completed goal"
            rospy.loginfo(self.feedback_message)
            self.action_goal = None
            return pt.Status.SUCCESS

        # no live updates, just report distance to planned wp
        # still running, set our feedback message to distance left
        #current_loc = self.vehicle.position_utm
        #mplan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        #if mplan is not None and current_loc is not None:
        #    wp = mplan.get_current_wp()
        #    x,y = current_loc
        #    h_dist = math.sqrt( (x-wp.x)**2 + (y-wp.y)**2 )
        #    v_dist = wp.depth - self.vehicle.depth
        #    self.feedback_message = "HDist:{:.2f}, VDist:{:.2f} towards {}".format(h_dist, v_dist, wp.wp.name)
        self.feedback_message = "follow course actionserver client feedback." + str(rospy.Time.now())

        if self.server_feedback_msg is not None and self.server_feedback_msg.feedback_message != "":
            self.feedback_message = "[S:{}]  [C:{}]".format(self.server_feedback_msg.feedback_message, self.feedback_message)

        return pt.Status.RUNNING
    
## New action servers and related actions

class A_SetBBVariable_True(pt.behaviour.Behaviour):
    def __init__(self, key):
        super(A_SetBBVariable_True, self).__init__(name="A_SetBBVariable_True")
        self.bb = pt.blackboard.Blackboard()
        self.bb_key = key
        self.feedback_message = "SetBBVariable :" + str(self.bb_key) + " -> True"

    def update(self):
        self.bb.set(self.bb_key, True)
        return pt.Status.SUCCESS

class A_SetBBVariable_False(pt.behaviour.Behaviour):
    def __init__(self, key):
        super(A_SetBBVariable_False, self).__init__(name="A_SetBBVariable_False")
        self.bb = pt.blackboard.Blackboard()
        self.bb_key = key
        self.feedback_message = "SetBBVariable :" + str(self.bb_key) + " -> False"

    def update(self):
        self.bb.set(self.bb_key, False)
        return pt.Status.SUCCESS

class A_updateAvoidance_maneuver(pt.behaviour.Behaviour):
    def __init__(self, key):
        self.bb = pt.blackboard.Blackboard()
        super(A_updateAvoidance_maneuver, self).__init__(name="A_updateAvoidance_maneuver")
        self.bb = pt.blackboard.Blackboard()
        self.vehicle = self.bb.get(bb_enums.VEHICLE_STATE)
        self.bb_maneuver_key = key
        self.max_wp_in_list = 100 #max allowed wps in list. When reached the oldest wp will be removed
        self.update_distance = 10 # Only add a new wp to the list if we are Xm away from the last recorded position
        self.wp_distance = 100
        self.list_of_wp = []
    
    def _set_maneuver(self, utm_pos):
        maneuver = Maneuver()
        maneuver.name = str("Avoidance wp")
        maneuver.vehicle_mode = 0
        maneuver.wp_goal_tolerance = 20
        maneuver.wp_rpm = 700
        maneuver.wp_targetDepth = 0
        maneuver.wp_targetAltitude = 200
        maneuver.maneuver_type = Maneuver.MANEUVER_TYPE_WP

        mpm = Mission_Plan_Maneuver(maneuver)
        mpm.utm_wp = PoseStamped()
        mpm.utm_wp.pose.position.x = utm_pos[0]
        mpm.utm_wp.pose.position.y = utm_pos[1]
        mpm.utm_wp.header.frame_id = 'utm'

        self.bb.set(self.bb_maneuver_key, mpm)
        self.feedback_message = "Avoid maneuver set to X:{}, Y:{}, Depth:{}]".format(str(mpm.utm_wp.pose.position.x), str(mpm.utm_wp.pose.position.y), str(maneuver.wp_targetDepth))

    def update(self):

        lolo_position = self.vehicle.position_utm

        if(lolo_position == None): return pt.Status.FAILURE
        
        #No avoid wps in list? Add current pos, and set current pos a avoid WP.
        if(len(self.list_of_wp) == 0):
            self.list_of_wp.append(lolo_position)
            self._set_maneuver(lolo_position)
            rospy.loginfo("Updated emergency WP")
            return pt.Status.SUCCESS

        #Check if our current position is more than Xm from the last recorded position
        last_pos = np.array(self.list_of_wp[-1])
        dist = np.linalg.norm(np.array(lolo_position) - last_pos)
        rospy.loginfo_throttle(10,"Distance from last saved position: " + str(dist))

        if(dist > self.update_distance and len(self.list_of_wp) < self.max_wp_in_list):
            self.list_of_wp.append(lolo_position)

        #Check if we are more than X meters away from the first wp in the list
        # and update the avoidance wp if that is the case
        dist = np.linalg.norm(np.array(lolo_position) - np.array(self.list_of_wp[0]))
        if(dist >= self.wp_distance):
            self._set_maneuver(self.list_of_wp[0])

            #Delete the wp from the list
            del self.list_of_wp[0]

        #Always success!
        return pt.Status.SUCCESS

class A_BBManeuver_depth_to_current(pt.behaviour.Behaviour):
    def __init__(self, key):
        self.bb = pt.blackboard.Blackboard()
        super(A_BBManeuver_depth_to_current, self).__init__(name="A_BBManeuver_depth_to_current")
        self.bb = pt.blackboard.Blackboard()
        self.vehicle = self.bb.get(bb_enums.VEHICLE_STATE)
        self.bb_maneuver_key = key

    def update(self):

        lolo_depth = self.vehicle.depth
        
        maneuver = self.bb.get(self.bb_maneuver_key)

        maneuver.maneuver.wp_targetDepth = lolo_depth
        maneuver.maneuver.course_targetDepth = lolo_depth

        self.bb.set(self.bb_maneuver_key, maneuver)

        rospy.loginfo_throttle(10, "Updated depth setpoint of Missionplan WP")

        #Always success!
        return pt.Status.SUCCESS


class A_setBBManeuverFromPlan(pt.behaviour.Behaviour):
    def __init__(self, key):
        self.bb = pt.blackboard.Blackboard()
        super(A_setBBManeuverFromPlan, self).__init__(name="A_setBBManeuverFromPlan")
        self.bb = pt.blackboard.Blackboard()
        self.vehicle = self.bb.get(bb_enums.VEHICLE_STATE)
        self.bb_maneuver_key = key

    def update(self):
        mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mission_plan is None:
            self.feedback_message = "No mission plan found!"
            rospy.logwarn(self.feedback_message)
            pt.Status.FAILURE

        #Get current maneuver
        maneuver = mission_plan.get_current_maneuver("A_setBBManeuverFromPlanAction")

        if maneuver is None:
            self.feedback_message = "No manuver found in mssion plan"
            rospy.loginfo_throttle(3, self.feedback_message)
            pt.Status.FAILURE
        
        self.bb.set(self.bb_maneuver_key, maneuver)
        return pt.Status.SUCCESS
        


class A_GotoWaypoint_BB(ptr.actions.ActionClient):
    def __init__(self,
                 auv_config,
                 key,
                 action_namespace = None,
                 node_name = "A_GotoWaypoint_BB"):
        """
        Runs an action server that will move the robot to the given waypoint.
        The waypoint is taken from the blackboard using the key parameter

        action_namespace -> if given, will send the goal to that server instead of
        the default goto_waypoint
        """

        self.bb = pt.blackboard.Blackboard()
        self.vehicle = self.bb.get(bb_enums.VEHICLE_STATE)
        self.node_name = node_name
        self.bb_maneuver_key = key


        self.action_goal_handle = None

        if action_namespace is None:
            action_namespace = auv_config.GOTO_ACTION_NAMESPACE
            action_namespace = action_namespace + "/goto_waypoint"

        # become action client
        ptr.actions.ActionClient.__init__(
            self,
            name = self.node_name,
            action_spec = GotoWaypointAction,
            action_goal = None,
            action_namespace = action_namespace,
            override_feedback_message_on_running = "Moving to waypoint"
        )
        self.server_feedback_msg = None

        self.action_server_ok = False

        self.goal_tf_frame = auv_config.UTM_LINK

        # every X seconds, try to reconnect to the action server
        # if the server wasnt up and ready when the BT was started
        self.last_reconnect_attempt_time = 0
        self.reconnect_attempt_period = 5

    def setup(self, timeout):
        """
        Overwriting the normal ptr action setup to stop it from failiing the setup step
        and instead handling this failure in the tree.
        """
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.action_client = actionlib.SimpleActionClient(
            self.action_namespace,
            self.action_spec
        )

        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
            self.action_client = None
        else:
            self.action_server_ok = True

        return True

    def make_goal_from_maneuver(self, maneuver):
        # construct the message
        goal = GotoWaypointGoal()
        goal.waypoint.pose.header
        goal.waypoint.pose.header.frame_id

        goal.waypoint.pose = maneuver.utm_wp
        
        #rospy.loginfo("Goal: " + goal.waypoint.pose.header.frame_id)
        goal.waypoint.goal_tolerance = maneuver.maneuver.wp_goal_tolerance

        goal.waypoint.travel_altitude = maneuver.maneuver.wp_targetAltitude
        goal.waypoint.travel_depth = maneuver.maneuver.wp_targetDepth

        goal.waypoint.speed_control_mode = GotoWaypoint.SPEED_CONTROL_RPM
        goal.waypoint.z_control_mode = GotoWaypoint.Z_CONTROL_DEPTH

        goal.waypoint.travel_rpm = maneuver.maneuver.wp_rpm
        goal.waypoint.speed_control_mode = GotoWaypoint.SPEED_CONTROL_RPM
        goal.waypoint.name = maneuver.name
        return goal

    def feedback_cb(self, msg):
        self.server_feedback_msg = msg

    def send_goal(self):
        self.server_feedback_msg = None
        self.action_goal_handle = self.action_client.send_goal(self.action_goal, feedback_cb=self.feedback_cb)
        self.sent_goal = True
        #self.vehicle.last_goto_wp = self.action_goal.waypoint

    def initialise(self):
        if not self.action_server_ok:
            self.feedback_message = "No action server found for {}!".format(self.action_namespace)
            rospy.logwarn_throttle(5, self.feedback_message)
            return

        #Get maneuver from BB
        maneuver = self.bb.get(self.bb_maneuver_key) #mission_plan.get_current_maneuver("GotoWaypointAction")

        if maneuver is None:
            self.feedback_message = "No maneuver in blackboard!"
            rospy.loginfo_throttle(3, self.feedback_message)
            return

        #if wp.frame_id != self.goal_tf_frame:
        #    self.feedback_message = 'The frame of the waypoint({0}) does not match the expected frame({1}) of the action client!'.format(wp.frame_id, self.goal_tf_frame)
        #    rospy.logerr_throttle(5, self.feedback_message)
        #    return


        self.action_goal = self.make_goal_from_maneuver(maneuver)
        rospy.loginfo("Goto waypoint goal initialized:"+str(self.action_goal.waypoint.name))
        # ensure that we still need to send the goal
        self.sent_goal = False

    def update(self):
        """
        Check only to see whether the underlying action server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.
        """

        if not self.action_server_ok:
            self.feedback_message = "Action Server not available!"
            rospy.logerr_throttle_identical(5, self.feedback_message)
            t = time.time()
            diff = t - self.last_reconnect_attempt_time
            if diff < self.reconnect_attempt_period:
                self.feedback_message = "Re-trying to connect in {}s".format(diff)
            else:
                self.setup(self.reconnect_attempt_period-1)

            return pt.Status.FAILURE

        # if your action client is not valid
        if not self.action_client:
            self.feedback_message = "ActionClient is invalid! Client:"+str(self.action_client)
            rospy.logerr(self.feedback_message)
            return pt.Status.FAILURE

        # if the action_goal is invalid
        if not self.action_goal:
            self.feedback_message = "No action_goal!"
            rospy.logwarn(self.feedback_message)
            return pt.Status.FAILURE

        # if goal hasn't been sent yet
        if not self.sent_goal:
            self.send_goal()
            rospy.loginfo("Sent goal to action server:"+str(self.action_goal))
            self.feedback_message = "Goal sent"
            return pt.Status.RUNNING

        # if the goal was aborted or preempted
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                              actionlib_msgs.GoalStatus.PREEMPTED]:
            self.feedback_message = "Aborted goal"
            rospy.loginfo(self.feedback_message)
            return pt.Status.FAILURE

        result = self.action_client.get_result()

        # if the goal was accomplished
        if result is not None and result.reached_waypoint:
            self.feedback_message = "Completed goal"
            rospy.loginfo(self.feedback_message)
            return pt.Status.SUCCESS
        
        self.feedback_message = "Waypoint actionserver client feedback." + str(rospy.Time.now())

        if self.server_feedback_msg is not None and self.server_feedback_msg.feedback_message != "":
            self.feedback_message = "[S:{}]  [C:{}]".format(self.server_feedback_msg.feedback_message, self.feedback_message)

        return pt.Status.RUNNING

class A_Followcourse_BB(ptr.actions.ActionClient):
    def __init__(self,
                 auv_config,
                 key,
                 action_namespace = None,
                 node_name = "A_Followcourse_BB"):
        """
        Runs an action server that will move the robot to the given waypoint

        action_namespace -> if given, will send the goal to that server instead of
        the default goto_waypoint
        """

        self.bb = pt.blackboard.Blackboard()
        self.vehicle = self.bb.get(bb_enums.VEHICLE_STATE)
        self.node_name = node_name
        self.bb_maneuver_key = key
        
        if action_namespace is None:
            action_namespace = auv_config.GOTO_ACTION_NAMESPACE
        action_namespace = action_namespace + "/follow_course"

        # become action client
        ptr.actions.ActionClient.__init__(self,
            name = self.node_name,
            action_spec = FollowCourseAction,
            action_goal = None,
            action_namespace = action_namespace,
            override_feedback_message_on_running = "Following course"
        )

        self.action_goal_handle = None

        self.server_feedback_msg = None
        self.action_server_ok = False

        # every X seconds, try to reconnect to the action server
        # if the server wasnt up and ready when the BT was started
        self.last_reconnect_attempt_time = 0
        self.reconnect_attempt_period = 5


    def setup(self, timeout):
        """
        Overwriting the normal ptr action setup to stop it from failiing the setup step
        and instead handling this failure in the tree.
        """
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.action_client = actionlib.SimpleActionClient(
            self.action_namespace,
            self.action_spec
        )

        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
            self.action_client = None
        else:
            self.action_server_ok = True
        return True

    def make_goal_from_maneuver(self, maneuver):
            # construct the message
        goal = FollowCourseGoal()
        goal.targetheading_deg = maneuver.maneuver.course_targetheading
        goal.rpm = maneuver.maneuver.course_rpm
        goal.runtime_s = maneuver.maneuver.course_runtime_s
        goal.targetAltitude = maneuver.maneuver.course_targetAltitude
        goal.targetDepth = maneuver.maneuver.course_targetDepth
        return goal

    def feedback_cb(self, msg):
        self.server_feedback_msg = msg

    def send_goal(self):
        self.server_feedback_msg = None
        self.action_goal_handle = self.action_client.send_goal(self.action_goal, feedback_cb=self.feedback_cb)
        self.sent_goal = True
        #self.vehicle.last_goto_wp = self.action_goal.waypoint

    def initialise(self):
        if not self.action_server_ok:
            self.feedback_message = "No action server found for {}!".format(self.action_namespace)
            rospy.logwarn_throttle(5, self.feedback_message)
            return

        #Get maneuver from BB
        maneuver = self.bb.get(self.bb_maneuver_key)

        if(maneuver is None):
            return
        
        self.action_goal = self.make_goal_from_maneuver(maneuver)
        rospy.loginfo("Maneuver goal initialized:")

        # ensure that we still need to send the goal
        self.sent_goal = False

    def update(self):
        """
        Check only to see whether the underlying action server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.
        """

        if not self.action_server_ok:
            self.feedback_message = "Action Server not available!"
            rospy.logerr_throttle_identical(5, self.feedback_message)
            t = time.time()
            diff = t - self.last_reconnect_attempt_time
            if diff < self.reconnect_attempt_period:
                self.feedback_message = "Re-trying to connect in {}s".format(diff)
            else:
                self.setup(self.reconnect_attempt_period-1)

            return pt.Status.FAILURE

        # if your action client is not valid
        if not self.action_client:
            self.feedback_message = "ActionClient is invalid! Client:"+str(self.action_client)
            rospy.logerr(self.feedback_message)
            return pt.Status.FAILURE

        # if the action_goal is invalid
        if not self.action_goal:
            self.feedback_message = "No action_goal! (course)"
            rospy.logwarn(self.feedback_message)
            return pt.Status.FAILURE

        # if goal hasn't been sent yet
        if not self.sent_goal:
            self.send_goal()
            rospy.loginfo("Sent goal to action server:"+str(self.action_goal))
            self.feedback_message = "Goal sent"
            return pt.Status.RUNNING

        # if the goal was aborted or preempted
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                            actionlib_msgs.GoalStatus.PREEMPTED]:
            self.feedback_message = "Aborted goal"
            rospy.loginfo(self.feedback_message)
            self.action_goal = None
            return pt.Status.FAILURE

        result = self.action_client.get_result()

        # if the goal was accomplished
        if result is not None and result.done:
            self.feedback_message = "Completed goal"
            rospy.loginfo(self.feedback_message)
            self.action_goal = None
            return pt.Status.SUCCESS

        # no live updates, just report distance to planned wp
        # still running, set our feedback message to distance left
        #current_loc = self.vehicle.position_utm
        #mplan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        #if mplan is not None and current_loc is not None:
        #    wp = mplan.get_current_wp()
        #    x,y = current_loc
        #    h_dist = math.sqrt( (x-wp.x)**2 + (y-wp.y)**2 )
        #    v_dist = wp.depth - self.vehicle.depth
        #    self.feedback_message = "HDist:{:.2f}, VDist:{:.2f} towards {}".format(h_dist, v_dist, wp.wp.name)
        self.feedback_message = "follow course actionserver client feedback." + str(rospy.Time.now())

        if self.server_feedback_msg is not None and self.server_feedback_msg.feedback_message != "":
            self.feedback_message = "[S:{}]  [C:{}]".format(self.server_feedback_msg.feedback_message, self.feedback_message)

        return pt.Status.RUNNING
