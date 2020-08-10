#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

import rospy
#Backward compatebility with ros kinetic
if not hasattr(rospy, 'loginfo_throttle_identical'): setattr(rospy, 'loginfo_throttle_identical', rospy.loginfo_throttle)
if not hasattr(rospy, 'logwarn_throttle_identical'): setattr(rospy, 'logwarn_throttle_identical', rospy.logwarn_throttle)
if not hasattr(rospy, 'logwarn_once'): setattr(rospy, 'logwarn_once', rospy.logwarn)
if not hasattr(rospy, 'logerr_throttle_identical'):  setattr(rospy, 'logerr_throttle_identical', rospy.logerr_throttle)

import py_trees as pt
import py_trees_ros as ptr

# just convenience really
from py_trees.composites import Selector as Fallback

# messages
from std_msgs.msg import Float64, Empty
from sam_msgs.msg import Leak
from cola2_msgs.msg import DVL
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix
from imc_ros_bridge.msg import EstimatedState
from geodesy.utm import fromLatLong, UTMPoint
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math

from auv_config import AUVConfig

# tree leaves
from bt_actions import A_GotoWaypoint, \
                       A_SetNextPlanAction, \
                       A_UpdateTF, \
                       A_EmergencySurface, \
                       A_EmergencySurfaceByForce, \
                       A_SetUTMFromGPS, \
                       A_UpdateLatLon, \
                       A_RefineMission, \
                       A_UpdateNeptusEstimatedState, \
                       A_UpdateNeptusPlanControlState, \
                       A_UpdateNeptusVehicleState, \
                       A_UpdateNeptusPlanDB, \
                       A_UpdateNeptusPlanControl, \
                       A_UpdateMissonForPOI, \
                       A_VizPublishPlan, \
                       A_FollowLeader, \
                       A_SetDVLRunning


from bt_conditions import C_PlanCompleted, \
                          C_NoAbortReceived, \
                          C_DepthOK, \
                          C_AltOK, \
                          C_LeakOK, \
                          C_StartPlanReceived, \
                          C_HaveRefinedMission, \
                          C_HaveCoarseMission, \
                          C_PlanIsNotChanged, \
                          C_NoNewPOIDetected, \
                          C_AutonomyDisabled, \
                          C_LeaderFollowerEnabled, \
                          C_LeaderExists, \
                          C_LeaderIsFarEnough, \
                          C_AtDVLDepth

from bt_common import Sequence, \
                      CheckBlackboardVariableValue, \
                      ReadTopic, \
                      A_RunOnce, \
                      Counter


# globally defined values
import bb_enums
import imc_enums
import common_globals

from vane_interface.msg import vane_status

################################# Vane config #################################
#Add variables for storing the vane_status message
setattr(bb_enums, "VANE_STATUS" , 'vane_status') 
 
class ASVConfig(AUVConfig):
    def __init__(self):
        AUVConfig.__init__(self)
        self.robot_name = "vane"
        self.STATUS_TOPIC = "status/general"
        self.GPS_FIX_TOPIC = 'core/gps/ais'
        self.ACTION_NAMESPACE = 'ctrl/wp_action_planner'
        self.LOCAL_LINK = 'utm'

    #Override function for writing a launch file
    def generate_launch_file(self, catkin_ws_path):
        def make_arg(name, default):
            return '\t<arg name="{}" default="{}" />\n'.format(name.lower(), default)

        def make_param(name):
            return '\t\t<param name="{}" value="$(arg {})" />\n'.format(name.lower(), name.lower())

        args_part = ''
        params_part ='\n\n\t<node name="vane_bt" pkg="smarc_bt" type="vane_bt.py" output="screen" ns="$(arg robot_name)">\n'
        for k,v in vars(self).iteritems():
            args_part += make_arg(k,v)
            params_part += make_param(k)

        params_part += '\t</node>\n'

        bt_launch_path = '/src/smarc_missions/smarc_bt/launch/bt_vane.launch'
        with open(catkin_ws_path+bt_launch_path, 'w+') as f:
            f.write('<!-- THIS LAUNCH FILE WAS AUTO-GENERATED FROM src/auv_config.py -->\n\n')
            f.write('<launch>\n')
            f.write(args_part.replace("sam", "vane"))
            f.write(params_part.replace("sam", "vane"))
            f.write('</launch>\n')
        print("Generated default launch file at {}".format(catkin_ws_path+bt_launch_path))
        print("You might need to restart the ros mon instance to read the new launch file")


################################# Vane actions #################################

class A_SendHeartBeat(pt.behaviour.Behaviour):
    """
    Sends heartbeat to vane to vane knows that the scientist is alive
    """
    def __init__(self):
        super(A_SendHeartBeat, self).__init__("A_SendHeartBeat")
        self.bb = pt.blackboard.Blackboard()
        self.heartbeat_topic = "/vane/Heartbeat"

    def setup(self, timeout):
        self.heartbeat_pub = rospy.Publisher(self.heartbeat_topic, Empty, queue_size=1)
        return True

    def update(self):
        #rospy.loginfo("Sending heartbeat to vane")
        self.heartbeat_pub.publish(Empty()) #Send heartbeat
        return pt.Status.SUCCESS

class A_UpdateLatLon_Vane(A_UpdateLatLon):
    """
    We use the lat/lon provoded by the captain
    """
    def __init__(self):
        A_UpdateLatLon.__init__(self)

    def update(self):
        status = self.bb.get(bb_enums.VANE_STATUS)
        if status is None:
            reason = "Could not update current lat/lon! It was None :("
            rospy.logwarn_throttle(5, reason)
            self.feedback_message = reason
            return pt.Status.FAILURE
        self.bb.set(bb_enums.CURRENT_LATITUDE, status.latitude)
        self.bb.set(bb_enums.CURRENT_LONGITUDE, status.longitude)
        
        return pt.Status.SUCCESS

class A_SetUTMFromGPS_vane(A_SetUTMFromGPS):
    def __init__(self):
        A_SetUTMFromGPS.__init__(self)
        
    def update(self):

        latitude = self.bb.get(bb_enums.CURRENT_LATITUDE)
        longitude = self.bb.get(bb_enums.CURRENT_LONGITUDE)
        
        if(latitude is None or latitude == 0.0 or longitude is None or latitude == 0.0):
            rospy.loginfo_throttle_identical(self._spam_period, "GPS lat/lon are 0s or Nones, cant set utm zone/band from these >:( ")
            # shitty gps
            self._spam_period = min(self._spam_period*2, self._max_spam_period)
            return pt.Status.SUCCESS

        self.gps_zone, self.gps_band = fromLatLong(math.degrees(latitude), math.degrees(longitude)).gridZone()

        if self.gps_zone is None or self.gps_band is None:
            rospy.logwarn_throttle_identical(10, "gps zone and band from fromLatLong was None")
            return pt.Status.SUCCESS

        # first read the UTMs given by ros params
        prev_band = self.bb.get(bb_enums.UTM_BAND)
        prev_zone = self.bb.get(bb_enums.UTM_ZONE)

        if prev_zone != self.gps_zone or prev_band != self.gps_band:
            rospy.logwarn_once("PREVIOUS UTM AND GPS_FIX UTM ARE DIFFERENT!\n Prev:"+str((prev_zone, prev_band))+" gps:"+str((self.gps_zone, self.gps_band)))

            if common_globals.TRUST_GPS:
                rospy.logwarn_once("USING GPS UTM!")
                self.bb.set(bb_enums.UTM_ZONE, self.gps_zone)
                self.bb.set(bb_enums.UTM_BAND, self.gps_band)
            else:
                rospy.logwarn_once("USING PREVIOUS UTM!")
                self.bb.set(bb_enums.UTM_ZONE, prev_zone)
                self.bb.set(bb_enums.UTM_BAND, prev_band)

        return pt.Status.SUCCESS
        
class A_UpdateNeptusEstimatedState_vane(A_UpdateNeptusEstimatedState):
    def __init__(self, estimated_state_topic):
        A_UpdateNeptusEstimatedState.__init__(self, estimated_state_topic)      

    def update(self):
        lat = self.bb.get(bb_enums.VANE_STATUS).latitude if self.bb.get(bb_enums.VANE_STATUS) else None
        lon = self.bb.get(bb_enums.VANE_STATUS).longitude if self.bb.get(bb_enums.VANE_STATUS) else None
        yaw = self.bb.get(bb_enums.VANE_STATUS).yaw if self.bb.get(bb_enums.VANE_STATUS) else None
        depth = 0

        if lat is None or lon is None or yaw is None:
            rospy.logwarn_throttle(10, "Could not update neptus estimated state because lat/lon/world_rot was None!")
            return pt.Status.SUCCESS

        # construct message for neptus
        e_state = EstimatedState()
        e_state.lat = lat
        e_state.lon= lon
        e_state.depth = depth
        e_state.psi = yaw
        #TODO Add more estimated state variables

        # send the message to neptus
        self.estimated_state_pub.publish(e_state)
        return pt.Status.SUCCESS

class A_GotoWaypoint_vane(A_GotoWaypoint):
    def __init__(self, action_namespace):
        A_GotoWaypoint.__init__(self, action_namespace)

    def initialise(self):
        if not self.action_server_ok:
            return

        wp, frame = self.bb.get(bb_enums.CURRENT_PLAN_ACTION)
        # if this is the first ever action, we need to get it ourselves
        if wp is None:
            rospy.logwarn("No wp found to execute! Was A_SetNextPlanAction called before this?")
            return

        # construct the message

        # Convert the coordinates to lat lon
        utmz = self.bb.get(bb_enums.UTM_ZONE)
        band = self.bb.get(bb_enums.UTM_BAND)

        print("GOTOWP: zone" + str(utmz) + " band" + str(band))

        if utmz is None or band is None:
            reason = "Could not update current lat/lon!"
            rospy.logwarn_throttle_identical(5, reason)
            return 

        # get positional feedback of the p2p goal
        easting, northing = wp[0], wp[1]
        # make utm point
        pnt = UTMPoint(easting=easting, northing=northing, altitude=0, zone=utmz, band=band)
        # get lat-lon
        pnt = pnt.toMsg()

        self.action_goal = MoveBaseGoal()
        self.action_goal.target_pose.pose.position.x = math.radians(pnt.latitude)
        self.action_goal.target_pose.pose.position.y = math.radians(pnt.longitude)
        self.action_goal.target_pose.pose.position.z = 0
        self.action_goal.target_pose.header.frame_id = "WGS84"
        rospy.loginfo("Goto waypoint action goal initialized")
        print("Target: " + str(self.action_goal.target_pose.pose.position))

        # ensure that we still need to send the goal
        self.sent_goal = False

################################ Vane Conditions ###############################


def const_tree(auv_config):
    """
    construct the entire tree.
    the structure of the code reflects the structure of the tree itself.
    sub-trees are constructed in inner functions.
    auv_config is in scope of all these inner functions.

    auv_config is a simple data object with a bunch of UPPERCASE fields in it.
    """
    # slightly hacky way to keep track of 'runnable' actions
    # such actions should add their names to this list in their init
    # just for Neptus vehicle state for now
    bb = pt.blackboard.Blackboard()
    bb.set(bb_enums.MANEUVER_ACTIONS, [])

    def const_data_ingestion_tree():
        read_abort = ptr.subscribers.EventToBlackboard(
            name = "A_ReadAbort",
            topic_name = auv_config.ABORT_TOPIC,
            variable_name = bb_enums.ABORT
        )

        read_gps = ReadTopic(
            name = "A_ReadGPS",
            topic_name = auv_config.GPS_FIX_TOPIC,
            topic_type = NavSatFix,
            blackboard_variables = {bb_enums.GPS_FIX:None}
        )

        read_status = ReadTopic(
            name = "A_ReadStatus",
            topic_name = auv_config.STATUS_TOPIC,
            topic_type = vane_status,
            blackboard_variables = {bb_enums.VANE_STATUS:None}
        )

        def const_neptus_tree():
            update_neptus = Sequence(name="SQ-UpdateNeptus",
                                     children=[
                A_UpdateNeptusEstimatedState_vane(auv_config.ESTIMATED_STATE_TOPIC),
                A_UpdateNeptusPlanControlState(auv_config.PLAN_CONTROL_STATE_TOPIC),
                A_UpdateNeptusVehicleState(auv_config.VEHICLE_STATE_TOPIC),
                A_UpdateNeptusPlanDB(auv_config.PLANDB_TOPIC,
                                     auv_config.UTM_LINK,
                                     auv_config.LOCAL_LINK),
                A_UpdateNeptusPlanControl(auv_config.PLAN_CONTROL_TOPIC),
                A_VizPublishPlan(auv_config.PLAN_VIZ_TOPIC)
                                     ])
            return update_neptus


        update_tf = A_UpdateTF(auv_config.UTM_LINK, auv_config.BASE_LINK)
        update_latlon = A_UpdateLatLon_Vane()
        set_utm_from_gps = A_SetUTMFromGPS_vane()
        neptus_tree = const_neptus_tree()


        return Sequence(name="SQ-DataIngestion",
                        # dont show all the things inside here
                        blackbox_level=1,
                        children=[
                            read_abort,
                            read_status,
                            read_gps,
                            set_utm_from_gps,
                            #update_tf,
                            update_latlon,
                            neptus_tree
                        ])


    def const_safety_tree():
        no_abort = C_NoAbortReceived()
        # more safety checks will go here

        safety_checks = Sequence(name="SQ-SafetyChecks",
                        blackbox_level=1,
                        children=[
                                  no_abort,
                        ])

        
        skip_wp = Sequence(name='SQ-CountEmergenciesAndSkip',
                           children = [
                               Counter(n=auv_config.EMERGENCY_TRIALS_BEFORE_GIVING_UP,
                                       name="A_EmergencyCounter",
                                       reset=True),
                               A_SetNextPlanAction()
                           ])

        return Fallback(name='FB_SafetyOK',
                        children = [
                            A_SendHeartBeat(),
                            safety_checks,
                            skip_wp
                        ])

    def const_synch_tree():
        have_refined_mission = C_HaveRefinedMission()
        have_coarse_mission = C_HaveCoarseMission()
        refine_mission = A_RefineMission(config.PATH_PLANNER_NAME,
                                         config.PATH_TOPIC)
        # we need one here too, to initialize the mission in the first place
        # set dont_visit to True so we dont skip the first wp of the plan
        set_next_plan_action = A_SetNextPlanAction(do_not_visit=True)


        refinement_tree = Sequence(name="SQ_Refinement",
                                   children=[
                                       have_coarse_mission,
                                       refine_mission,
                                       set_next_plan_action
                                   ])

        return Fallback(name='FB_SynchMission',
                        children=[
                            have_refined_mission,
                            refinement_tree
                        ])


    def const_execute_mission_tree():
        plan_complete = C_PlanCompleted()
        # but still wait for operator to tell us to 'go'
        start_received = C_StartPlanReceived()
        gotowp = A_GotoWaypoint_vane(auv_config.ACTION_NAMESPACE)
        # and this will run after every success of the goto action
        set_next_plan_action = A_SetNextPlanAction()
        plan_is_same = C_PlanIsNotChanged()
        #  idle = pt.behaviours.Running(name="Idle")

        follow_plan = Sequence(name="SQ-FollowMissionPlan",
                               children=[
                                         start_received,
                                         plan_is_same,
                                         gotowp,
                                         set_next_plan_action
                               ])

        return Fallback(name="FB-ExecuteMissionPlan",
                        children=[
                                  plan_complete,
                                  follow_plan
                                  #  idle
                        ])



    # The root of the tree is here

    planned_mission = Sequence(name="SQ_PlannedMission",
                               children=[
                                  const_synch_tree(),
                                  const_execute_mission_tree()
                               ])


    run_tree = Fallback(name="FB-Run",
                        children=[
                            planned_mission
                        ])


    root = Sequence(name='SQ-ROOT',
                    children=[
                              const_data_ingestion_tree(),
                              const_safety_tree(),
                              run_tree
                    ])

    return ptr.trees.BehaviourTree(root)



def main(config, catkin_ws_path):

    utm_zone = rospy.get_param("~utm_zone", common_globals.DEFAULT_UTM_ZONE)
    utm_band = rospy.get_param("~utm_band", common_globals.DEFAULT_UTM_BAND)

    bb = pt.blackboard.Blackboard()
    bb.set(bb_enums.UTM_ZONE, utm_zone)
    bb.set(bb_enums.UTM_BAND, utm_band)



    try:
        rospy.loginfo("Constructing tree")
        tree = const_tree(config)
        rospy.loginfo("Setting up tree")
        setup_ok = tree.setup(timeout=common_globals.SETUP_TIMEOUT)
        viz = pt.display.ascii_tree(tree.root)
        rospy.loginfo(viz)
        #path = catkin_ws_path+'/src/smarc_missions/smarc_bt/last_ran_tree.txt'
        #with open(path, 'w+') as f:
        #    f.write(viz)
        #    rospy.loginfo("Wrote the tree to {}".format(path))

        if setup_ok:
            rospy.loginfo("Ticktocking....")
            rate = rospy.Rate(common_globals.BT_TICK_RATE)

            while not rospy.is_shutdown():
                tip = tree.tip()
                if tip is None:
                    bb.set(bb_enums.TREE_TIP_NAME, '')
                    bb.set(bb_enums.TREE_TIP_STATUS, 'Status.X')
                else:
                    bb.set(bb_enums.TREE_TIP_NAME, tip.name)
                    bb.set(bb_enums.TREE_TIP_STATUS, str(tip.status))

                tree.tick()

                #  pt.display.print_ascii_tree(tree.root, show_status=True)
                rate.sleep()

        else:
            rospy.logerr("Tree could not be setup! Exiting!")

    except rospy.ROSInitException:
        rospy.loginfo("ROS Interrupt")




if __name__ == '__main__':
    # init the node
    rospy.init_node("bt")

    config = ASVConfig()

    print(config.robot_name)

    

    # uncomment this to generate bt_vane.launch file from auv_config.py
    # do this after you add a new field into auv_config.py
    # point path to where your catkin_ws is
    import os
    catkin_ws_path = os.path.expanduser("~")+'/vane_ws'
    try:
        config.generate_launch_file(catkin_ws_path)
    except Exception as e:
        print("Did not generate the launch file")
        print(e)

    # read all the fields from rosparams, lowercased and with ~ prepended
    print('@@@@@@@@@@@@@')
    config.read_rosparams()
    print(config)
    print('@@@@@@@@@@@@@')
    main(config, catkin_ws_path)

