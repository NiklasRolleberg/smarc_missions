#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

import time, math

import rospy, tf
from geometry_msgs.msg import PointStamped
from geographic_msgs.msg import GeoPoint
from smarc_msgs.msg import DVL, Leak, ThrusterFeedback
from smarc_bt.msg import GotoWaypoint
from sensor_msgs.msg import NavSatFix, BatteryState
from sam_msgs.msg import PercentStamped
from lolo_msgs.msg import Pressures
from lolo_msgs.msg import Temperatures
from lolo_msgs.msg import Status
from std_msgs.msg import Float64
from std_msgs.msg import Float32

RADTODEG = 360 / (math.pi * 2)

class StringAnimation(object):
    """
    A nice little animation thing to show updates happening
    in a string format
    """
    def __init__(self, num_slots):
        self._frames = "◜◝◞◟"
        self._current_frames = [-1] * num_slots
        self._str = "X" * num_slots

    def update(self, slot):
        self._current_frames[slot] = (self._current_frames[slot] + 1) % len(self._frames)

    def __str__(self):
        s = ""
        for slot in range(len(self._str)):
            if self._current_frames[slot] == -1:
                f = 'X'
            else:
                f = self._frames[self._current_frames[slot]]
            s += f
        self._str = s
        return self._str



class Vehicle(object):
    """
    A common vehicle object to keep track of all common
    data about a vehicle
    """
    def __init__(self,
                 auv_config):

        self.auv_config = auv_config
        self.robot_name = auv_config.robot_name

        # for visualizations
        self._animation = StringAnimation(num_slots=5)
        self.last_goto_wp = GotoWaypoint()
        self._last_wp_pub = rospy.Publisher(self.auv_config.LAST_WP_TOPIC, GotoWaypoint, queue_size=1)

        self._init_tf_vars()
        # some state strings to be reported in case of trouble
        self._status_str_tf = "Uninitialized"
        self._last_update_tf = -1
        
        # these will come from dvl
        self.dvl_velocity_msg = None
        self._dvl_sub = rospy.Subscriber(self.auv_config.DVL_TOPIC, DVL, self._dvl_cb, queue_size=2)
        self._status_str_dvl = "Uninitialized"
        self._last_update_dvl = -1

        # leak...
        self.leak = None
        self._leak_sub = rospy.Subscriber(self.auv_config.LEAK_TOPIC, Leak, self._leak_cb, queue_size=2)
        self._status_str_leak = "Uninitialized"
        self._last_update_leak = -1

        # raw lat lon 
        self.position_latlon = [None, None]
        self._latlon_sub = rospy.Subscriber(self.auv_config.LATLON_TOPIC, GeoPoint, self._latlon_cb, queue_size=2)

        # raw GPS object
        self.raw_gps_obj = None
        self._gps_sub = rospy.Subscriber(self.auv_config.GPS_TOPIC, NavSatFix, self._gps_cb, queue_size=2)
        self._status_str_gps = "Uninitialized"
        self._last_update_gps = -1

        '''
        # VBS, LCG
        self.vbs = None
        self.lcg = None
        self._vbs_sub = rospy.Subscriber(self.auv_config.VBS_TOPIC, PercentStamped, self._vbs_cb, queue_size=2)
        self._lcg_sub = rospy.Subscriber(self.auv_config.LCG_TOPIC, PercentStamped, self._lcg_cb, queue_size=2)
        '''

        #Depth
        self.depth = None
        self._depth_sub = rospy.Subscriber(self.auv_config.DEPTH_TOPIC, Float64, self._depth_cb, queue_size=2)
        self._status_str_depth = "Uninitialized"
        self.last_update_depth = -1
        self.last_time_at_surface = -1

        # Altitude
        self.altitude = None
        self._altitude_sub = rospy.Subscriber(self.auv_config.ALTITUDE_TOPIC, Float32, self._altitude_cb, queue_size=2)
        self._status_str_altitude = "Uninitialized"
        self.last_update_altitude = -1

        # thrusters
        self.t1 = None
        self.t2 = None
        self._t1_sub = rospy.Subscriber(self.auv_config.T1_TOPIC, ThrusterFeedback, self._t1_cb, queue_size=2)
        self._t2_sub = rospy.Subscriber(self.auv_config.T2_TOPIC, ThrusterFeedback, self._t2_cb, queue_size=2)

        # battery1
        self.batt_v = None
        self.batt_percent = None
        self._batt_sub = rospy.Subscriber(self.auv_config.BATT_TOPIC, BatteryState, self._batt_cb, queue_size=2)
        
        # battery2?
        #self.batt_v = None
        #self.batt_percent = None
        #self._batt_sub = rospy.Subscriber(self.auv_config.BATT_TOPIC, BatteryState, self._batt_cb, queue_size=2)

        #Temperatures
        self.cap_temp = None
        self.esc_temp = None
        self.sci_temp = None
        self.bat1_temp = None
        self.bat2_temp = None
        self.edw_temp = None
        self._batt_sub = rospy.Subscriber(self.auv_config.TEMP_TOPIC, Temperatures, self._temperature_callback, queue_size=2)

        #System

        #Power out
        self.servo1_output = None
        self.servo2_output = None
        self.aux_output = None
        self.lumen_output = None

        #ISB communication status
        self.edw_OK = None #edw_status
        self.trigger_OK = None #trigger_status
        self.battery1_OK = None #battery1_status
        self.battery2_OK = None #battery2_status
        self.actuators_OK = None #actuators_status
        self.thrusters_OK = None #thrusters_status
        self.vertical_thrusters_OK = None #vertical_thrusters_status
        self.usbl_OK = None #usbl_status
        self.time_OK = None #time_status
        self.scientist_OK = None #scientist_status

        #Leak checks
        self.captain_leak = None
        self.esc_leak = None
        self.edw_leak = None
        self.prevco_leak = None
        self.battery1_leak = None
        self.battery2_leak = None

        self._system_sub = rospy.Subscriber(self.auv_config.SYSTEM_STATUS_TOPIC, Status, self._status_callback, queue_size=2)





    def __str__(self):
        anim = self._animation.__str__()
        status = [
            ('TF', anim[0], 'Depth:{}'.format(self.depth)),
            ('DVL', anim[1], 'Alt:{}'.format(self.altitude)),
            ('Leak', anim[2], self._status_str_leak),
            ('Latlon', anim[3], str(self.position_latlon)),
            ('GPS', anim[4], self._status_str_gps)
        ]

        s = ""

        for name, frame, string in status:
            s += "{}:{} - {}\n".format(name, frame, string)
        return s


    def _init_tf_vars(self):
        # these will come from the TF tree
        # and are None'd at every update attempt
        # position does not include height or depth to avoid confusion
        # use depth for that
        self.position_utm = [None, None]
        self.orientation_quat = [None, None, None, None]
        self.orientation_rpy = [None, None, None]
        # northing, north = 0, east = 90, south = 180
        self.heading = None
        # for convenicent use in ROS elsewhere
        self.position_point_stamped = None


    def setup_tf_listener(self, timeout_secs=120):
        """
        create a tf listener to be used later and return it
        because we cant store a tf listener in the blackboard of a BT
        due to serialization problems
        so we just... dont store it in this object...
        """
        listener = tf.TransformListener()
        try:
            listener.waitForTransform(self.auv_config.UTM_LINK,
                                      self.auv_config.BASE_LINK,
                                      rospy.Time(),
                                      rospy.Duration(secs=timeout_secs))
            self._status_str_tf = "Got xform"
            return listener
        except:
            self._status_str_tf = "waitForTransform failed from '{}' to '{}' after {}s, is the TF tree in one piece?".format(self.auv_config.UTM_LINK, self.auv_config.BASE_LINK, timeout_secs)
            return None


    def tick(self, tf_listener):
        """
        mimic the behaviour of the BT, since this should be in lock-step with it
        """
        self._update_tf(tf_listener)
        self._last_wp_pub.publish(self.last_goto_wp)


    def _update_tf(self, listener):
        # init the vars so that we can catch later if they are
        # updated properly
        self._init_tf_vars()
        # create a fresh  listener every time, because the BB will break
        # when we put this object in it with the listener as a variable
        try:
            posi, ori = listener.lookupTransform(self.auv_config.UTM_LINK,
                                                 self.auv_config.BASE_LINK,
                                                 rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
            self._status_str_tf = "lookupTransform failed from '{}' to '{}', is the TF tree in one piece?".format(self.auv_config.UTM_LINK, self.auv_config.BASE_LINK)
            return
        except Exception as e:
            self._status_str_tf = "lookupTransform failed:\n{}".format(e)
            return


        # position for x,y
        self.position_utm = [posi[0], posi[1]]
        # depth for z.
        #self.depth = -posi[2]
        self.orientation_quat = [ori[0], ori[1], ori[2], ori[3]]
        rpy = tf.transformations.euler_from_quaternion(ori)
        self.orientation_rpy = [rpy[0], rpy[1], rpy[2]]
        # heading from yaw. VERY HACKY
        self.heading = RADTODEG * (math.pi/2 - rpy[2])

        ps = PointStamped()
        ps.header.frame_id = self.auv_config.UTM_LINK
        ps.header.stamp = rospy.Time(0)
        ps.point.x = posi[0]
        ps.point.y = posi[1]
        ps.point.z = posi[2]
        self.position_point_stamped = ps

        self._status_str_tf = "TF Up to date"
        self._last_update_tf = time.time()
        self._animation.update(0)


    def _dvl_cb(self, msg):
        #self.altitude = msg.altitude
        self.dvl_velocity_msg = msg.velocity
        self._last_update_dvl = time.time()
        self._status_str_dvl = "Working"
        self._animation.update(1)

    def _leak_cb(self, msg):
        self.leak = msg.value
        self._last_update_leak = time.time()
        self._status_str_leak = "Working"
        self._animation.update(2)

    def _latlon_cb(self, msg):
        self.position_latlon = [msg.latitude, msg.longitude]
        self._animation.update(3)

    def _gps_cb(self, msg):
        self.raw_gps_obj = msg
        self._status_str_gps = "Working"
        self._last_update_gps = time.time()
        self._animation.update(4)

    def _vbs_cb(self, msg):
        self.vbs = msg.value

    def _lcg_cb(self, msg):
        self.lcg = msg.value

    def _t1_cb(self, msg):
        self.t1 = msg.rpm.rpm

    def _t2_cb(self, msg):
        self.t2 = msg.rpm.rpm

    def _batt_cb(self, msg):
        self.batt_v = msg.voltage
        self.batt_percent = msg.percentage

    def _depth_cb(self, msg: float) -> None:
        self.depth = msg.data
        self.last_update_depth = time.time()
        if(self.depth < 1): #We are at the surface
            self.last_time_at_surface = time.time()
        self._status_str_depth = "Working"
        self._animation.update(1)

    def _altitude_cb(self,msg : float) ->None:
        self.altitude = msg.data
        self.last_update_altitude = time.time()
        self._status_str_altitude = "Working"
        self._animation.update(1)

    def _temperature_callback(self, msg:Temperatures) -> None:
        self.cap_temp = msg.captain_eth
        self.esc_temp = max(msg.port_esc, msg.strb_esc)
        self.sci_temp = msg.prevco_isb
        self.bat1_temp = msg.battery1_isb
        self.bat2_temp = msg.battery2_isb
        self.edw_temp = msg.edw_isb

    def _status_callback(self, msg:Temperatures) -> None:
        #Power out
        self.servo1_output = msg.servo1_output
        self.servo2_output = msg.servo2_output
        self.aux_output = msg.aux_output
        self.lumen_output = msg.lumen_output

        #ISB communication status
        self.edw_OK = msg.edw_status
        self.trigger_OK = msg.trigger_status
        self.battery1_OK = msg.battery1_status
        self.battery2_OK = msg.battery2_status
        self.actuators_OK = msg.actuators_status
        self.thrusters_OK = msg.thrusters_status
        self.vertical_thrusters_OK = msg.vertical_thrusters_status
        self.usbl_OK = msg.usbl_status
        self.time_OK = msg.time_status
        self.scientist_OK = msg.scientist_status

        #Leak checks
        self.captain_leak = msg.captain_leak
        self.esc_leak = msg.esc_leak
        self.edw_leak = msg.edw_leak
        self.prevco_leak = msg.prevco_leak
        self.battery1_leak = msg.battery1_leak
        self.battery2_leak = msg.battery2_leak
