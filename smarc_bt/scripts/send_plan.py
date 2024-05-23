#!/usr/bin/env python

import rospy
from smarc_bt.msg import MissionControl, Maneuver

import string
import random
from datetime import datetime

def main():
    #Random seed
    random.seed(datetime.now().timestamp())
    
    #Ros node and publisher
    rospy.init_node('misisonsender', anonymous=True)
    pub = rospy.Publisher('/lolo/smarc_bt/mission_control', MissionControl, queue_size=400)
    r = rospy.Rate(1)
    r.sleep()

    #Create mission message
    mission = MissionControl()
    mission.name = "niklas_mission"
    mission.hash = ''.join(random.choices(string.ascii_lowercase, k=10))
    mission.timeout = 1800
    mission.command = MissionControl.CMD_SET_PLAN
    
    ## Mission

    #Go to start point ---OK
    wp = Maneuver()
    wp.name = str("wp 1")
    wp.vehicle_mode = 0
    wp.wp_goal_tolerance = 10
    wp.wp_rpm = 350
    wp.wp_targetDepth = -1
    wp.wp_targetAltitude = 10
    wp.wp_targetLat = 58.82322942631409
    wp.wp_targetLon = 17.634719610214233
    wp.maneuver_type = Maneuver.MANEUVER_TYPE_WP
    mission.maneuvers.append(wp)

    #Align to course ---OK
    wp = Maneuver()
    wp.name = "course1"
    wp.vehicle_mode = 0
    wp.course_rpm = 0
    wp.course_targetheading = 200
    wp.course_runtime_s = 120
    wp.course_targetAltitude = 10
    wp.course_targetDepth = -1
    wp.maneuver_type = Maneuver.MANEUVER_TYPE_COURSE
    mission.maneuvers.append(wp)

    #Start moving slowly ---OK
    wp = Maneuver()
    wp.name = "course2"
    wp.vehicle_mode = 0
    wp.course_rpm = 200
    wp.course_targetheading = 200
    wp.course_runtime_s = 90
    wp.course_targetAltitude = 10
    wp.course_targetDepth = -1
    wp.maneuver_type = Maneuver.MANEUVER_TYPE_COURSE
    mission.maneuvers.append(wp)

    #Speed up ---OK
    wp = Maneuver()
    wp.name = "course3"
    wp.vehicle_mode = 0
    wp.course_rpm = 400
    wp.course_targetheading = 200
    wp.course_runtime_s = 60
    wp.course_targetAltitude = 10
    wp.course_targetDepth = -1
    wp.maneuver_type = Maneuver.MANEUVER_TYPE_COURSE
    mission.maneuvers.append(wp)

    #Dive ---OK
    wp = Maneuver()
    wp.name = "course4"
    wp.vehicle_mode = 0
    wp.course_rpm = 400
    wp.course_targetheading = 200
    wp.course_runtime_s = 60
    wp.course_targetAltitude = 1
    wp.course_targetDepth = 2
    wp.maneuver_type = Maneuver.MANEUVER_TYPE_COURSE
    mission.maneuvers.append(wp)

    #Surface ---OK
    wp = Maneuver()
    wp.name = "course4"
    wp.vehicle_mode = 0
    wp.course_rpm = 200
    wp.course_targetheading = 200
    wp.course_runtime_s = 30
    wp.course_targetAltitude = 10
    wp.course_targetDepth = -1
    wp.maneuver_type = Maneuver.MANEUVER_TYPE_COURSE
    mission.maneuvers.append(wp)

    #Go to start point
    wp = Maneuver()
    wp.name = str("wp2")
    wp.vehicle_mode = 0
    wp.wp_goal_tolerance = 10
    wp.wp_rpm = 350
    wp.wp_targetDepth = -1
    wp.wp_targetAltitude = 10
    wp.wp_targetLat = 58.82322942631409
    wp.wp_targetLon = 17.634719610214233
    wp.maneuver_type = Maneuver.MANEUVER_TYPE_WP
    mission.maneuvers.append(wp)

    #Go to start point
    wp = Maneuver()
    wp.name = str("wp3")
    wp.vehicle_mode = 0
    wp.wp_goal_tolerance = 10
    wp.wp_rpm = 350
    wp.wp_targetDepth = -1
    wp.wp_targetAltitude = 10
    wp.wp_targetLat = 58.82322942631409
    wp.wp_targetLon = 17.634719610214233
    wp.maneuver_type = Maneuver.MANEUVER_TYPE_WP
    mission.maneuvers.append(wp)


    pub.publish(mission)

    cmd = MissionControl()
    cmd.name = mission.name
    cmd.hash = mission.hash
    #cmd.timeout = 1000
    cmd.command = MissionControl.CMD_START
    r.sleep()
    pub.publish(cmd)

    return
    i = 0
    while not rospy.is_shutdown() and i < 200:
        r.sleep()
        i+=1
        print(i)

    cmd.command = MissionControl.CMD_STOP
    pub.publish(cmd)


    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
