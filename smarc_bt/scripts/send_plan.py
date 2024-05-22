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

    #Create mission message
    mission = MissionControl()
    mission.name = "niklas_mission"
    mission.hash = ''.join(random.choices(string.ascii_lowercase, k=10))
    mission.timeout = 1000
    mission.command = MissionControl.CMD_SET_PLAN
    
    wp = Maneuver()
    wp.name = "course"
    wp.vehicle_mode = 0
    wp.course_rpm = 200
    wp.course_targetheading = 89
    wp.course_runtime_s = 10
    wp.course_targetAltitude = 10
    wp.course_targetDepth = 0
    wp.maneuver_type = Maneuver.MANEUVER_TYPE_COURSE
    mission.maneuvers.append(wp)

    wp = Maneuver()
    wp.name = "course2"
    wp.vehicle_mode = 0
    wp.course_rpm = 200
    wp.course_targetheading = 0
    wp.course_runtime_s = 30
    wp.course_targetAltitude = 10
    wp.course_targetDepth = 0
    wp.maneuver_type = Maneuver.MANEUVER_TYPE_COURSE
    mission.maneuvers.append(wp)


    wp = Maneuver()
    wp.name = str("7")
    wp.vehicle_mode = 0
    wp.wp_goal_tolerance = 10
    wp.wp_rpm = 200
    wp.wp_targetDepth = 0
    wp.wp_targetAltitude = 10
    wp.wp_targetLat = 58.25259
    wp.wp_targetLon = 11.46197+0.001*6
    wp.maneuver_type = Maneuver.MANEUVER_TYPE_WP
    mission.maneuvers.append(wp)
    
    r.sleep()
    pub.publish(mission)

    cmd = MissionControl()
    cmd.name = mission.name
    cmd.hash = mission.hash
    #cmd.timeout = 1000
    cmd.command = MissionControl.CMD_START
    r.sleep()
    pub.publish(cmd)


    i = 0
    while not rospy.is_shutdown() and i < 60:
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
