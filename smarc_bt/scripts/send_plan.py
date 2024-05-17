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
    mission.hash = res = ''.join(random.choices(string.ascii_lowercase, k=10))
    mission.timeout = 1000
    mission.command = MissionControl.CMD_SET_PLAN

    for i in range(3):
        wp = Maneuver()
        wp.name = str(i)
        wp.vehicle_mode = 0
        wp.wp_goal_tolerance = 5
        wp.wp_rpm = 200
        wp.wp_targetDepth = 0
        wp.wp_targetAltitude = 10
        wp.wp_targetLat = 58.821559689368776
        wp.wp_targetLon = 17.627995331480648
        
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
    while not rospy.is_shutdown() and i < 10:
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
