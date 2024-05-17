#!/usr/bin/env python

import rospy
from smarc_bt.msg import MissionControl, GotoWaypoint

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
        wp = GotoWaypoint()
        wp.pose.header.frame_id = 'utm'
        wp.pose.pose.position.x = 651740.1628267602
        wp.pose.pose.position.y = 6523160.96751601
        wp.goal_tolerance = 10.0
        wp.z_control_mode = 1
        wp.travel_altitude = -1.0
        wp.travel_depth = -5.0
        wp.speed_control_mode = 1
        wp.travel_rpm = 400.0
        wp.travel_speed = 0.0
        wp.lat = 58.821559689368776
        wp.lon = 17.627995331480648
        wp.arrival_heading = 0.0
        wp.use_heading = False
        wp.name = str(i)
        wp.vehicle_mode = ''
        
        mission.waypoints.append(wp)
    
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
