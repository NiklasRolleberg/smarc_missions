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
    mission.command = MissionControl.CMD_STOP
    pub.publish(mission)


    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
