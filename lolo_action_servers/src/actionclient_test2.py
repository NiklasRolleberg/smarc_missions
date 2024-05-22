#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib
from smarc_bt.msg import FollowCourseAction, FollowCourseGoal
from random import random

class Actionserver_tester(object):

    def __init__(self):
        rospy.init_node('waypoint_action_test_py')
        self.client = actionlib.SimpleActionClient('/lolo/ctrl/follow_course', FollowCourseAction)
        print("client started")

    def followCourse(self, course, depth, altitude, RPM, runtime):
        try:
            self.client.cancel_all_goals()
            
            # Waits until the action server has started up and started
            # listening for goals.
            print("waiting for server")
            self.client.wait_for_server()

            # Creates a goal to send to the action server.
            goal = FollowCourseGoal()
            goal.targetheading_deg = course
            goal.rpm = RPM
            goal.runtime_s = runtime
            goal.targetAltitude = altitude
            goal.targetDepth = depth
    
            # Sends the goal to the action server.
            self.client.send_goal(goal)
            print("goal sent")

            # Waits for the server to finish performing the action.
            print("waiting for server to finnish action")
            self.client.wait_for_result()

            # Prints out the result of executing the action
            result = self.client.get_result()  # A FibonacciResult
            return result
            

        except rospy.ROSInterruptException:
            self.client.cancel_all_goals()
            print("program interrupted before completion", file=sys.stderr)
            return None


if __name__ == '__main__':

    #kristineberg : 
    # max: x=330. y=700
    # min: 

    at = Actionserver_tester()
    #at.gotoWP(x=330,y=700,depth=0.5,altitude=5,RPM=200)
    #res = at.gotoWP(x=120,y=220,depth=10,altitude=5,RPM=200)
    #print(res)
    #res = at.gotoWP(x=100,y=200,depth=10,altitude=5,RPM=200)
    #print(res)

    #at.gotoWP(x=120,y=700,depth=5,altitude=5,RPM=250)

    for i in range(1):
        x = 120 + 100*(random()-0.5)
        y = 700 + 200*(random()-0.5)
        depth = 5+2*random()
        at.followCourse(course = 10,depth=5,altitude=5,RPM=250, runtime = 10)
    

    