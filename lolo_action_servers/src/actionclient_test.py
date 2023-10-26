#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib
from smarc_bt.msg import GotoWaypointAction, GotoWaypointGoal, GotoWaypoint, MissionControl

class Actionserver_tester(object):

    def __init__(self):
        rospy.init_node('waypoint_action_test_py')
        self.client = actionlib.SimpleActionClient('/lolo/ctrl/goto_waypoint', GotoWaypointAction)
        print("client started")

    def gotoWP(self, x, y, depth, altitude, RPM):
        try:
            self.client.cancel_all_goals()
            
            # Waits until the action server has started up and started
            # listening for goals.
            print("waiting for server")
            self.client.wait_for_server()

            # Creates a goal to send to the action server.
            goal = GotoWaypointGoal()
            goal.waypoint.pose.pose.position.x = x
            goal.waypoint.pose.pose.position.y = y
            #goal.waypoint.pose.header.frame_id = 'utm'
            #goal.waypoint.pose.header.frame_id = 'map'
            goal.waypoint.pose.header.frame_id = 'map'
            goal.waypoint.travel_depth = depth
            goal.waypoint.travel_rpm = RPM
            goal.waypoint.goal_tolerance = 5
            goal.waypoint.speed_control_mode = GotoWaypoint.SPEED_CONTROL_RPM
            goal.waypoint.z_control_mode = GotoWaypoint.Z_CONTROL_DEPTH
            goal.waypoint.travel_altitude = altitude
            goal.waypoint.z_control_mode = GotoWaypoint.Z_CONTROL_ALTITUDE


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
    at.gotoWP(x=330,y=700,depth=0.5,altitude=5,RPM=200)
    at.gotoWP(x=120,y=220,depth=10,altitude=5,RPM=200)
    at.gotoWP(x=100,y=200,depth=10,altitude=5,RPM=200)

    

    