#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib
from smarc_bt.msg import GotoWaypointAction, GotoWaypointGoal, GotoWaypoint, MissionControl


if __name__ == '__main__':

    rospy.init_node('waypoint_action_test_py')
    client = actionlib.SimpleActionClient('/lolo/ctrl/goto_waypoint', GotoWaypointAction)
    print("client started")

    try:

        client.cancel_all_goals()
        
        # Waits until the action server has started up and started
        # listening for goals.
        print("waiting for server")
        client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = GotoWaypointGoal()
        goal.waypoint.pose.pose.position.x = -1000
        goal.waypoint.pose.pose.position.y = -100
        #goal.waypoint.pose.header.frame_id = 'utm'
        #goal.waypoint.pose.header.frame_id = 'map'
        goal.waypoint.pose.header.frame_id = 'map'
        goal.waypoint.travel_depth = 300
        goal.waypoint.travel_rpm = 400
        goal.waypoint.goal_tolerance = 5
        goal.waypoint.speed_control_mode = GotoWaypoint.SPEED_CONTROL_RPM
        #goal.waypoint.z_control_mode = GotoWaypoint.Z_CONTROL_DEPTH
        goal.waypoint.travel_altitude = 15
        goal.waypoint.z_control_mode = GotoWaypoint.Z_CONTROL_ALTITUDE


        # Sends the goal to the action server.
        client.send_goal(goal)
        print("goal sent")

        # Waits for the server to finish performing the action.
        print("waiting for server to finnish action")
        client.wait_for_result()
        

        # Prints out the result of executing the action
        result = client.get_result()  # A FibonacciResult
        print("Result: " + str(result))
        

    except rospy.ROSInterruptException:
        client.cancel_all_goals()
        print("program interrupted before completion", file=sys.stderr)