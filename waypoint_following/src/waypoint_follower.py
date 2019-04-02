#! /usr/bin/env python
# Christopher Iliffe Sprague
# sprague@kth.se

import py_trees as pt, py_trees_ros as ptr, rospy, numpy as np
from std_msgs.msg import Empty, Bool, Float64, String
from nav_msgs.msg import Odometry
from waypoint_following.msg import WaypointAction, WaypointGoal
from go_action_server import Go_Action_Server

class Waypoint_Follower(ptr.trees.BehaviourTree):

    def __init__(self, waypoints):

        # list of 3D waypoints
        self.waypoints = waypoints

        # initial waypoint index
        self.i = 0

        # data tree
        self.abort = ptr.subscribers.EventToBlackboard(
            name='abort',
            topic_name='/abort',
            variable_name='abort'
        )
        self.pose = ptr.subscribers.ToBlackboard(
            name='pose',
            topic_name='/sam_auv_1/pose_gt',
            topic_type=Odometry,
            blackboard_variables={'pose':'pose'},
            initialise_variables={'pose':0},
            clearing_policy=pt.common.ClearingPolicy.NEVER
        )
        self.data = pt.composites.Sequence(
            name="data",
            children=[self.abort, self.pose]
        )

        # waypoint tree
        self.go_server = Go_Action_Server().start()
        self.go_client = ptr.actions.ActionClient(
            name='go_to_waypoint',
            action_spec=WaypointAction,
            action_goal=WaypointGoal(str(self.waypoints[self.i])),
            action_namespace='/go_to_waypoint'
        )


        self.root = pt.composites.Parallel(
            name='root',
            children=[self.data, self.go_client]
        )


        
        ptr.trees.BehaviourTree.__init__(self, self.root)

                
                




if __name__ == "__main__":

    # instantiate the behaviour tree
    rospy.init_node('waypoint_follower')
    tree = Waypoint_Follower([(100, 100, -10), (-100, 100, -10)])
    tree.setup(timeout=100)
    tree.tick_tock(
        1000,
        number_of_iterations=pt.trees.CONTINUOUS_TICK_TOCK,
        post_tick_handler=lambda t: pt.display.print_ascii_tree(t.root, show_status=True)
    )

