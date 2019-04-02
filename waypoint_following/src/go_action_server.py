# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

import py_trees as pt, py_trees_ros as ptr, rospy, numpy as np, rospy
from waypoint_following.msg import WaypointAction, WaypointGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Bool, Float64, String
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

class Go_Action_Server(ptr.mock.action_server.ActionServer):

    def __init__(self):

        # inherit 
        ptr.mock.action_server.ActionServer.__init__(
            self,
            '/go_to_waypoint',
            WaypointAction,
            None
        )

        # state space
        self.state = rospy.Subscriber('sam_auv_1/pose_gt', Odometry, self.get_state)

        # action space
        self.thrust0 = rospy.Publisher(
            '/sam_auv_1/thrusters/0/input',
            FloatStamped,
            queue_size=100
        )
        self.thrust1 = rospy.Publisher(
            '/sam_auv_1/thrusters/0/input',
            FloatStamped,
            queue_size=100
        )
        self.pitch = rospy.Publisher(
            '/sam_auv_1/joint1_position_controller/command',
            Float64,
            queue_size=100
        )
        self.yaw = rospy.Publisher(
            '/sam_auv_1/joint2_position_controller/command',
            Float64,
            queue_size=100
        )


    def get_state(self, data):
        self.state = np.array([
            data.pose.pose.position.x,
            data.pose.pose.position.y,
            data.pose.pose.position.z,
            data.twist.twist.linear.x,
            data.twist.twist.linear.y,
            data.twist.twist.linear.z
        ])

    def execute(self, goal):

        # get waypoint
        wpi = np.array(eval(goal.waypoint))

        # print the goal
        rospy.loginfo("Recieved waypoint {}".format(wpi))

        # pid parameters
        error_p  = np.zeros(3)
        integral = np.zeros(3)
        dt = 0.1

        # execute
        while True:

            # if we're quiting
            if rospy.is_shutdown() or self.action_server.is_preempt_requested():
                self.action_server.set_preempted(self.action.action_result.result, "goal was preempted")
                success = False
                break

            # if we're trying 
            else:

                # relative position to target
                p = wpi - self.state[:3]
                
                # direction to target
                phat = p/np.linalg.norm(p)

                # heading direction
                vhat = self.state[3:]/np.linalg.norm(self.state[3:])

                # pid
                error = phat - vhat
                integral += error*dt
                derrivative =(error - error_p)/dt
                error_p = np.copy(error)

                print(error)

                # full throttle
                thrust = FloatStamped()
                thrust.header.frame_id = 'sam_auv_1'
                thrust.data = 100
                self.thrust0.publish(thrust)
                self.thrust1.publish(thrust)
                
                # pitch and yaw
                out = 1*np.linalg.norm(error[:2]) + 0*np.linalg.norm(integral[:2]) + 0.1*np.linalg.norm(derrivative[:2])
                self.pitch.publish(out)
                out = 1*error[-1] + 0*integral[-1] + 0.1*derrivative[-1]
                self.yaw.publish(out)