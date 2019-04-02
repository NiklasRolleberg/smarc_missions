cmake_minimum_required(VERSION 2.8.3)
project(waypoint_following)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  roscpp
  rospy
  std_msgs
)

add_action_files(DIRECTORY action FILES Waypoint.action)

 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
 )

 catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
