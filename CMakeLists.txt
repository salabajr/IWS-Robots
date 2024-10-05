cmake_minimum_required(VERSION 3.0.2)
project(ur3_leader_follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  message_generation
  genmsg
  actionlib_msgs
  cartesian_control_msgs
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Error.msg
  Sequence.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ActuateGripper.srv
  GoToPose.srv
  StartRecording.srv
)

add_action_files(
  DIRECTORY action
  FILES
  FollowWaypoints.action
  FollowFrame.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(
  PROGRAMS
  nodes/leader_node.py
  nodes/follower_node.py
  nodes/bar_broadcaster_node.py
  nodes/supervisor_node.py
  nodes/motion_trigger_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

