cmake_minimum_required(VERSION 3.0.2)
project(cmpf_msgs)

find_package(catkin REQUIRED COMPONENTS 
  actionlib 
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
) 

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  TrajectoryMsg.msg
  Lanelet2MapBin.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Lanelet2Map.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  NavigateToPose.action
  ComputeRouteToPose.action
  Trajectory.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
  nav_msgs
	actionlib_msgs
)

catkin_package(
	CATKIN_DEPENDS 
  actionlib_msgs 
  std_msgs 
  geometry_msgs
  nav_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)