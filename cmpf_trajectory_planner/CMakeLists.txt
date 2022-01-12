cmake_minimum_required(VERSION 3.0.2)
project(cmpf_trajectory_planner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(${PROJECT_NAME}_CATKIN_DEPS
  roscpp
  actionlib 
  pluginlib
  cmpf_msgs
  cmpf_core
  cmpf_utils
  tf2
  tf2_ros
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_nodelet
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

include_directories( 
  include()
  ${catkin_INCLUDE_DIRS}
)

# Trajectory Planner nodelet
add_library(${PROJECT_NAME}_nodelet
  src/trajectory_planner_nodelet.cpp
)
add_dependencies(${PROJECT_NAME}_nodelet 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
)

# Plugins

# Dummy Trajectory Planner
add_library(cmpf_dummy_trajectory_planner
  plugins/dummy_trajectory_planner.cpp
)
add_dependencies(cmpf_dummy_trajectory_planner
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(cmpf_dummy_trajectory_planner
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}_nodelet
  cmpf_dummy_trajectory_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES 
  cmpf_trajectory_planner_nodelet_plugin.xml
  cmpf_trajectory_planner_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)