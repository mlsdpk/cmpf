cmake_minimum_required(VERSION 3.0.2)
project(cmpf_decoupled_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(${PROJECT_NAME}_CATKIN_DEPS
  roscpp
  cmpf_msgs
  carla_msgs
  cmpf_core
  geometry_msgs
  pluginlib
  tf2
  tf2_ros
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cmpf_decoupled_controller
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

# DecoupledController plugin
add_library(cmpf_decoupled_controller
  src/decoupled_controller.cpp
)
add_dependencies(cmpf_decoupled_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(cmpf_decoupled_controller
  ${catkin_LIBRARIES}
)

# PurePursuit plugin
add_library(pure_pursuit_controller
  plugins/pure_pursuit_controller.cpp
)
add_dependencies(pure_pursuit_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(pure_pursuit_controller
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS
  cmpf_decoupled_controller
  pure_pursuit_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES 
  decoupled_controller_plugin.xml
  controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)