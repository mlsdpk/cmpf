cmake_minimum_required(VERSION 3.0.2)
project(cmpf_controller_server)

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
  tf2
  tf2_ros
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

catkin_package(
  LIBRARIES ${PROJECT_NAME}_nodelet
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

include_directories( 
  ${catkin_INCLUDE_DIRS}
)

# Controller nodelet
add_library(${PROJECT_NAME}_nodelet
  src/controller_server_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES 
  cmpf_controller_server_nodelet_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)