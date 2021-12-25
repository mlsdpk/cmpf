cmake_minimum_required(VERSION 3.0.2)
project(cmpf_behavior_tree)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(${PROJECT_NAME}_CATKIN_DEPS
  roscpp
  behaviortree_cpp_v3
  cmpf_msgs
  actionlib
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bt_server_nodelet ${bt_plugin_libs}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

include_directories( 
  include
  ${catkin_INCLUDE_DIRS}
)

# Controller nodelet
add_library(bt_server_nodelet
  src/bt_server_nodelet.cpp
)
add_dependencies(bt_server_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(bt_server_nodelet
  ${catkin_LIBRARIES}
)

# BT plugins
add_library(cmpf_follow_trajectory_action_client_bt_node plugins/follow_trajectory_action_client.cpp)
list(APPEND bt_plugin_libs cmpf_follow_trajectory_action_client_bt_node)

foreach(bt_plugin ${bt_plugin_libs})
  add_dependencies(${bt_plugin}
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(${bt_plugin}
    ${catkin_LIBRARIES}
  )
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

#############
## Install ##
#############

install(TARGETS
  bt_server_nodelet
  ${bt_plugin_libs}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES 
  cmpf_bt_server_nodelet_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)