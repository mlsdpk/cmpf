cmake_minimum_required(VERSION 3.0.2)
project(cmpf_bringup)

find_package(catkin REQUIRED)

catkin_package(
)

install(
  DIRECTORY 
  launch
  map
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

