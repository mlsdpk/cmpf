cmake_minimum_required(VERSION 3.0.2)
project(cmpf_behavior_prediction)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()
catkin_python_setup()

# Install
catkin_install_python(PROGRAMS src/behavior_prediction_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)