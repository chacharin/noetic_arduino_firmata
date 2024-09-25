cmake_minimum_required(VERSION 3.0.2)
project(noetic_uno_firmata)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  src/arduino_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
