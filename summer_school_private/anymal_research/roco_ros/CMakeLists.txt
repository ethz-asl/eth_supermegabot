#-------------------------------------------------------------------------------
# Robotic Systems Lab
# ETH Zurich
#
# Author(s): Gabriel Hottiger
# email    : ghottiger@anybotics.com
# Date     : Nov-2016
#
#-------------------------------------------------------------------------------
cmake_minimum_required (VERSION 2.8)
project(roco_ros)

# Find catkin
find_package(catkin REQUIRED COMPONENTS
  message_logger
  roco
  roscpp
)

find_package(Boost REQUIRED)

# Catkin package definition
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
    message_logger
    roco
    roscpp
  DEPENDS
    Boost
)

add_definitions(-std=c++11 -Wall -Werror)

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
