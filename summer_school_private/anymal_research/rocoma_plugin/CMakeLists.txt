cmake_minimum_required(VERSION 2.8.3)
project(rocoma_plugin)

add_definitions(-std=c++11 -Wall -Werror)

find_package(catkin REQUIRED
  COMPONENTS
    roco
    roco_ros
    rocoma
    pluginlib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
    roco
    roco_ros
    rocoma
    pluginlib
  DEPENDS
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
