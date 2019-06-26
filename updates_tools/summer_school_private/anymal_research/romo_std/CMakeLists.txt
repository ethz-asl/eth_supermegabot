# Project configuration
cmake_minimum_required (VERSION 2.8)
project(romo_std)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
  romo
  message_logger
  std_utils
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
 CATKIN_DEPENDS romo message_logger std_utils
)

###########
## Build ##
###########
add_definitions(-std=c++11 -Wall -Werror)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
