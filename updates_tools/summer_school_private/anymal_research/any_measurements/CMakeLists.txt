cmake_minimum_required(VERSION 2.8.3)
project(any_measurements)


# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS message_logger
  DEPENDS 
    kindr
)

###########
## Build ##
###########
#add_definitions(-std=c++11)

#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#)


#add_library(${PROJECT_NAME}
#
#)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
