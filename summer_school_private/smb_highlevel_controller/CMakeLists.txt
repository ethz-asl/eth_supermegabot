cmake_minimum_required(VERSION 2.8.3)
project(smb_highlevel_controller)

add_definitions(-std=c++11)

find_package(Boost REQUIRED COMPONENTS thread chrono)

find_package(catkin REQUIRED
  COMPONENTS
    roslib
    any_msgs
    kindr_ros
    smb_msgs
    parameter_handler_ros
    robot_utils
    robot_utils_ros
    roco
    roscpp
    any_node
    message_logger
    signal_logger
    signal_logger_ros
    joy_manager_msgs
    signal_logger_std
    notification
#    rocoma_ros
    smb_common
    smb_model
    smb_ros
    smb_description
    smb_description_ros
    smb_roco
    signal_handler
    cosmo_ros
    param_io
    rocoma_ros
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    smb_highlevel_controller
  CATKIN_DEPENDS
    roslib
    any_msgs
    kindr_ros
    smb_msgs
    parameter_handler_ros
    robot_utils
    robot_utils_ros
    roco
    roscpp
    any_node
    joy_manager_msgs
    message_logger
    signal_logger
    signal_logger_ros
    signal_logger_std
    notification
#    rocoma_ros
    smb_common
    smb_model
    smb_ros
    smb_description
    smb_description_ros
    smb_roco
    signal_handler
    cosmo_ros
    param_io
    rocoma_ros
  DEPENDS
    Boost
)

###########
## Build ##
###########

include_directories(include)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${kindr_INCLUDE_DIRS})


add_library(smb_highlevel_controller
  src/SmbHighLevelController.cpp
)
add_dependencies(smb_highlevel_controller
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(smb_highlevel_controller
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(smb_highlevel_controller_node
  src/smb_highlevel_controller_node.cpp
)
add_dependencies(smb_highlevel_controller_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(smb_highlevel_controller_node
  smb_highlevel_controller
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


#############
## Install ##
#############

#install(TARGETS smb_highlevel_controller smb_highlevel_controller_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)
#
#install(DIRECTORY launch config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
