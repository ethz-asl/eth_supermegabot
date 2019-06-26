cmake_minimum_required(VERSION 2.8.3)
project(smb_opc)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set(PACKAGE_DEPENDENCIES
    smb_description
    #joy_smb_highlevel_controller
    kindr_rviz_plugins
    multi_dof_joint_trajectory_rviz_plugins
    #smb_tf_publisher
    rqt_gazebo_interface
    #smb_rqt_controlmanager
    rqt_joypad
    rqt_parameters
    rqt_signal_logger
    rviz_gazebo_interaction
    rviz_world_loader
    rocoma_rqt
    #smb_joy_module
    #smb_joy_interface_module
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#TODO Is this required?
#install(
#  PROGRAMS
#    bin/rqt_smb_opc.sh
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING
  PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
