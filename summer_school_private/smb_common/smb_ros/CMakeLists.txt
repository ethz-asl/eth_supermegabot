cmake_minimum_required(VERSION 2.8.3)
project(smb_ros)

find_package(catkin REQUIRED COMPONENTS
  smb_description
    smb_model
    smb_common
    smb_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    smb_model
    smb_common
    smb_msgs
)

add_definitions("--std=c++11")

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})


#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
