cmake_minimum_required(VERSION 2.8.3)
project(smb_description_ros)

find_package(catkin REQUIRED COMPONENTS
  smb_description
    smb_msgs
    smb_ros
)

catkin_package(
  INCLUDE_DIRS include
 # LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    smb_description
    smb_msgs
    smb_ros
    smb_model_ros
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
