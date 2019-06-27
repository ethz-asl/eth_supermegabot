cmake_minimum_required(VERSION 2.8.3)
project(smb_gazebo)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra")

set(PACKAGE_DEPENDENCIES
    anybot_gazebo_plugin
    param_io
    smb_common
    smb_description
    smb_description_ros
    smb_model
    smb_model_ros
    smb_msgs
    romo_std
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES}
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    smb_gazebo
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
  DEPENDS
    gazebo
    boost
)

add_library(
  smb_gazebo
  src/SmbGazeboPlugin.cpp
)
add_dependencies(
  smb_gazebo
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  smb_gazebo
  ${catkin_LIBRARIES}
)

install(TARGETS smb_gazebo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
