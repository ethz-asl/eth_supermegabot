cmake_minimum_required(VERSION 2.8.3)
project(anybot_gazebo_plugin)

add_definitions(-std=c++11 -Wall -Werror)

find_package(catkin REQUIRED
  COMPONENTS
    any_gazebo_msgs
    any_hector_gazebo_plugins
    any_measurements_ros
    any_msgs
    gazebo_plugins
    gazebo_ros
    gazebo_worlds
    geometry_msgs
    roscpp
    urdf
    xacro
    robot_utils
    std_utils
    romo_std
    message_logger
    cosmo
    cosmo_ros
    signal_logger
    signal_logger_ros
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
    ${GAZEBO_INCLUDE_DIRS}
  CATKIN_DEPENDS
    any_gazebo_msgs
    any_hector_gazebo_plugins
    any_measurements_ros
    any_msgs
    gazebo_plugins
    gazebo_ros
    gazebo_worlds
    geometry_msgs
    roscpp
    urdf
    xacro
    robot_utils
    std_utils
    romo_std
    message_logger
    cosmo
    cosmo_ros
    signal_logger
    signal_logger_ros
  DEPENDS
    Boost
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
