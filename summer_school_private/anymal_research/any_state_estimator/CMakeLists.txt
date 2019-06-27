cmake_minimum_required(VERSION 2.8.3)
project(any_state_estimator)

add_definitions(-std=c++11 -Wall -Werror)

set(PACKAGE_DEPENDENCIES
    any_state_estimator_msgs
    message_logger
    romo
    cosmo
    cosmo_node
    cosmo_ros
    signal_logger
    signal_logger_ros
)

find_package(Boost REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
