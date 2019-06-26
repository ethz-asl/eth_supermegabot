cmake_minimum_required(VERSION 2.8.3)
project(joy_interface)

add_definitions(-std=c++11 -Wall -Werror)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(
  catkin REQUIRED
  COMPONENTS
    any_node
    joy
    joy_manager_msgs
    notification
    sensor_msgs
    pluginlib
    roscpp
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    joy_interface
    feedback
  CATKIN_DEPENDS
    any_node
    joy
    joy_manager_msgs
    notification
    sensor_msgs
    pluginlib
    roscpp
  DEPENDS
    Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(
  joy_interface
    src/JoyInterface.cpp
)

add_dependencies(
  joy_interface
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  joy_interface
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_executable(
  joy_interface_node
    src/joy_interface_node.cpp
)

add_dependencies(
  joy_interface_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  joy_interface_node
    joy_interface
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)


add_library(
  feedback
    src/Feedback.cpp
)

target_link_libraries(
  feedback
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_executable(
  feedback_node
    src/feedback_node.cpp
)

target_link_libraries(
    feedback_node
    feedback
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS joy_interface joy_interface_node feedback feedback_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY
  config
  launch
  udev
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
