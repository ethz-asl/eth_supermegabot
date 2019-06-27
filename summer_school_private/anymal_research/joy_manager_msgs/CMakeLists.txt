cmake_minimum_required(VERSION 2.8.3)
project(joy_manager_msgs)

add_definitions(-std=c++11 -Wall -Werror)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    message_runtime
    sensor_msgs
    std_msgs
)

add_message_files(
  FILES
    AnyJoy.msg
)
generate_messages(
  DEPENDENCIES
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_generation
    message_runtime
    sensor_msgs
    std_msgs
)
