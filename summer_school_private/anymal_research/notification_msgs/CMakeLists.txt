cmake_minimum_required(VERSION 2.8.3)
project(notification_msgs)

add_definitions(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    message_runtime
    std_msgs
)

add_message_files(
  FILES
    Notification.msg
)
generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_generation
    message_runtime
    std_msgs
)

catkin_package(
#  INCLUDE_DIRS
#    include
#  LIBRARIES
#    notification
  CATKIN_DEPENDS
    message_generation
    message_runtime
    std_msgs
#  DEPENDS
#    system_lib
)

