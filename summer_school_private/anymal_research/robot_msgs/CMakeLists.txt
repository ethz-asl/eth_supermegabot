cmake_minimum_required(VERSION 2.8.3)
project(robot_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
  sensor_msgs
)

add_message_files(
  FILES
    Contact.msg
    Contacts.msg
    RobotState.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
    sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    std_msgs
    sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
