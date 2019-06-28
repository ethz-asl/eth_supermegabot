cmake_minimum_required(VERSION 2.8.3)
project(rocoma_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  ActiveControllerName.msg
  ControllerManagerState.msg
  EmergencyStop.msg
)

add_service_files(
  FILES
  GetActiveController.srv
  GetAvailableControllers.srv
  SwitchController.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
 CATKIN_DEPENDS
  message_runtime
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
