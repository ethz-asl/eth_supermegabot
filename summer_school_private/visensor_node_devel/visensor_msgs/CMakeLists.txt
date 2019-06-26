cmake_minimum_required(VERSION 2.8.3)
project(visensor_msgs)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
)

add_message_files(
  DIRECTORY msg
  FILES visensor_imu.msg 
  visensor_time_host.msg
  visensor_trigger.msg
  visensor_calibration.msg
)

add_service_files(
    FILES
    visensor_calibration_service.srv
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs
)

