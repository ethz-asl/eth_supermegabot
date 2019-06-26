cmake_minimum_required(VERSION 2.8.3)
project(any_state_estimator_msgs)

set(PACKAGE_DEPENDENCIES
    geometry_msgs
    std_msgs
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES}
    message_generation
)

add_service_files(
  FILES
    ResetStateEstimator.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
    message_runtime
)
