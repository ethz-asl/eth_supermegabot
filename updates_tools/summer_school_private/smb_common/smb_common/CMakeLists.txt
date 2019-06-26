cmake_minimum_required(VERSION 2.8.3)
project(smb_common)

set(PACKAGE_DEPENDENCIES
  any_measurements
  any_measurements_ros
  std_utils
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    ${PACKAGE_DEPENDENCIES}
)

add_definitions("--std=c++11 -Wall")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/SmbWheelCommand.cpp
  src/SmbWheelReading.cpp
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
