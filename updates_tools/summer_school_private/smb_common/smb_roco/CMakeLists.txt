cmake_minimum_required(VERSION 2.8.3)
project(smb_roco)

find_package(catkin REQUIRED COMPONENTS
    roco
    romo_std
    message_logger
    robot_utils
    any_measurements
    any_measurements_ros
    smb_description
    smb_common
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roco
    romo_std
    message_logger
    robot_utils
    any_measurements
    any_measurements_ros
    smb_description
    smb_common
)

add_definitions("--std=c++11")

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
    src/RocoCommand.cpp
    src/RocoState.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)