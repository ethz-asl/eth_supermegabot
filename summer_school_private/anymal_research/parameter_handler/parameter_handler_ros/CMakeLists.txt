cmake_minimum_required(VERSION 2.8.3)
project(parameter_handler_ros)

add_definitions(-std=c++11 -Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
                 parameter_handler_std
                 parameter_handler_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES parameter_handler_ros
  CATKIN_DEPENDS parameter_handler_std parameter_handler_msgs
#  DEPENDS system_lib
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(parameter_handler_ros
  src/ParameterHandlerRos.cpp
)

add_dependencies(parameter_handler_ros ${catkin_EXPORTED_TARGETS})

target_link_libraries(parameter_handler_ros
   ${catkin_LIBRARIES}
)

install(TARGETS parameter_handler_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
