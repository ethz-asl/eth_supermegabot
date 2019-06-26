cmake_minimum_required(VERSION 2.8.3)
project(signal_logger_example)

add_definitions(-std=c++11 -Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
  any_node
  signal_logger
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES signal_logger_example
  CATKIN_DEPENDS any_node signal_logger
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/SignalLoggerExample.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
