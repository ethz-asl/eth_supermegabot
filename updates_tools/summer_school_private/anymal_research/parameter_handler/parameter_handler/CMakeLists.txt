cmake_minimum_required(VERSION 2.8.3)
project(parameter_handler)

add_definitions(-std=c++11 -Wall -Werror)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_logger
  tinyxml_tools
)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES parameter_handler
  CATKIN_DEPENDS message_logger tinyxml_tools
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(parameter_handler
  src/parameter_handler.cpp
)

add_dependencies(parameter_handler ${catkin_EXPORTED_TARGETS})

target_link_libraries(parameter_handler
  ${catkin_LIBRARIES}
)

catkin_add_gtest(test_parameter_handler
  test/test_main.cpp
  test/Parameter_tests.cpp
)

add_dependencies(test_parameter_handler 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(test_parameter_handler
  parameter_handler
  ${catkin_LIBRARIES}
)

install(TARGETS parameter_handler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)