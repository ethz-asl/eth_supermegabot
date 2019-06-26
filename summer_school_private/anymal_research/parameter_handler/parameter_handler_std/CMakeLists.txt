cmake_minimum_required(VERSION 2.8.3)
project(parameter_handler_std)

add_definitions(-std=c++11 -Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
                message_logger
                tinyxml_tools
                parameter_handler
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES parameter_handler_std
  CATKIN_DEPENDS message_logger tinyxml_tools parameter_handler
#  DEPENDS system_lib
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(parameter_handler_std
  src/ParameterHandlerStd.cpp
)

add_dependencies(parameter_handler_std ${catkin_EXPORTED_TARGETS})

target_link_libraries(parameter_handler_std
   ${catkin_LIBRARIES}
)

catkin_add_gtest(test_parameter_handler_std 
    test/test_main.cpp
    test/parameter_handler_std_tests.cpp
)
add_dependencies(test_parameter_handler_std ${catkin_EXPORTED_TARGETS})


target_link_libraries(test_parameter_handler_std
    parameter_handler_std 
    ${catkin_LIBRARIES}
)

install(TARGETS parameter_handler_std
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)