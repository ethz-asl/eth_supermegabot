# Project configuration
cmake_minimum_required (VERSION 2.8)

project(robot_utils_ros)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
  robot_utils
  message_generation
  std_msgs
  std_utils
)

# Declare the message and service files to be built
add_message_files(
  FILES
    ForceCalibratorCommand.msg
    ForceCalibratorCommands.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    robot_utils
    message_runtime
    std_msgs
    std_utils
  DEPENDS
)

###########
## Build ##
###########

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_definitions(--std=c++11)

include_directories (
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/ConvertRosMessages.cpp
)
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
