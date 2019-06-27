cmake_minimum_required(VERSION 2.8.3)
project(signal_logger)

include(cmake/${PROJECT_NAME}-extras.cmake)

# List of necessary dependencies
set(SILO_CATKIN_PACKAGES
  signal_logger_core
  signal_logger_std
)

# Append optional dependencies
if(SILO_USE_ROS)
  list(APPEND SILO_CATKIN_PACKAGES signal_logger_ros)
endif(SILO_USE_ROS)

# Find depencendies
find_package(catkin REQUIRED COMPONENTS
  ${SILO_CATKIN_PACKAGES}
)

# Configure catkin package
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS ${SILO_CATKIN_PACKAGES}
 CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
)

###########
## Build ##
###########
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_definitions(-std=c++11 -Wall -Werror)
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
  src/signal_logger.cpp
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
