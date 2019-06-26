cmake_minimum_required(VERSION 2.8.3)
project(signal_logger_core)

include(cmake/${PROJECT_NAME}-extras.cmake)

# Find dependencies
find_package(catkin REQUIRED COMPONENTS message_logger)
find_package(Eigen3 REQUIRED)

if(SILO_USE_KINDR)
  # Attempt to find catkinized kindr
	find_package(kindr QUIET)
	if(NOT kindr_FOUND)
	  # Attempt to find package-based kindr
	  find_package(PkgConfig REQUIRED)
	  pkg_check_modules(kindr kindr REQUIRED)
	endif(NOT kindr_FOUND)
endif(SILO_USE_KINDR)

# Configure catkin package
catkin_package(
 INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS message_logger
 DEPENDS kindr
 CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
)

###########
## Build ##
###########
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_definitions(-std=c++11 -Wall -Werror)
add_definitions(-DMELO_MIN_SEVERITY=MELO_SEVERITY_INFO)
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
if(SILO_USE_KINDR)
	include_directories(${kindr_INCLUDE_DIRS})
endif()

add_library(${PROJECT_NAME} SHARED
  src/SignalLoggerBase.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    yaml-cpp
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
