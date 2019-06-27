#-------------------------------------------------------------------------------
# Autonomous Systems Lab
# ETH Zurich
#
# Author(s): Christian Gehring
# email    : cgehring@anybotics.com
# Date     : 1-Dec-2014
#
#-------------------------------------------------------------------------------
cmake_minimum_required (VERSION 2.8)
project(roco)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS message_logger)
find_package(Boost REQUIRED)

# Catkin package definition
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roco
  CATKIN_DEPENDS
    message_logger
  DEPENDS
    Boost
)

# Build
add_definitions(-std=c++11 -Wall -Werror)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(roco SHARED
    src/time/Time.cpp
    src/time/TimeStd.cpp
    src/workers/Worker.cpp
)
set_property(TARGET roco PROPERTY POSITION_INDEPENDENT_CODE ON)

target_link_libraries(roco
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Add Doxygen documentation
if (BUILD_DOC)
add_subdirectory(doc/doxygen)
endif (BUILD_DOC)

if (BUILD_EXAMPLE)
add_subdirectory(example example)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
