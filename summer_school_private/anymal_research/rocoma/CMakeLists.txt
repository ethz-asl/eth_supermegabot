cmake_minimum_required(VERSION 2.8.3)
project(rocoma)

find_package(Boost REQUIRED COMPONENTS thread chrono)
find_package(catkin REQUIRED
  COMPONENTS
    roco
    any_worker
    message_logger
    signal_logger
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roco
    any_worker
    message_logger
    signal_logger
  DEPENDS Boost
)

###########
## Build ##
###########

add_definitions(-std=c++11 -Wall -Werror)
add_definitions(-DMELO_MIN_SEVERITY=MELO_SEVERITY_INFO)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/ControllerManager.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
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
