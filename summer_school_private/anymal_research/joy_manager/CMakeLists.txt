cmake_minimum_required(VERSION 2.8.3)
project(joy_manager)

add_definitions(-std=c++11 -Wall -Werror)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(
  catkin REQUIRED
  COMPONENTS
    any_measurements
    any_measurements_ros
    any_node
    cosmo_ros
    geometry_msgs
    joy_manager_msgs
    kindr_ros
    notification
    pluginlib
    roscpp
    sensor_msgs
    std_msgs
)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    joy_manager
  CATKIN_DEPENDS
    any_measurements
    any_measurements_ros
    any_node
    cosmo_ros
    geometry_msgs
    joy_manager_msgs
    kindr_ros
    notification
    pluginlib
    roscpp
    sensor_msgs
    std_msgs
  DEPENDS
    Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${kindr_INCLUDE_DIRS}
)

add_library(
  joy_manager
    src/JoyManager.cpp
    src/Joystick.cpp
    src/TwistOutput.cpp
    src/PoseOutput.cpp
    src/JoyOutput.cpp
    src/WrenchOutput.cpp
)

add_dependencies(
  joy_manager
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  joy_manager
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_executable(
  joy_manager_node
    src/joy_manager_node.cpp
)

add_dependencies(
  joy_manager_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  joy_manager_node
    joy_manager
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  ## Add gtest based cpp test target and link libraries
  add_rostest_gtest(${PROJECT_NAME}-test
    test/joy_manager.test
    test/test_main.cpp
    test/JoyManagerTest.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test
    ${catkin_LIBRARIES}
    pthread
  )
endif()

#############
## Install ##
#############

install(TARGETS joy_manager joy_manager_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
