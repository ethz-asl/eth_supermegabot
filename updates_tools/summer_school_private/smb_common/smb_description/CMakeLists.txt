cmake_minimum_required(VERSION 2.8.3)
project(smb_description)

find_package(catkin REQUIRED COMPONENTS
    any_description
    any_measurements
    geometry_msgs
    robot_utils
    romo
    sensor_msgs
    std_utils
    xacro
		smb_common
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    any_description
    any_measurements
    geometry_msgs
    robot_utils
    romo
    sensor_msgs
    std_utils
    xacro
		smb_common
)

add_definitions("--std=c++11")

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
	src/SmbDescription.cpp
)

#add_library(${PROJECT_NAME} SHARED
#    src/topology_conversions.cpp
#    src/topology.cpp
#)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch rviz urdf resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
