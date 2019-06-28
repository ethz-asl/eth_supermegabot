cmake_minimum_required(VERSION 2.8.3)
project(visensor_node)

SET(EXPERT_MODE TRUE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  message_generation
  geometry_msgs
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
  camera_info_manager
  dynamic_reconfigure
  cmake_modules
  visensor_msgs
)

# check libvisensor version, flags not used later
find_package(libvisensor 2.3.0 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${libvisensor_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc calib3d)

if(EXPERT_MODE)
	add_definitions(-DEXPERT_MODE)
	generate_dynamic_reconfigure_options(cfg/Driver.cfg)
else(EXPERT_MODE)
    generate_dynamic_reconfigure_options(cfg/DriverMinimal.cfg)
endif(EXPERT_MODE)

if(NOT DEFINED CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif(NOT DEFINED CMAKE_BUILD_TYPE)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++0x -D__STRICT_ANSI__")

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS
  roscpp 
  sensor_msgs 
  cv_bridge 
  std_msgs 
  image_transport 
  camera_info_manager 
)

include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

#build and add libvisensor system library dependency
add_executable(visensor_node src/visensor_node.cpp src/visensor.cpp )

add_dependencies(visensor_node ${${PROJECT_NAME}_EXPORTED_TARGETS}} ${PROJECT_NAME}_gencfg)
target_link_libraries(visensor_node ${libvisensor_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
