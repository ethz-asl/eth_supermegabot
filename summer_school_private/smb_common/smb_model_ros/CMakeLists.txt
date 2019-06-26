# Project configuration
cmake_minimum_required (VERSION 2.8)

project(smb_model_ros)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
    smb_description
    smb_model
    smb_msgs
    kindr_ros
)

find_package(Eigen3 REQUIRED)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

# Declare this project as a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES smb_model_ros
  CATKIN_DEPENDS 
    smb_description 
    smb_model
    smb_msgs 
    kindr_ros 
    kindr
  DEPENDS
)

# Set output paths
add_definitions(-std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${kindr_INCLUDE_DIRS}
)

# Build the Super Mega Bot Model library
add_library(${PROJECT_NAME} 
  src/SmbStateRosConverter.cpp
  src/SmbModelRosConverter.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
