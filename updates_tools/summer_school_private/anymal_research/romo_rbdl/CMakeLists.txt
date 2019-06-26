# Project configuration
cmake_minimum_required (VERSION 2.8)
project(romo_rbdl)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
  romo
  romo_std
  message_logger
  std_utils
  any_rbdl
  urdf
  collisions_urdf
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    romo 
    romo_std 
    message_logger
    std_utils
    any_rbdl 
    urdf
    collisions_urdf
)

###########
## Build ##
###########
add_definitions(-std=c++11 -Wall -Werror)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# Build the Robot Model library
add_library(${PROJECT_NAME}
  src/rbdl_kinematics.cpp
  src/rbdl_utils.cpp
)

add_dependencies(${PROJECT_NAME} 
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

# Build unit tests
#catkin_add_gtest(test_romo 
#)
#add_dependencies(test_romo ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_romo
#)
