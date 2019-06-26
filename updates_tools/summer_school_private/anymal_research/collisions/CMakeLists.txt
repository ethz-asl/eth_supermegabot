cmake_minimum_required(VERSION 2.8.3)
project(collisions)
add_compile_options(-std=c++11 -Wall -Werror)

find_package(catkin REQUIRED
  collisions_geometry
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(include
  ${EIGEN3_INCLUDE_DIR})
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/CollisionGroup.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

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
