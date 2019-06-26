cmake_minimum_required(VERSION 2.8.3)
project(collisions_bullet)

add_compile_options(-std=c++11 -Wall -Werror)

find_package(PkgConfig REQUIRED)
set(bullet_FOUND 0)
pkg_check_modules(bullet REQUIRED bullet)

find_package(catkin REQUIRED COMPONENTS
  collisions
  collisions_geometry
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include ${bullet_INCLUDE_DIRS}
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS collisions collisions_geometry
   DEPENDS bullet
)

###########
## Build ##
###########

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${bullet_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/conversions.cpp
  src/btCollisionObjectContainer.cpp
  src/ColliderManagerBullet.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${bullet_LIBRARIES}
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
