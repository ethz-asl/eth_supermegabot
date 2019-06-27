cmake_minimum_required(VERSION 2.8.3)
project(collisions_visualization)
add_compile_options(-std=c++11 -Wall -Werror)

find_package(catkin REQUIRED
  collisions_geometry
  collisions
  visualization_msgs
  std_msgs
  roscpp
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

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/geometry_to_marker.cpp
  src/CollisionsMarkerPublisher.cpp
  src/CollisionGeometryMarkerPublisher.cpp
  src/MarkerOptions.cpp
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

#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#
#  add_rostest_gtest(${PROJECT_NAME}-test
#    test/collisions_visualization.test
#    test/test_collisions_visualization.cpp
#    test/visual_test.cpp
#    test/test_declarations.cpp
#  )
#
#  if(TARGET ${PROJECT_NAME}-test)
#    target_link_libraries(${PROJECT_NAME}-test
#      ${PROJECT_NAME}
#      ${catkin_LIBRARIES}
#    )
#  endif()
#
#endif()
