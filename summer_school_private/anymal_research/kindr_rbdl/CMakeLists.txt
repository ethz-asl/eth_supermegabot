cmake_minimum_required(VERSION 2.8.3)
project(kindr_rbdl)

add_definitions(-std=c++11 -Wall -Werror)

find_package(catkin REQUIRED
  COMPONENTS
      kindr
      any_rbdl
)


catkin_package(
  CATKIN_DEPENDS kindr any_rbdl
  DEPENDS gazebo
)

include_directories(${catkin_INCLUDE_DIRS})


# Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
    test/test_main.cpp
    test/QuaternionTest.cpp
    test/SpatialTransformTest.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
endif()
