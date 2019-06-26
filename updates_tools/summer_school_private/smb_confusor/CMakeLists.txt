cmake_minimum_required(VERSION 2.8.3)
project(smb_confusor)

add_definitions(-std=c++11)

set(PACKAGE_DEPENDENCIES
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    tf
    confusion
    )

find_package(catkin REQUIRED
    COMPONENTS
    ${PACKAGE_DEPENDENCIES})

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
    INCLUDE_DIRS
      include
      ${EIGEN3_INCLUDE_DIR}
    LIBRARIES
      BmmPropagator SmbPropagator
    CATKIN_DEPENDS
      ${PACKAGE_DEPENDENCIES}

    DEPENDS
      Boost)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIR}
)

add_library(BmmPropagator
    src/BmmPropagator.cpp)

add_library(SmbPropagator
    src/SmbPropagator.cpp)

add_executable(smb_confusor src/SmbConFusor.cpp)
target_link_libraries(smb_confusor BmmPropagator ${catkin_LIBRARIES} ${CERES_LIBRARIES})

add_executable(smb_propagator_tester src/SmbPropagatorTester.cpp)
target_link_libraries(smb_propagator_tester SmbPropagator ${catkin_LIBRARIES} ${CERES_LIBRARIES})

#############
## Testing ##
#############
#catkin_add_gtest(test_anymal_state_estimator
#    test/test_main.cpp
#    )
#
#target_link_libraries(test_anymal_state_estimator
#    ${PROJECT_NAME}
#    ${catkin_LIBRARIES}
#    )
#
#install(TARGETS ${PROJECT_NAME}
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#    )
#
#install(DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    PATTERN ".svn" EXCLUDE
#    )
#
#install(DIRECTORY config
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#    )
