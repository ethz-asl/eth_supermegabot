cmake_minimum_required(VERSION 2.8.3)
project(smb_state_estimator)

add_definitions(-std=c++11)

set(PACKAGE_DEPENDENCIES
    any_state_estimator
    roscpp
    any_node
    robot_utils
    robot_utils_ros
    any_msgs
    std_srvs
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    notification
    message_logger
    any_measurements
    any_measurements_ros
    cosmo_ros
    signal_logger
    signal_logger_ros
    signal_logger_std
    smb_description_ros
    confusion
    smb_confusor
)

find_package(catkin REQUIRED
    COMPONENTS
    ${PACKAGE_DEPENDENCIES})

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Ceres REQUIRED)

# Attempt to find catkinized kindr
#find_package(kindr QUIET)
#if(NOT kindr_FOUND)
#  # Attempt to find package-based kindr
#  find_package(PkgConfig REQUIRED)
#  pkg_check_modules(kindr kindr REQUIRED)
#endif()

catkin_package(
    INCLUDE_DIRS
      include
      ${EIGEN3_INCLUDE_DIR}
    LIBRARIES
      ${PROJECT_NAME}
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

add_library(${PROJECT_NAME}
    src/SmbStateEstimator.cpp)

#add_library(BmmPropagator
#    src/BmmPropagator.cpp)
#
#add_library(SmbPropagator
#    src/SmbPropagator.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(
#  ${PROJECT_NAME}
#  ${PROJECT_NAME}_generate_messages_cpp
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS})

target_link_libraries(
    ${PROJECT_NAME}
#    BmmPropagator
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES})

add_executable(${PROJECT_NAME}_node
    src/smb_state_estimator_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES})

#add_executable(smb_confusor src/SmbConFusor.cpp)
#target_link_libraries(smb_confusor ${catkin_LIBRARIES} ${CERES_LIBRARIES})

#add_executable(smb_propagator_tester src/SmbPropagatorTester.cpp)
#target_link_libraries(smb_propagator_tester SmbPropagator ${catkin_LIBRARIES} ${CERES_LIBRARIES})

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
