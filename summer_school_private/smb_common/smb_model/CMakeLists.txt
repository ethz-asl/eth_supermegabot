# Project configuration
cmake_minimum_required (VERSION 2.8)

project(smb_model)

# Find Catkin
find_package(catkin REQUIRED
  COMPONENTS
    message_logger
    robot_utils
    romo
    romo_rbdl
    romo_std
    signal_logger
    romo_test
    smb_common
    smb_description
)

find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

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
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    message_logger
    robot_utils
    romo
    romo_rbdl
    romo_std
    signal_logger
    romo_test
    smb_common
    smb_description
  DEPENDS
    kindr
)

# Set output paths
add_definitions(-std=c++11)


include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${kindr_INCLUDE_DIRS})
include_directories(${urdf_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

# Build the Super Mega Bot Model library
add_library(${PROJECT_NAME}
    src/SmbModel.cpp
    src/SmbState.cpp
    src/common/rbdl_utils.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${urdf_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Build unit tests
catkin_add_gtest(test_${PROJECT_NAME}
    test/test_main.cpp
    test/romo_tests.cpp
)

add_dependencies(test_${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
