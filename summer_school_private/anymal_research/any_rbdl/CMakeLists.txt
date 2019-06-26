cmake_minimum_required(VERSION 2.8.3)
if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()

project(any_rbdl)

add_definitions(-std=c++11)

set(ANY_RBDL_VERSION_MAJOR 2)
set(ANY_RBDL_VERSION_MINOR 6)
set(ANY_RBDL_VERSION_PATCH 0)
set(ANY_RBDL_VERSION
  ${ANY_RBDL_VERSION_MAJOR}.${ANY_RBDL_VERSION_MINOR}.${ANY_RBDL_VERSION_PATCH}
)
set(ANY_RBDL_SO_VERSION
  ${ANY_RBDL_VERSION_MAJOR}.${ANY_RBDL_VERSION_MINOR}.${ANY_RBDL_VERSION_PATCH}
)
set(PROJECT_VERSION ${ANY_RBDL_VERSION})

# Find packages
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

# Options
option(BUILD_STATIC "Build statically linked library (otherwise dynamiclly linked)" OFF)
option(ENABLE_LOGGING "Enable logging (warning: major impact on performance!)" OFF)
option(STORE_VERSION "Enable storing of version information in the library (requires build from valid repository)" OFF)
option(BUILD_ADDON_URDFREADER "Build the (experimental) urdf reader" ON)
option(BUILD_ADDON_BENCHMARK "Build the benchmarking tool" OFF)
option(BUILD_ADDON_LUAMODEL "Build the lua model reader" OFF)

IF (STORE_VERSION)
  # Set versioning information that can be queried during runtime
  EXEC_PROGRAM("hg" ${CMAKE_CURRENT_SOURCE_DIR} ARGS "id -i"
	OUTPUT_VARIABLE ANY_RBDL_BUILD_REVISION)
  EXEC_PROGRAM("hg" ${CMAKE_CURRENT_SOURCE_DIR} ARGS "branch"
	OUTPUT_VARIABLE ANY_RBDL_BUILD_BRANCH)
  set(ANY_RBDL_BUILD_TYPE ${CMAKE_BUILD_TYPE})
ELSE (STORE_VERSION)
  set(ANY_RBDL_BUILD_REVISION "unknown")
  set(ANY_RBDL_BUILD_BRANCH "unknown")
  set(ANY_RBDL_BUILD_TYPE "unknown")
ENDIF (STORE_VERSION)

# generate config header file
set(devel_pkg_inc ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME})
file(MAKE_DIRECTORY ${devel_pkg_inc})
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/include/any_rbdl/rbdl_config.h.cmake"
  "${devel_pkg_inc}/rbdl_config.h"
)

set(ANY_RBDL_INCLUDE_DIRS
  include
  ${CATKIN_DEVEL_PREFIX}/include
)
set(ANY_RBDL_LIBRARIES ${PROJECT_NAME})

IF (BUILD_ADDON_URDFREADER)
  list(APPEND ANY_RBDL_INCLUDE_DIRS addons/urdfreader/include)
  list(APPEND ANY_RBDL_LIBRARIES any_rbdl_urdfreader)
  list(APPEND ANY_RBDL_DEPENDENCIES)
ENDIF (BUILD_ADDON_URDFREADER)

IF (BUILD_ADDON_LUAMODEL)
  list(APPEND ANY_RBDL_INCLUDE_DIRS addons/luamodel/include)
  list(APPEND ANY_RBDL_LIBRARIES any_rbdl_luamodel)
ENDIF (BUILD_ADDON_LUAMODEL)




# Declare this project as a catkin package
catkin_package(
  INCLUDE_DIRS
	${ANY_RBDL_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${ANY_RBDL_LIBRARIES}
  CATKIN_DEPENDS
  DEPENDS ${ANY_RBDL_DEPENDENCIES}
)

include_directories(
  ${ANY_RBDL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Source files
set(ANY_RBDL_SOURCES
  src/rbdl_version.cc
  src/rbdl_mathutils.cc
  src/rbdl_utils.cc
  src/Contacts.cc
  src/Dynamics.cc
  src/Logging.cc
  src/Joint.cc
  src/Model.cc
  src/Kinematics.cc
)

# Library
add_library(${PROJECT_NAME}
  ${ANY_RBDL_SOURCES}
)

set_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES
  VERSION ${ANY_RBDL_VERSION}
  SOVERSION ${ANY_RBDL_SO_VERSION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Addons
IF (BUILD_ADDON_URDFREADER)
  add_subdirectory(addons/urdfreader)
ENDIF (BUILD_ADDON_URDFREADER)

IF (BUILD_ADDON_BENCHMARK)
  add_subdirectory(addons/benchmark)
ENDIF (BUILD_ADDON_BENCHMARK)

IF (BUILD_ADDON_LUAMODEL)
  add_subdirectory(addons/luamodel)
ENDIF (BUILD_ADDON_LUAMODEL)

IF (BUILD_TESTS)
 add_subdirectory(tests)
ENDIF (BUILD_TESTS)

if(CATKIN_ENABLE_TESTING)

find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

include_directories(
  ${kindr_INCLUDE_DIRS}
)

SET ( TESTS_SRCS
  tests/main.cc
  tests/MathTests.cc
  tests/SpatialAlgebraTests.cc
  tests/MultiDofTests.cc
  tests/KinematicsTests.cc
  tests/BodyTests.cc
  tests/ModelTests.cc
  tests/FloatingBaseTests.cc
  tests/CalcVelocitiesTests.cc
  tests/CalcAccelerationsTests.cc
  tests/DynamicsTests.cc
  tests/InverseDynamicsTests.cc
  tests/CompositeRigidBodyTests.cc
  tests/ImpulsesTests.cc
  tests/TwolegModelTests.cc
  tests/ContactsTests.cc
  tests/UtilsTests.cc
  tests/SparseFactorizationTests.cc
)

# Build unit tests
catkin_add_gtest(test_${PROJECT_NAME}
  ${TESTS_SRCS}
)
add_dependencies(test_${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_${PROJECT_NAME}
	${PROJECT_NAME}
)
endif()

# Install
file(GLOB headers
  ${devel_pkg_inc}/rbdl_config.h
)

install(FILES ${headers}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "SimpleMath" EXCLUDE
  PATTERN "*.cmake" EXCLUDE
)
