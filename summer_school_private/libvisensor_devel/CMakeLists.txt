#Thomas Schneider, 2/12/2013
#Pascal Gohl, 21/08/2014
cmake_minimum_required(VERSION 2.8.0)

############
# CATKIN STUFF
############
project(libvisensor)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)

if(NOT DEFINED DONT_USE_CATKIN)
find_package(catkin)
catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS
  LIBRARIES visensor
)
endif()

############
# SETTINGS
############
set(PKGNAME visensor)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

###########
# BUILD
###########
FILE(
  GLOB SRCS
  src/*.cpp
  src/communication_layers/*.cpp
  src/config/*.cpp
  src/helpers/*.cpp
  src/networking/*.cpp
  src/serial_bridge/*.cpp
  src/sensors/*.cpp
  src/synchronization/*.cpp
  src/visensor/*.cpp
  src/yaml-cpp/*.cpp
)
INCLUDE_DIRECTORIES("include")
include_directories(${EIGEN_INCLUDE_DIR})


#Release / debug build

ADD_DEFINITIONS (-DVISENSOR_EXPORT -fPIC -fmessage-length=0 -MMD -MP -Wall -pedantic -std=c++0x -fvisibility=hidden)
if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  ADD_DEFINITIONS (-march=native -O3)
endif(NOT DEFINED CMAKE_BUILD_TYPE)

# uncomment next line to log timestamps to track timesync
# add_definitions(-DUSE_TIMESYNC_LOGGING)

#shared library
ADD_LIBRARY(${PKGNAME} SHARED ${SRCS})
SET_TARGET_PROPERTIES(${PKGNAME} PROPERTIES OUTPUT_NAME ${PKGNAME})
if(APPLE)
	TARGET_LINK_LIBRARIES(${PKGNAME} boost_thread-mt boost_system-mt ssh2)
else(APPLE)
	TARGET_LINK_LIBRARIES(${PKGNAME} boost_thread boost_system ssh2)
endif(APPLE)

#static library
ADD_LIBRARY(${PKGNAME}_static STATIC ${SRCS})
SET_TARGET_PROPERTIES(${PKGNAME}_static PROPERTIES OUTPUT_NAME ${PKGNAME})
if(APPLE)
	TARGET_LINK_LIBRARIES(${PKGNAME}_static boost_thread-mt boost_system-mt ssh2 )
else(APPLE)
	TARGET_LINK_LIBRARIES(${PKGNAME}_static boost_thread boost_system ssh2 )
endif(APPLE)

###########
# INSTALL
###########
if(DEFINED DONT_USE_CATKIN)
  SET(LIBVISENSOR_INSTALL_PATH lib)
else()
  SET(LIBVISENSOR_INSTALL_PATH ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION})
endif()
  
#shared library
INSTALL(
  FILES ${LIBVISENSOR_INSTALL_PATH}/lib${PKGNAME}.so
  DESTINATION lib
  COMPONENT shared-lib
)

#static library + headers (-dev package)
INSTALL(
  FILES ${LIBVISENSOR_INSTALL_PATH}/lib${PKGNAME}.a
  DESTINATION lib
  COMPONENT static-lib
)

INSTALL(
  FILES include/visensor/visensor_api.hpp
    include/visensor/visensor_config.hpp
    include/visensor/visensor_constants.hpp
    include/visensor/visensor_datatypes.hpp
    include/visensor/visensor_exceptions.hpp
    include/visensor/visensor_version.hpp
    include/visensor/visensor.hpp
  DESTINATION include/visensor
  COMPONENT static-lib
)

INSTALL(
  FILES cmake/FindVISensor.cmake
  DESTINATION lib/visensor
  COMPONENT static-lib
)

INSTALL(
  FILES pkgconfig/visensor.pc
  DESTINATION share/pkgconfig
  COMPONENT
  static-lib
)

##########
# DOXYGEN
##########
find_package(Doxygen)
if(DOXYGEN_FOUND)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
COMMENT "Generating API documentation with Doxygen" VERBATIM
)
endif(DOXYGEN_FOUND)

############
# TESTS
############
if(NOT DEFINED DONT_USE_CATKIN)
  catkin_add_gtest(test_time_synchronizer test/time_synchronizer-test.cpp src/synchronization/time_synchronizer.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  catkin_add_gtest(visensor_configuration-test test/visensor_configuration-test.cpp src/networking/file_transfer.cpp src/config/visensor_configuration.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  TARGET_LINK_LIBRARIES(visensor_configuration-test ${PKGNAME}_static)
endif()
