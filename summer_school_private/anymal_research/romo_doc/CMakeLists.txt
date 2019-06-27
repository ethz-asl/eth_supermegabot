cmake_minimum_required(VERSION 2.8.3)
project(romo_doc)

#--------------------------#
# Documentation

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(Doxygen)

IF(DOXYGEN)
  find_package(catkin REQUIRED COMPONENTS
    romo
    romo_std
    romo_rbdl
  )

  catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS
      romo
      romo_std
      romo_rbdl
    DEPENDS doxygen
  )

  # Find romo for documentation
  find_path (SOURCE_ROMO_DIR NAMES romo/RobotModel.hpp PATHS ${romo_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_ROMO_DIR}")
    message (STATUS "Found romo at: ${SOURCE_ROMO_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find romo! ${SOURCE_ROMO_DIR}")
  endif()

  # Find romo_std for documentation
  find_path (SOURCE_ROMO_STD_DIR NAMES romo_std/common/containers.hpp PATHS ${romo_std_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_ROMO_STD_DIR}")
    message (STATUS "Found romo_std at: ${SOURCE_ROMO_STD_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find romo_std! ${SOURCE_ROMO_STD_DIR}")
  endif()

  # Find romo_rbdl for documentation
  find_path (SOURCE_ROMO_RBDL_DIR NAMES romo_rbdl/RobotModelRbdl.hpp PATHS ${romo_rbdl_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_ROMO_RBDL_DIR}")
    message (STATUS "Found romo_rbdl at: ${SOURCE_ROMO_RBDL_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find romo_rbdl! ${SOURCE_ROMO_RBDL_DIR}")
  endif()

  # Set some environment variables
  Set(DOCSOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  Set(DOXYGEN_IMAGE_DIRS "\"${CMAKE_CURRENT_SOURCE_DIR}/figures/\"" )
  Set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${SOURCE_ROMO_DIR}/../\" \"${SOURCE_ROMO_STD_DIR}/../\" \"${SOURCE_ROMO_RBDL_DIR}/../\" \"${CMAKE_CURRENT_SOURCE_DIR}/\" ")
  Set(HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
  MAKE_DIRECTORY(${HTML_DIR})

  # Add configure file
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.config.in ${DOCSOURCE_DIR}/doxygen.config IMMEDIATE)

  # Build doxygen
  execute_process(COMMAND ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config)
  #ADD_CUSTOM_TARGET(doc ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config)

  # Unset cache files
  unset(${SOURCE_ROMO_DIR} CACHE)
  unset(${SOURCE_ROMO_STD_DIR} CACHE)
  unset(${SOURCE_ROMO_RBDL_DIR} CACHE)

ELSE(DOXYGEN)
  message(FATAL_ERROR "Doxygen not found")
ENDIF(DOXYGEN)
