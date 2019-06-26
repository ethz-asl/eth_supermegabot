cmake_minimum_required(VERSION 2.8.3)
project(parameter_handler_doc)

#--------------------------#
# Documentation

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(Doxygen)

IF(DOXYGEN)
  find_package(catkin REQUIRED COMPONENTS
    parameter_handler
    parameter_handler_ros
    parameter_handler_std
    parameter_handler_msgs
    rqt_parameters
  )

  catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS
      parameter_handler
      parameter_handler_ros
      parameter_handler_std
      parameter_handler_msgs
      rqt_parameters
    DEPENDS
  )

  # Find parameter_handler for documentation
  find_path (SOURCE_PARA__DIR NAMES parameter_handler/parameter_handler.hpp PATHS ${parameter_handler_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_PARA__DIR}")
    message (STATUS "Found parameter handler at: ${SOURCE_PARA__DIR}")
  else()
    message (FATAL_ERROR "Couldn't find parameter handler! ${SOURCE_PARA__DIR}")
  endif()

  # Find parameter_handler_msgs for documentation
  find_path (SOURCE_PARA_MSGS_DIR NAMES parameter_handler_msgs/GetParameterList.h PATHS ${parameter_handler_msgs_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_PARA_MSGS_DIR}")
    message (STATUS "Found parameter handler msgs at: ${SOURCE_PARA_MSGS_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find parameter handler msgs! ${SOURCE_PARA_MSGS_DIR}")
  endif()

  # Find parameter_handler_ros for documentation
  find_path (SOURCE_PARA_ROS_DIR NAMES parameter_handler_ros/ParameterHandlerRos.hpp PATHS ${parameter_handler_ros_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_PARA_ROS_DIR}")
    message (STATUS "Found parameter handler ros at: ${SOURCE_PARA_ROS_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find parameter handler ros! ${SOURCE_PARA_ROS_DIR}")
  endif()

  # Find parameter_handler_std for documentation
  find_path (SOURCE_PARA_STD_DIR NAMES parameter_handler_std/ParameterHandlerStd.hpp PATHS ${parameter_handler_std_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_PARA_STD_DIR}")
    message (STATUS "Found parameter handler std at: ${SOURCE_PARA_STD_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find parameter handler std! ${SOURCE_PARA_STD_DIR}")
  endif()

  # Find rqt_parameters for documentation
  find_path (SOURCE_PARA_RQT_DIR NAMES rqt_parameters/ParameterBase.hpp PATHS ${rqt_parameters_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_PARA_RQT_DIR}")
    message (STATUS "Found parameter handler rqt at: ${SOURCE_PARA_RQT_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find parameter handler rqt! ${SOURCE_PARA_RQT_DIR}")
  endif()

  # Set some environment variables
  Set(DOCSOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  Set(DOXYGEN_IMAGE_DIRS "\"${CMAKE_CURRENT_SOURCE_DIR}/figures/\"" )
  Set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${SOURCE_PARA__DIR}/../\" \"${SOURCE_PARA_ROS_DIR}/../\" \"${SOURCE_PARA_STD_DIR}/../\" \"${SOURCE_PARA_RQT_DIR}/../\" \"${CMAKE_CURRENT_SOURCE_DIR}/\" ")
  Set(HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
  MAKE_DIRECTORY(${HTML_DIR})

  # Add configure file
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.config.in ${DOCSOURCE_DIR}/doxygen.config IMMEDIATE)

  # Build doxygen
  execute_process(COMMAND ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config)
  #ADD_CUSTOM_TARGET(doc ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config)

  # Unset cache files
  unset(${DOXYGEN_SOURCE_DIRS} CACHE)
  unset(${SOURCE_PARA__DIR} CACHE)
  unset(${SOURCE_PARA_MSGS_DIR} CACHE)
  unset(${SOURCE_PARA_ROS_DIR} CACHE)
  unset(${SOURCE_PARA_STD_DIR} CACHE)
  unset(${SOURCE_PARA_RQT_DIR} CACHE)

ELSE(DOXYGEN)
  message(FATAL_ERROR "Doxygen not found")
ENDIF(DOXYGEN)
