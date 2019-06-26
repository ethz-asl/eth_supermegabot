cmake_minimum_required(VERSION 2.8.3)
project(signal_logger_doc)

#--------------------------#
# Documentation

add_definitions(-std=c++11)

find_package(Doxygen)

IF(DOXYGEN)
  find_package(catkin REQUIRED COMPONENTS
    signal_logger
    signal_logger_ros
    signal_logger_std
    signal_logger_msgs
    rqt_signal_logger
  )

  catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS
      signal_logger
      signal_logger_ros
      signal_logger_std
      signal_logger_msgs
      rqt_signal_logger
    DEPENDS
  )

  # Find signal_logger for documentation
  find_path (SOURCE_SILO_DIR NAMES signal_logger/signal_logger.hpp PATHS ${signal_logger_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_SILO_DIR}")
    message (STATUS "Found signal logger at: ${SOURCE_SILO_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find signal logger! ${SOURCE_SILO_DIR}")
  endif()

  # Find signal_logger_core for documentation
  find_path (SOURCE_SILOCORE_DIR NAMES signal_logger_core/SignalLoggerBase.hpp PATHS ${signal_logger_core_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_SILOCORE_DIR}")
    message (STATUS "Found signal logger core at: ${SOURCE_SILOCORE_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find signal logger core! ${SOURCE_SILOCORE_DIR}")
  endif()

  # Find signal_logger_msgs for documentation
  find_path (SOURCE_SILOMSGS_DIR NAMES signal_logger_msgs/LogElement.h PATHS ${signal_logger_msgs_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_SILOMSGS_DIR}")
    message (STATUS "Found signal logger msgs at: ${SOURCE_SILOMSGS_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find signal logger msgs! ${SOURCE_SILOMSGS_DIR}")
  endif()

  # Find signal_logger_ros for documentation
  find_path (SOURCE_SILOROS_DIR NAMES signal_logger_ros/SignalLoggerRos.hpp PATHS ${signal_logger_ros_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_SILOROS_DIR}")
    message (STATUS "Found signal logger ros at: ${SOURCE_SILOROS_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find signal logger ros! ${SOURCE_SILOROS_DIR}")
  endif()

  # Find signal_logger_std for documentation
  find_path (SOURCE_SILOSTD_DIR NAMES signal_logger_std/SignalLoggerStd.hpp PATHS ${signal_logger_std_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_SILOSTD_DIR}")
    message (STATUS "Found signal logger std at: ${SOURCE_SILOSTD_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find signal logger std! ${SOURCE_SILOSTD_DIR}")
  endif()

  # Find rqt_signal_logger for documentation
  find_path (SOURCE_SILORQT_DIR NAMES rqt_signal_logger/LogElement.hpp PATHS ${rqt_signal_logger_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_SILORQT_DIR}")
    message (STATUS "Found signal logger rqt at: ${SOURCE_SILORQT_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find signal logger rqt! ${SOURCE_SILORQT_DIR}")
  endif()

  # Set some environment variables
  Set(DOCSOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  Set(DOXYGEN_IMAGE_DIRS "\"${CMAKE_CURRENT_SOURCE_DIR}/figures/\"" )
  Set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${SOURCE_SILO_DIR}/../\"  \"${SOURCE_SILOCORE_DIR}/../\" \"${SOURCE_SILOMSGS_DIR}/../\" \"${SOURCE_SILOROS_DIR}/../\" \"${SOURCE_SILOSTD_DIR}/../\" \"${SOURCE_SILORQT_DIR}/../\" \"${CMAKE_CURRENT_SOURCE_DIR}/\" ")
  Set(HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
  MAKE_DIRECTORY(${HTML_DIR})

  # Add configure file
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.config.in ${DOCSOURCE_DIR}/doxygen.config IMMEDIATE)

  # Build doxygen
  execute_process(COMMAND ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config)
  #ADD_CUSTOM_TARGET(doc ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config)

  # Unset cache files
  unset(${DOXYGEN_SOURCE_DIRS} CACHE)
  unset(${SOURCE_SILO_DIR} CACHE)
  unset(${SOURCE_SILOCORE_DIR} CACHE)
  unset(${SOURCE_SILOMSGS_DIR} CACHE)
  unset(${SOURCE_SILOROS_DIR} CACHE)
  unset(${SOURCE_SILOSTD_DIR} CACHE)
  unset(${SOURCE_SILORQT_DIR} CACHE)

ELSE(DOXYGEN)
  message(FATAL_ERROR "Doxygen not found")
ENDIF(DOXYGEN)
