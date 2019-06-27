/*!
 * @file	  SignalLoggerOptions.hpp
 * @author	Gabriel Hottiger
 * @date	  Jan 30, 2017
 */

#pragma once

// signal logger
#include "signal_logger_core/typedefs.hpp"

// STL
#include <string>

namespace signal_logger {

//! Struct containing logger options
struct SignalLoggerOptions {

  /** Constructor
   * @param updateFrequency       Frequency at which collectLoggerData() is called. (default: 0u)
   * @param maxLoggingTime        Maximal log time for fixed size time buffer. If set to 0.0 expontentially growing time buffer is used. (default: LOGGER_DEFAULT_MAXIMUM_LOG_TIME)
   * @param collectScriptFileName File name of the data collection yaml script. Format is specified in the documentation (default: LOGGER_DEFAULT_SCRIPT_FILENAME)
   * @param loggerPrefix          Prefix of the logger element names. (e.g. /log in /log/elementA)  (default: LOGGER_DEFAULT_PREFIX)
   * @param loggerPrefix          Logger name for files etc.
   */
  SignalLoggerOptions(const unsigned int updateFrequency,
                      const double maxLoggingTime,
                      std::string collectScriptFileName,
                      std::string loggerPrefix,
                      std::string loggerName)
  : updateFrequency_(updateFrequency),
    maxLoggingTime_(maxLoggingTime),
    collectScriptFileName_(std::move(collectScriptFileName)),
    loggerPrefix_(std::move(loggerPrefix)),
    loggerName_(std::move(loggerName))
  {

  }

  //! Default Constructor
  SignalLoggerOptions()
  : SignalLoggerOptions(0u, LOGGER_DEFAULT_MAXIMUM_LOG_TIME, LOGGER_DEFAULT_SCRIPT_FILENAME, LOGGER_DEFAULT_PREFIX, LOGGER_DEFAULT_NAME)
  {

  }

  //! Rate at which collectLoggerData() is called
  unsigned int updateFrequency_;
  //! Maximal log time for fixed size time buffer
  double maxLoggingTime_;
  //! Collected data script filename
  std::string collectScriptFileName_;
  //! Logger prefix
  std::string loggerPrefix_;
  //! Logger name
  std::string loggerName_;
};

}
