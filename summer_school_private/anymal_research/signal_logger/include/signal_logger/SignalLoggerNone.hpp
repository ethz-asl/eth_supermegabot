/*!
 * @file     SignalLoggerNone.hpp
 * @author   Gabriel Hottiger
 * @date     Oct 10, 2016
 * @brief    Empty implementation of the signal logger
 */

#pragma once

// signal logger
#include "signal_logger_core/SignalLoggerBase.hpp"

namespace signal_logger {

class SignalLoggerNone : public SignalLoggerBase
{
 public:
  //! Constructor
  SignalLoggerNone() { }

  //! Destructor
  virtual ~SignalLoggerNone() { }

  //! Add
  template<typename ValueType_>
  void add( const ValueType_ * const var,
            const std::string & name,
            const std::string & group       = LOG_ELEMENT_DEFAULT_GROUP_NAME,
            const std::string & unit        = LOG_ELEMENT_DEFAULT_UNIT,
            const std::size_t divider       = LOG_ELEMENT_DEFAULT_DIVIDER,
            const LogElementAction action   = LOG_ELEMENT_DEFAULT_ACTION,
            const std::size_t bufferSize    = LOG_ELEMENT_DEFAULT_BUFFER_SIZE,
            const BufferType bufferType     = LOG_ELEMENT_DEFAULT_BUFFER_TYPE)
  {
  }

  /** Initializes the logger
   * @param updateFrequency   Update frequency of the controller (frequency of collectLoggerData being called)
   * @param logScriptFileName Filename of the log script, this specifies which topics shall be logged
   */
  virtual void initLogger(const SignalLoggerOptions& options) override { }

  //! Starts the logger (enable collecting)
  virtual bool startLogger(bool updateLogger = false) override { return true; }

  //! Stop the logger (disable collecting)
  virtual bool stopLogger() override { return true; }

  //! Stop and then restart the logger
  virtual bool restartLogger(bool updateLogger = false) override { return true; }

  //! Update the logger
  virtual bool updateLogger(const bool readScript = true, const std::string & scriptname = "") override { return true; }

  //! Save logger script
  virtual bool saveLoggerScript(const std::string & scriptName = LOGGER_DEFAULT_SCRIPT_FILENAME) override { return true; }

  //! Collect log data, read data and push it into the buffer
  virtual bool collectLoggerData() override { return true; }

  //! Publish a single data point of every element from the buffer
  virtual bool publishData() override { return true; }

  //! Save all the buffered data into a log file
  virtual bool saveLoggerData(const signal_logger::LogFileTypeSet & logfileTypes) override { return true; }

  //! Stop the logger and save all the buffered data into a log file
  virtual bool stopAndSaveLoggerData(const signal_logger::LogFileTypeSet & logfileTypes) override { return true; }

 protected:
  //! Saves the logger data in a file in a seperate thread
  virtual bool workerSaveData(const std::string & logFileName, const signal_logger::LogFileTypeSet & logfileTypes) override { return true; };

};

}
