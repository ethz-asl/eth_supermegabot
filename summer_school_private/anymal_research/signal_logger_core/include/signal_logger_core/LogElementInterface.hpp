/*!
 * @file     LogElementInterface.hpp
 * @author   Gabriel Hottiger
 * @date     Sep 22, 2016
 * @brief    A class that defines the basic interface of logger elements.
 */


#pragma once

// signal logger
#include "signal_logger_core/BufferInterface.hpp"
#include "signal_logger_core/LogElementOptions.hpp"
#include "signal_logger_core/LogElementTypes.hpp"
#include "signal_logger_core/typedefs.hpp"

// STL
#include <mutex>
#include <typeindex>

namespace signal_logger {

/**
 *  A list of pointers to this class can be stored in the logger, since the class itself is not templated.
 *  \brief Basic interface of logger elements
 */
//! A class that defines the basic interface of logger elements.
class LogElementInterface
{
 public:
  //! Default constructor
  LogElementInterface() { }

  //! Destructor
  virtual ~LogElementInterface() { }

  //! Reads pointer and pushes the data into the buffer
  virtual void collectData() = 0;

  /** Reads buffer and processes data (probably called from different thread)
   *  @param time                 time log element
   *  @param noCollectDataCalls   number of collectLoggerData calls, this allows time synchronization for publishing
   */
  virtual void publishData(const TimeElement & time , unsigned int noCollectDataCalls) = 0;

  /** Reads buffer and writes data to a file
   *  @param time                 time log element
   *  @param noCollectDataCalls   number of collectLoggerData calls, this allows time synchronization of the log file
   *  @param type                 type of the log file
   */
  virtual void saveDataToLogFile(const TimeElement & times, unsigned int noCollectDataCalls, LogFileType type) = 0;

  //! Stores a copy of the element, file is saved from this
  virtual void copy() = 0;

  //! Reset logger element called before logger start
  virtual void reset() = 0;

  //! Cleanup logger element
  virtual void cleanup() = 0;

  //! @return options of the log element
  virtual LogElementOptions & getOptions() = 0;

  //! @return options of the log element
  virtual const LogElementOptions & getOptions() const = 0;

  //! @return options of the log element copy
  virtual const LogElementOptions & getCopyOptions() const = 0;

  //! @return buffer of the log element
  virtual BufferInterface & getBuffer() = 0;

  //! @return buffer of the log element
  virtual const BufferInterface & getBuffer() const = 0;

  //! @return is log element enabled
  virtual bool isEnabled() const = 0;

  //! @param enabled log element
  virtual void setIsEnabled(const bool isEnabled) = 0;

  //! @return mutex of the log element
  virtual std::mutex& acquireMutex() const = 0;

  //! @return type_index
  virtual std::type_index getType() const = 0;

};

} /* namespace signal_logger */
