/*!
 * @file     SignalLoggerStd.hpp
 * @author   Gabriel Hottiger
 * @date     Sep 23, 2016
 * @brief    Implementation of a Std signal logger. Provides binary file storing functionality.
 */

#pragma once

// signal logger
#include "signal_logger_core/SignalLoggerBase.hpp"
#include "signal_logger_core/typedefs.hpp"
#include "signal_logger_std/LogElementStd.hpp"

// STL
#include <fstream>
#include <sstream>
#include <memory>

namespace signal_logger_std {

class SignalLoggerStd : public signal_logger::SignalLoggerBase
{
 public:
  SignalLoggerStd();
  virtual ~SignalLoggerStd();

  /** Add variable to logger. This is a default implementation if no specialization is provided an error is posted.
   * @tparam ValueType_       Data type of the logger element
   * @param  var              Pointer to log variable
   * @param  name             name of the log variable
   * @param  group            logger group the variable belongs to
   * @param  unit             unit of the log variable
   * @param  divider          divider is defining the update frequency of the logger element (ctrl_freq/divider)
   * @param  action           log action of the log variable
   * @param  bufferSize       size of the buffer storing log elements
   * @param  bufferType       determines the buffer type
   */
  template<typename ValueType_>
  void add( const ValueType_ * const var,
            const std::string & name,
            const std::string & group                      = signal_logger::LOG_ELEMENT_DEFAULT_GROUP_NAME,
            const std::string & unit                       = signal_logger::LOG_ELEMENT_DEFAULT_UNIT,
            const std::size_t divider                      = signal_logger::LOG_ELEMENT_DEFAULT_DIVIDER,
            const signal_logger::LogElementAction action   = signal_logger::LOG_ELEMENT_DEFAULT_ACTION,
            const std::size_t bufferSize                   = signal_logger::LOG_ELEMENT_DEFAULT_BUFFER_SIZE,
            const signal_logger::BufferType bufferType     = signal_logger::LOG_ELEMENT_DEFAULT_BUFFER_TYPE)
  {
    std::string elementName = options_.loggerPrefix_ + "/" + group + "/" + name;
    elementName.erase(std::unique(elementName.begin(), elementName.end(), signal_logger::both_slashes()), elementName.end());
    {
      // Lock the logger (blocking!)
      boost::unique_lock<boost::shared_mutex> addLoggerLock(loggerMutex_);
      logElementsToAdd_[elementName].reset(new signal_logger_std::LogElementStd<ValueType_>(var, bufferType, bufferSize, elementName ,
                                                                                            unit, divider, action, &textStream_, &binaryStream_));
    }
  }

  //! Save all the buffered data into a log file
  virtual bool workerSaveData(const std::string & logFileName, const signal_logger::LogFileTypeSet & logfileTypes) override;

  /** Initializes the pointer to the logelement */
  virtual void initTimeLogElement() override;

 protected:
  //! Log file
  std::ofstream file_;
  //! Text stream
  std::stringstream textStream_;
  //! Binary stream
  std::stringstream binaryStream_;
};

} /* namespace signal_logger_std */
