/*!
 * @file     LogElementStd.hpp
 * @author   Gabriel Hottiger
 * @date     Sep 22, 2016
 * @brief    Implementation of a Log element for std logging. Save data to binary file.
 */

#pragma once

// Signal logger
#include "signal_logger_core/LogElementBase.hpp"
#include "signal_logger_std/signal_logger_std_traits.hpp"

// STL
#include <fstream>
#include <unordered_set>

namespace signal_logger_std {

template <typename ValueType_>
class LogElementStd: public signal_logger::LogElementBase<ValueType_>
{
 public:
  /** Constructor
   *  @param ptr        pointer to the log var
   *  @param bufferType buffer type of the log var
   *  @param bufferSize buffer size of the log var
   *  @param name       name of the log var
   *  @param unit       unit of the log var
   *  @param divider    log_freq = ctrl_freq/divider
   *  @param action     save, publish or save and publish
   *  @param textStream string stream for text part of the log file
   *  @param binaryStream string stream for binary part of the log file
   */
  LogElementStd(const ValueType_ * const ptr,
                const signal_logger::BufferType bufferType,
                const std::size_t bufferSize,
                const std::string & name,
                const std::string & unit,
                const std::size_t divider,
                const signal_logger::LogElementAction action,
                std::stringstream * textStream,
                std::stringstream * binaryStream) :
      signal_logger::LogElementBase<ValueType_>(ptr, bufferType, bufferSize, name, unit, divider, action),
      textStream_(textStream),
      binaryStream_(binaryStream)
  {

  }

  //! Destructor
  virtual ~LogElementStd()
  {

  }

  //! Save Data to file
  void saveDataToLogFile(const signal_logger::TimeElement & times,
                         unsigned int nrCollectDataCalls,
                         signal_logger::LogFileType type) override
  {
    // Lock the copy mutex
    std::unique_lock<std::mutex> lock(this->mutexCopy_);
    if(this->bufferCopy_.noTotalItems() > 0 ) {
      unsigned int startDiff = 0;
      unsigned int endDiff = (nrCollectDataCalls - 1) % this->optionsCopy_.getDivider();

      if( type == signal_logger::LogFileType::CSV ) {
        if(this->bufferCopy_.getBufferType() == signal_logger::BufferType::LOOPING) {
          /* Last index of time: (times.size() - 1)
           * Index of newest time corresponding to a data point:  (nrCollectDataCalls - 1) % this->dividerCopy_
           * Offset of oldest time that corresponds to a data point: (this->bufferCopy_.size()-1) * this->dividerCopy_
           */
          startDiff = (times.getTimeBufferCopy().noTotalItems() - 1) - endDiff - (this->bufferCopy_.noTotalItems()-1) * this->optionsCopy_.getDivider();
        }
      }

      // Write to fill
      signal_logger_std::traits::sls_traits<ValueType_, ValueType_>::writeLogElementToStreams(
          textStream_, binaryStream_, type, this->bufferCopy_, this->optionsCopy_.getName(), this->optionsCopy_.getDivider(), startDiff, endDiff);
    }
  }

 protected:
  //! Text stream
  std::stringstream* textStream_;
  //! Binary stream
  std::stringstream* binaryStream_;

};

} /* namespace signal_logger */
