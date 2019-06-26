/*!
 * @file     LogElementBase.hpp
 * @author   Gabriel Hottiger
 * @date     Sep 22, 2016
 * @brief    Templated base class for log elements.
 */

#pragma once

// Signal logger
#include "signal_logger_core/LogElementInterface.hpp"
#include "signal_logger_core/Buffer.hpp"

// STL
#include <atomic>
#include <mutex>

namespace signal_logger {

template <typename ValueType_>
class LogElementBase: public LogElementInterface
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
   */
  LogElementBase(const ValueType_ * const ptr,
                 const BufferType  bufferType,
                 const std::size_t bufferSize,
                 const std::string & name,
                 const std::string & unit,
                 const std::size_t divider,
                 const LogElementAction action) :
    LogElementBase(ptr, bufferType, bufferSize, LogElementOptions(name, unit, divider, action))
  {

  }

  /** Constructor
   *  @param ptr        pointer to the log var
   *  @param bufferType buffer type of the log var
   *  @param bufferSize buffer size of the log var
   *  @param options    options of the log var
   */
  LogElementBase(const ValueType_ * const ptr,
                 const BufferType  bufferType,
                 const std::size_t bufferSize,
                 const LogElementOptions & options) :
   LogElementInterface(),
   buffer_(ptr, options.getName()), // Zero buffer size log element not enabled
   options_(options),
   isEnabled_(false),
   mutex_(),
   bufferCopy_(ptr, options.getName()),
   optionsCopy_(options),
   mutexCopy_()
 {
    // Initialize the buffer with the given options
    buffer_.setBufferType(bufferType);
    buffer_.setBufferSize(bufferSize);
 }

  //! Destructor
  virtual ~LogElementBase() { }

  //! Data collection is not element specific
  void collectData() override final { buffer_.collect(); }

  //! Reads buffer and processes data (probably called from different thread)
  virtual void publishData(const TimeElement & time,
                           unsigned int nrCollectDataCalls) override { }

  //! Writes local buffer copy to a file
  virtual void saveDataToLogFile(const TimeElement & times,
                                 unsigned int nrCollectDataCalls,
                                 LogFileType type) override { }

  //! Stores a copy of the current buffer, file is saved from this
  virtual void copy() {
    // Lock all mutexes and copy the buffer
    std::unique_lock<std::mutex> lock(mutex_);
    std::unique_lock<std::mutex> lockCopy(mutexCopy_);
    bufferCopy_.transfer(buffer_);
    optionsCopy_.transfer(options_);
  }

  //! Reset logger element called before logger start
  virtual void reset() override { buffer_.clear(); }

  //! Cleanup logger element
  virtual void cleanup() override { }

  //! @return options of the copy of log element
  const LogElementOptions & getCopyOptions() const { return optionsCopy_; }

  //! @return options of the log element
  const LogElementOptions & getOptions() const { return options_; }

  //! @return options of the log element
  LogElementOptions & getOptions() { return options_; }

  //! @return buffer of the log element
  const BufferInterface & getBuffer() const { return buffer_; }

  //! @return non const buffer interface
  BufferInterface & getBuffer() { return buffer_; }

  //! @return mutex of the log element
  std::mutex& acquireMutex() const { return mutex_; }

  //! @return is log element enabled
  bool isEnabled() const { return isEnabled_.load(); }

  //! @param enabled log element
  void setIsEnabled(const bool isEnabled) {
    isEnabled_.store(isEnabled);
    buffer_.allocate(isEnabled_);
    update();
  }

  //! @return type_index
  virtual std::type_index getType() const { return typeid(ValueType_); }

  //! Expose readNewValues function of buffer
  vector_type<ValueType_> readNewValues() { return buffer_.readNewValues(); }

  /*** Get access to the buffer copy
   *   @tparam V  log element type (ValueType_)
   *   @return    buffer copy
   */
  template<typename V = ValueType_>
  const Buffer<V>& getTimeBufferCopy(typename std::enable_if<std::is_same<TimestampPair, V>::value>::type* = 0 /* is timestamp pair */) const
  {
    return bufferCopy_;
  }

  /*** Get access to the buffer
   *   @tparam V  log element type (ValueType_)
   *   @return    buffer copy
   */
  template<typename V = ValueType_>
  const Buffer<V>& getTimeBuffer(typename std::enable_if<std::is_same<TimestampPair, V>::value>::type* = 0 /* is timestamp pair */) const
  {
    return buffer_;
  }


protected:
  //! Update the element
  virtual void update() { };

 protected:
  //! Buffer (threadsafe)
  Buffer<ValueType_> buffer_;
  //! Options of the log element
  LogElementOptions options_;
  //! Indicates if log element is currently active
  std::atomic_bool isEnabled_;
  //! Mutex protecting element
  mutable std::mutex mutex_;

  //! Buffer copy (actually the buffer is moved during saving)
  Buffer<ValueType_> bufferCopy_;
  //! Options copy of the log element
  LogElementOptions optionsCopy_;
  //! Mutex protecting element copy
  mutable std::mutex mutexCopy_;

};

} /* namespace signal_logger */
