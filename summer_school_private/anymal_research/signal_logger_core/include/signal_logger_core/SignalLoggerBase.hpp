/*!
 * @file     SignalLoggerBase.hpp
 * @author   Gabriel Hottiger, Christian Gehring
 * @date     Sep 26, 2016
 * @brief    Base class for signal loggers.
 */

#pragma once

// signal logger
#include "signal_logger_core/LogElementTypes.hpp"
#include "signal_logger_core/LogElementBase.hpp"
#include "signal_logger_core/LogElementInterface.hpp"
#include "signal_logger_core/SignalLoggerOptions.hpp"
#include "signal_logger_core/typedefs.hpp"

// message logger
#include "message_logger/message_logger.hpp"

// eigen
#include <Eigen/Dense>

// boost
#include <boost/thread.hpp>
#include <boost/algorithm/string/predicate.hpp>

// stl
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <unordered_map>
#include <functional>

namespace signal_logger {

//! Class that severs as base class for all the loggers
class SignalLoggerBase {
 protected:
  //! Log element map types
  using LogPair = std::pair<std::string, std::unique_ptr<LogElementInterface>>;
  using LogElementMap = std::unordered_map<std::string, std::unique_ptr<LogElementInterface>>;
  using LogElementMapIterator = LogElementMap::iterator;

 public:
  /** Constructor
   * @param loggerPrefix prefix to the logger variables
   */
  SignalLoggerBase();

  //! Destructor
  virtual ~SignalLoggerBase() = default;

  /** Initializes the logger
   * @param options Signal logger options
   */
  virtual void initLogger(const SignalLoggerOptions& options);

  //! True if logger is running
  bool isRunning() const { return isCollectingData_; }

  //! Starts the logger (enable collecting)
  virtual bool startLogger(bool updateLogger = false);

  //! Stop the logger (disable collecting)
  virtual bool stopLogger();

  //! Stop and then restart the logger
  virtual bool restartLogger(bool updateLogger = false);

  /** Update the logger (ded variables are added) */
  /**
   * @brief Update the logger, variables added since the last call are added
   * @param readScript Determines whether the script is reloaded (sets the state as declared in the script)
   * @param scriptname If empty, the script defined in the options is loaded. Otherwise scriptname will be used.
   * @return true, iff logger was updated successfully
   */
  virtual bool updateLogger(const bool readScript = true, const std::string & scriptname = "");

  /** Save logger script **/
  virtual bool saveLoggerScript(const std::string & scriptName = std::string(LOGGER_DEFAULT_SCRIPT_FILENAME) );

  //! Collect log data, read data and push it into the buffer
  virtual bool collectLoggerData();

  //! Publish a single data point of every element from the buffer
  virtual bool publishData();

  /** Save all the buffered data into a log file
   *  @param logFileTypes types of the log file
   */
  virtual bool saveLoggerData(const LogFileTypeSet & logfileTypes);

  /** Stop the logger and save all the buffered data into log files
   *  @param logFileTypes types of the log file
   */
  virtual bool stopAndSaveLoggerData(const LogFileTypeSet & logfileTypes);

  //! Cleanup logger
  virtual bool cleanup();

  //! Set max logging time
  void setMaxLoggingTime(double maxLoggingTime);

  /**
   * @brief Checks if logger has a log element with name 'name'
   * @param name Name of the log element to check
   * @return true, iff element exists
   */
  bool hasElement(const std::string & name);

  /**
   * @brief Gets a const reference to a log element
   * @param name Name of the log element to get
   * @throws std::out_of_range() if no element with 'name' exists
   * @return const log element reference
   */
  const LogElementInterface & getElement(const std::string & name);

  /**
   * @brief Enables the element if:
   *    - The logger is not running
   *    - Or, the logger is running, the element has looping buffer type
   *      and the time buffer is looping and large enough to provide timing info to the element
   * @param name name of the element
   * @return true, iff successful
   */
  bool enableElement(const std::string & name);

  /**
   * @brief Disables element if logger is not running
   * @param name name of the element
   * @return true, iff successful
   */
  bool disableElement(const std::string & name);

  /**
   * @brief Sets buffer size of element if logger is not running
   * @param name name of the element
   * @param size desired buffer size
   * @return true, iff successful
   */
  bool setElementBufferSize(const std::string & name, const std::size_t size);

  /**
   * @brief Sets buffer type of element if logger is not running
   * @param name name of the element
   * @param type desired buffer type
   * @return true, iff successful
   */
  bool setElementBufferType(const std::string & name, const BufferType type);

  /**
   * @brief Sets divider of element if logger is not running
   * @param name name of the element
   * @param divider desired divider
   * @return true, iff successful
   */
  bool setElementDivider(const std::string & name, const std::size_t divider);

  /**
   * @brief Sets action of element if logger is not running
   * @param name name of the element
   * @param action desired action
   * @return true, iff successful
   */
  bool setElementAction(const std::string & name, const LogElementAction action);

  //@{
  /**
   * @brief Property setting functions extended to namespace
   * @param ns namespace to which changes are applied
   * @return true, iff successful
   */
  bool enableNamespace(const std::string & ns);
  bool disableNamespace(const std::string & ns);
  bool setNamespaceBufferSize(const std::string & ns, const std::size_t size);
  bool setNamespaceBufferType(const std::string & ns, const BufferType type);
  bool setNamespaceDivider(const std::string & ns, const std::size_t divider);
  bool setNamespaceAction(const std::string & ns, const LogElementAction action);
  //@}

  /**
   * @brief Get new values (since the last call to this function) as a vector (first entry is the oldest)
   * @tparam ValueType_ Valuetype of the log element to read values from
   * @param name Name of the element to read values from
   * @throws std::out_of_range if ValueType_ does not correspond stored type, or no element with name 'name' exists
   * @return vector of values since last call to this function (vector_type has a different allocator for Eigen types)
   */
  template<typename ValueType_>
  vector_type<ValueType_> readNewValues(const std::string & name) {
    boost::shared_lock<boost::shared_mutex> lockLogger(loggerMutex_);

    if(!hasElement(name) || logElements_[name]->getType() !=  typeid(ValueType_) ) {
      throw std::out_of_range("[SignalLoggerBase]::readNewValues(): Element " + name +
          " was not added to the logger or is not of type " + std::type_index(typeid(ValueType_)).name() + "!");
    }

    return static_cast< LogElementBase<ValueType_>* >(logElements_[name].get())->readNewValues();
  }

 protected:
  /** Reads collect script and enables all log data
   * @param scriptName filename of the logging script
   */
  bool readDataCollectScript(const std::string & scriptName);

  /** Save collect script
   * @param scriptName filename of the logging script
   */
  bool saveDataCollectScript(const std::string & scriptName);

  /** Saves the logger data in a file in a seperate thread
   * @param logFileName filename of the log file
   * @param logfileTypes types of the log files
   */
  virtual bool workerSaveData(const std::string & logFileName, const LogFileTypeSet & logfileTypes) = 0;

  /** Initializes the pointer to the log element */
  virtual void initTimeLogElement();

  /** Returns the current time
   * @return current time
   */
  virtual signal_logger::TimestampPair getCurrentTime();

  //! Lock free version of has element
  virtual bool hasElementLockFree(const std::string & name);

  //! Lock free version of updateLogger
  virtual bool updateLoggerLockFree(const bool readScript = true, const std::string & scriptname = "");

 private:
  /** Wraps function workerSaveData to do common preparations and shutdown
   * @param logFileTypse types of the log files
   */
  bool workerSaveDataWrapper(const LogFileTypeSet & logfileTypes);

  /** Wait until logger can be started and start logger
   */
  bool workerStartLogger(bool updateLogger = false);

  //! Comparison operator, get element with largest scaled buffer size
  struct maxScaledBufferSize {
    /*** Defines the comparison operator
     *   @param i  first element to compare
     *   @param j  second element to compare
     *   @return fun(i) < fun(j)
     */
    bool operator() (const LogElementMapIterator& i, const LogElementMapIterator& j)
    {
      return (i->second->getOptions().getDivider()*i->second->getBuffer().getBufferSize()) <
             (j->second->getOptions().getDivider()*j->second->getBuffer().getBufferSize());
    }
  };

  /**
   * @brief Set property of log element, if:
   *          - The logger is not running
   *          - Or the logger is running and the element is disabled
   * @tparam PropertyType_ Type of the property
   * @param elementName    Name of the element of which to change the property
   * @param property       Desired value of the property
   * @param propertyName   Descriptive name of the property (used for warn outputs)
   * @param propertyGetter Lambda that gets the property from a log element interface ptr
   * @param propertySetter Lambda that sets the desired property to a log element interface ptr
   * @return true, iff successful
   */
  template <typename PropertyType_>
  bool setElementProperty(const std::string & elementName,
                          const PropertyType_ property,
                          const std::string & propertyName,
                          std::function<PropertyType_(const std::unique_ptr<LogElementInterface> &)> propertyGetter,
                          std::function<void(const std::unique_ptr<LogElementInterface> &, const PropertyType_)> propertySetter)
  {
    boost::upgrade_lock<boost::shared_mutex> lockLogger(loggerMutex_);
    if( !hasElementLockFree(elementName) ) {
      MELO_WARN_STREAM("[SignalLogger::setElementProperty] Can not set " << propertyName << " of non-existing element with name " << elementName << "!");
      return false;
    } else {
      if( propertyGetter(logElements_[elementName]) == property ) { return true; }
      if( isCollectingData_ && logElements_[elementName]->isEnabled() ) {
        MELO_WARN_STREAM("[SignalLogger::setElementProperty] Can not set " << propertyName << " of enabled element " << elementName << " when the logger is running!");
        return false;
      }
      boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLockLogger(lockLogger);
      propertySetter(logElements_[elementName], property);
    }
    return true;
  }

  /**
   * @brief Set a property for a complete namespace
   * @param ns namespace for which property shall be changed (remember log_element_name = (lognamespace + / + ns + / + name) )
   * @param setElementProperty Function setting the property of a single element
   * @return true, iff successful
   */
  bool setElementPropertyForNamespace(const std::string & ns,
                                      std::function<bool(const std::string &)> setElementProperty)
  {
    std::vector<std::string> elementList;
    {
      boost::shared_lock<boost::shared_mutex> lockLogger(loggerMutex_);
      for(auto & element : logElements_ ) {
        if( boost::starts_with(element.first, options_.loggerPrefix_ + "/" + ns) ) {
          elementList.push_back(element.first);
        }
      }
    }

    bool success =  true;
    for(auto & element : elementList ) {
      success = setElementProperty(element) && success;
    }

    return success;
  }

 protected:
  //! Logger Options
  SignalLoggerOptions options_;
  //! Nr of calls to collect data
  std::atomic_uint noCollectDataCalls_;
  //! Copy of nr of calls to collect data for saving
  std::atomic_uint noCollectDataCallsCopy_;

  //-- Log Elements

  //! Map of log elements (excluding time, elements added after last updateLogger() call)
  LogElementMap logElements_;
  //! Vector of iterators in logElements_ map
  std::vector<LogElementMapIterator> enabledElements_;
  //! Map of all log elements added since last call to updateLogger()
  LogElementMap logElementsToAdd_;

  //! Time variable
  TimestampPair logTime_;
  //! Corresponding time log element
  std::shared_ptr<LogElementBase<TimestampPair>> timeElement_;

  //-- Execution Flags

  //! Flag to check if logger is initialized
  std::atomic_bool isInitialized_;
  //! Flag to collect data
  std::atomic_bool isCollectingData_;
  //! Flag to save data
  std::atomic_bool isSavingData_;
  //! Flag to save data
  std::atomic_bool isCopyingBuffer_;
  //! Flag is starting in different thread
  std::atomic_bool isStarting_;
  //! Flag if data should be published
  std::atomic_bool shouldPublish_;

  //-- Mutexes
  //! Init, add, update, start, stop and cleanup should be called in sequence
  boost::shared_mutex loggerMutex_;
  //! Allow call to save logger data only once
  std::mutex saveLoggerDataMutex_;

};

} // end namespace signal_logger
