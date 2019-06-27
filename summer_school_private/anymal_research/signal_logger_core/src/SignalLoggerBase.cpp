/*!
 * @file     SignalLoggerBase.cpp
 * @author   Gabriel Hottiger, Christian Gehring
 * @date     Sep 28, 2016
 * @brief    Base class for signal loggers.
 */

// signal logger
#include "signal_logger_core/SignalLoggerBase.hpp"

// yaml
#include <yaml-cpp/yaml.h>
#include "signal_logger_core/yaml_helper.hpp"

// boost
#include <boost/iterator/counting_iterator.hpp>

// stl
#include "assert.h"
#include <thread>
#include <fstream>
#include <chrono>
#include <ctime>

// system
#include <sys/stat.h>

namespace signal_logger {

SignalLoggerBase::SignalLoggerBase():
                  options_(),
                  noCollectDataCalls_(0u),
                  noCollectDataCallsCopy_(0u),
                  logElements_(),
                  enabledElements_(),
                  logElementsToAdd_(),
                  logTime_(),
                  timeElement_(),
                  isInitialized_(false),
                  isCollectingData_(false),
                  isSavingData_(false),
                  isCopyingBuffer_(false),
                  isStarting_(false),
                  shouldPublish_(false),
                  loggerMutex_(),
                  saveLoggerDataMutex_()
{
}

void SignalLoggerBase::initLogger(const SignalLoggerOptions& options)
{
  // Lock the logger (blocking!)
  boost::unique_lock<boost::shared_mutex> initLoggerLock(loggerMutex_);

  // Assert corrupted configuration
  assert(options.updateFrequency_ > 0);

  // Set configuration
  options_ = options;

  // Init time element
  initTimeLogElement();

  // Notify user
  MELO_INFO("[SignalLogger::initLogger] Initialized logger '%s'!", options_.loggerName_.c_str());
  isInitialized_ = true;
}

bool SignalLoggerBase::startLogger(bool updateLogger)
{
  // Delayed logger start if saving prevents lock of mutex
  boost::unique_lock<boost::shared_mutex> tryStartLoggerLock(loggerMutex_, boost::try_to_lock);
  if(!tryStartLoggerLock && isCopyingBuffer_) {
    // Still copying the data from the buffer, Wait in other thread until logger can be started.
    std::thread t1(&SignalLoggerBase::workerStartLogger, this, updateLogger);
    t1.detach();
    return true;
  }

  // Lock the logger if not locked yet (blocking!)
  boost::unique_lock<boost::shared_mutex> startLoggerLock(loggerMutex_, boost::defer_lock);
  if(!tryStartLoggerLock) { startLoggerLock.lock(); }

  // Warn the user on invalid request
  if(!isInitialized_ || isStarting_) {
    MELO_WARN("[SignalLogger::startLogger] Could not start logger '%s'!%s%s", options_.loggerName_.c_str(),
              !isInitialized_?" Not initialized!":"", isStarting_?" Delayed logger start was already requested!":"");
    return false;
  } else if( isCollectingData_ ) {
    MELO_DEBUG("[SignalLogger::startLogger] Already started logger '%s'!", options_.loggerName_.c_str());
    return true;
  }

  if(isCopyingBuffer_ || (updateLogger && isSavingData_) ) {
    // Still copying the data from the buffer, Wait in other thread until logger can be started.
    std::thread t1(&SignalLoggerBase::workerStartLogger, this, updateLogger);
    t1.detach();
    return true;
  }

  // Update logger if requested
  if(updateLogger) {
    if(!this->updateLoggerLockFree()) {
      MELO_WARN("[SignalLogger::startLogger] Could not update logger '%s' during logger start!", options_.loggerName_.c_str());
      return true;
    }
  }

  // Decide on the time buffer to use ( Init with exponentially growing when max log time is zero, fixed size buffer otherwise)
  const double maxLogTime = (options_.maxLoggingTime_ == 0.0) ? LOGGER_EXP_GROWING_MAXIMUM_LOG_TIME : options_.maxLoggingTime_;
  size_t timeBufferSize =  static_cast<size_t >(maxLogTime * options_.updateFrequency_);
  BufferType timeBufferType =  (options_.maxLoggingTime_ == 0.0) ? BufferType::EXPONENTIALLY_GROWING : BufferType::FIXED_SIZE;

  if(enabledElements_.empty()) {
    // If no elements are logged, log with a looping buffer TODO: keep max log time?
    timeBufferType = BufferType::LOOPING;
  } else {
    // If all elements are looping or all elements are fixed size use a time buffer of the according size/type
    for(auto bufferType : { BufferType::FIXED_SIZE, BufferType::LOOPING } ) {
      auto hasOtherElement = [bufferType] (const LogElementMapIterator& s) { return s->second->getBuffer().getBufferType() != bufferType; };
      auto otherElement = std::find_if(enabledElements_.begin(), enabledElements_.end(), hasOtherElement);
      if( otherElement == enabledElements_.end() ) {
        auto maxElement = std::max_element(enabledElements_.begin(), enabledElements_.end(), maxScaledBufferSize());
        timeBufferSize = ( maxElement != enabledElements_.end() ) ? ( (*maxElement)->second->getOptions().getDivider() *
            (*maxElement)->second->getBuffer().getBufferSize() ) : 0;
        timeBufferType = bufferType;
        MELO_INFO_STREAM("[Signal logger] Logger '" << options_.loggerName_ << "' uses "
                         << ( (bufferType == BufferType::FIXED_SIZE)? "fixed size" : "looping" )
                         << " Time Buffer of size:" << timeBufferSize);
      }
    }
  }


  // Set time buffer properties to the element
  timeElement_->getBuffer().setBufferType(timeBufferType);
  timeElement_->getBuffer().setBufferSize(timeBufferSize);

  // Reset elements ( Clear buffers )
  timeElement_->reset();
  for(auto & elem : logElements_) { elem.second->reset(); }

  // Reset flags and data collection calls
  isCollectingData_ = true;
  shouldPublish_ = true;
  noCollectDataCalls_ = 0;

  return true;
}

bool SignalLoggerBase::stopLogger()
{
  // If lock can not be acquired because of saving ignore the call
  boost::unique_lock<boost::shared_mutex> tryStopLoggerLock(loggerMutex_, boost::try_to_lock);
  if(!tryStopLoggerLock && isSavingData_) {
    MELO_WARN("Saving data while trying to stop logger '%s'. Do nothing!", options_.loggerName_.c_str());
    return false;
  }

  // Lock the logger if not locked yet (blocking!)
  boost::unique_lock<boost::shared_mutex> stopLoggerLock(loggerMutex_, boost::defer_lock);
  if(!tryStopLoggerLock) { stopLoggerLock.lock(); }

  // Those are cases where the logger is already stopped. So "stopping" was successful.
  if(!isInitialized_) {
    MELO_DEBUG("[SignalLogger::stopLogger] Could not stop non-initialized logger '%s'!", options_.loggerName_.c_str());
  } else if(!isCollectingData_) {
    MELO_DEBUG("[SignalLogger::stopLogger] Logger '%s' was already stopped!", options_.loggerName_.c_str());
  }

  // If publishData is running notify stop
  shouldPublish_ = false;

  // Stop the collection of data!
  isCollectingData_ = false;

  return true;
}

bool SignalLoggerBase::restartLogger(bool updateLogger)
{
  // Mutex is locked internally
  bool stopped = stopLogger();
  return startLogger(updateLogger) && stopped;
}

bool SignalLoggerBase::updateLogger(const bool readScript, const std::string & scriptname) {

  // If lock can not be acquired because of saving ignore the call
  boost::unique_lock<boost::shared_mutex> tryUpdateLoggerLock(loggerMutex_, boost::try_to_lock);
  if(!tryUpdateLoggerLock && isSavingData_) {
    MELO_WARN("Saving data while trying to update logger '%s'. Do nothing!", options_.loggerName_.c_str());
    return false;
  }

  // Lock the logger if not locked yet (blocking!)
  boost::unique_lock<boost::shared_mutex> updateLoggerLock(loggerMutex_, boost::defer_lock);
  if(!tryUpdateLoggerLock) { updateLoggerLock.lock(); }


  return updateLoggerLockFree(readScript, scriptname);
}

bool SignalLoggerBase::updateLoggerLockFree(const bool readScript, const std::string & scriptname) {
  // Check if update logger call is valid
  if(!isInitialized_ || isCollectingData_ || isSavingData_)
  {
    MELO_WARN("[Signal logger] Could not update logger '%s'!%s%s%s", options_.loggerName_.c_str(), !isInitialized_?" Not initialized!":"",
              isSavingData_?" Saving data!":"", isCollectingData_?" Collecting data!":"");
    return false;
  }

  // Add elements to list
  for(auto & elemToAdd : logElementsToAdd_ ) {
    // Transfer ownership to logElements
    logElements_[elemToAdd.first] = std::unique_ptr<LogElementInterface>(std::move(elemToAdd.second));
  }
  // Clear elements
  logElementsToAdd_.clear();

  // Read the script
  if(readScript) {
    if( !readDataCollectScript( scriptname.empty() ? options_.collectScriptFileName_ : scriptname ) ) {
      MELO_ERROR("[Signal logger] Could not load logger script for logger '%s'!", options_.loggerName_.c_str());
      return false;
    }
  }

  return true;
}

bool SignalLoggerBase::saveLoggerScript(const std::string & scriptName) {
  // If lock can not be acquired because of saving ignore the call
  boost::unique_lock<boost::shared_mutex> trySaveLoggerScriptLock(loggerMutex_, boost::try_to_lock);
  if(!trySaveLoggerScriptLock && isSavingData_) {
    MELO_WARN("Saving data while trying to save logger script for logger '%s'. Do nothing!", options_.loggerName_.c_str());
    return false;
  }

  // Lock the logger if not locked yet (blocking!)
  boost::unique_lock<boost::shared_mutex> saveLoggerScriptLock(loggerMutex_, boost::defer_lock);
  if(!trySaveLoggerScriptLock) { saveLoggerScriptLock.lock(); }

  if( !saveDataCollectScript( scriptName ) ){
    MELO_ERROR("[Signal logger] Could not save logger script for logger '%s'!", options_.loggerName_.c_str());
    return false;
  }
  return true;
}

bool SignalLoggerBase::collectLoggerData()
{
  // Try lock logger for read (non blocking!)
  boost::shared_lock<boost::shared_mutex> collectLoggerDataLock(loggerMutex_, boost::try_to_lock);

  if(collectLoggerDataLock)
  {
    //! Check if logger is initialized
    if(!isInitialized_) { return false; }

    //! If buffer is copied return -> don't collect
    if(isCopyingBuffer_) { return true; }

    // Is logger started?
    if(isCollectingData_)
    {
      // Set current time (thread safe, since only used in init, start which have unique locks on the logger)
      logTime_ = this->getCurrentTime();

      // Check if time buffer is full when using a fixed size buffer
      if(timeElement_->getBuffer().getBufferType() == BufferType::FIXED_SIZE && timeElement_->getBuffer().noTotalItems() == timeElement_->getBuffer().getBufferSize())
      {
        MELO_WARN("[Signal Logger] Stopped logger '%s'. Time buffer is full!", options_.loggerName_.c_str());
        // Free lock and stop logger
        collectLoggerDataLock.unlock();
        this->stopLogger();
        return true;
      }

      // Lock all mutexes for time synchronization
      for(auto & elem : enabledElements_)
      {
        elem->second->acquireMutex().lock();
      }

      // Update time
      {
        std::unique_lock<std::mutex> uniqueTimeElementLock(timeElement_->acquireMutex());
        timeElement_->collectData();
      }

      // Collect element into buffer and unlock element mutexes (time/value pair is valid now)
      for(auto & elem : enabledElements_)
      {
        if((noCollectDataCalls_ % elem->second->getOptions().getDivider()) == 0) {
          elem->second->collectData();
        }
        elem->second->acquireMutex().unlock();
      }

      // Iterate
      ++noCollectDataCalls_;
    }
  }
  else {
    MELO_DEBUG("[Signal Logger] Dropping collect data call for logger '%s'!", options_.loggerName_.c_str());
  }

  return true;
}

bool SignalLoggerBase::publishData()
{
  // Try lock logger for read (non blocking!)
  boost::shared_lock<boost::shared_mutex> publishDataLock(loggerMutex_, boost::try_to_lock);

  if(publishDataLock && shouldPublish_)
  {
    // Publish data from buffer
    for(auto & elem : enabledElements_)
    {
      // Publishing blocks the mutex for quite a long time, allow interference by stopLogger using atomic bool
      if(shouldPublish_) {
        if(elem->second->getOptions().isPublished()) {
          elem->second->publishData(*timeElement_, noCollectDataCalls_);
        }
      }
      else {
        return true;
      }
    }
  }

  return true;
}

bool SignalLoggerBase::saveLoggerData(const LogFileTypeSet & logfileTypes)
{
  // Make sure start stop are not called in the meantime
  boost::shared_lock<boost::shared_mutex> saveLoggerDataLock(loggerMutex_);

  // Allow only single call to save logger data
  std::unique_lock<std::mutex> lockSaveData(saveLoggerDataMutex_, std::try_to_lock);

  if(lockSaveData) {
    if(!isInitialized_ || isSavingData_)
    {
      MELO_WARN("[Signal logger] Could not save data for logger '%s'! %s%s", options_.loggerName_.c_str(),
                isSavingData_?" Already saving data!":"", !isInitialized_?" Not initialized!":"");
      return false;
    }

    // set save flag
    isCopyingBuffer_ = true;
    isSavingData_ = true;

    // Save data in different thread
    std::thread t1(&SignalLoggerBase::workerSaveDataWrapper, this, logfileTypes);
    t1.detach();
  }
  else {
    MELO_WARN("[Signal logger] Already saving data to file for logger '%s'!", options_.loggerName_.c_str());
  }

  return true;
}

bool SignalLoggerBase::stopAndSaveLoggerData(const LogFileTypeSet & logfileTypes)
{
  // Mutex is locked internally
  bool stopped = stopLogger();
  return saveLoggerData(logfileTypes) && stopped;
}

bool SignalLoggerBase::cleanup()
{
  // Waiting for data to be saved
  while(isSavingData_) {
    MELO_INFO("[SignalLogger:cleanup]: Waiting for saving of logger '%s' to complete ... ", options_.loggerName_.c_str());
    usleep( static_cast<__useconds_t >(5e5) ); // Wait half a second
  }

  // Make sure logger is stopped
  stopLogger();

  // Lock the logger (blocking!)
  boost::unique_lock<boost::shared_mutex> lockLogger(loggerMutex_);

  // Publish data from buffer
  for(auto & elem : logElements_) { elem.second->cleanup(); }
  for(auto & elem : logElementsToAdd_) { elem.second->cleanup(); }

  // Clear maps
  enabledElements_.clear();
  logElements_.clear();
  logElementsToAdd_.clear();

  return true;
}

void SignalLoggerBase::setMaxLoggingTime(double maxLoggingTime) {
  // The max logging time is only needed in startLogger, no issues it setting it anytime
  boost::shared_lock<boost::shared_mutex> lockLogger(loggerMutex_);
  options_.maxLoggingTime_ = std::max(0.0, maxLoggingTime);
}


bool SignalLoggerBase::hasElementLockFree(const std::string & name) {
  return logElements_.find(name) != logElements_.end();
}

bool SignalLoggerBase::hasElement(const std::string & name)
{
  boost::shared_lock<boost::shared_mutex> lockLogger(loggerMutex_);
  return hasElementLockFree(name);
}

const LogElementInterface & SignalLoggerBase::getElement(const std::string & name)
{
  boost::shared_lock<boost::shared_mutex> lockLogger(loggerMutex_);
  if(!hasElementLockFree(name)) {
    throw std::out_of_range("[SignalLoggerBase]::getElement(): Element " + name + " was not added to logger '" +
        options_.loggerName_ + "'!");
  }
  return *logElements_[name];
}

bool SignalLoggerBase::enableElement(const std::string & name)
{
  boost::upgrade_lock<boost::shared_mutex> lockLogger(loggerMutex_);
  if( !hasElementLockFree(name) ) {
    MELO_WARN_STREAM("[SignalLogger::enableElement] Can not enable non-existing element with name " << name << " for logger '" <<
        options_.loggerName_ << "'!");
    return false;
  }
  if( logElements_[name]->isEnabled() ) { return true; }

  if( isCollectingData_ && logElements_[name]->getBuffer().getBufferType() != BufferType::LOOPING ) {
    MELO_WARN_STREAM("[SignalLogger::enableElement]: Can not enable element " << name << " with non-looping buffer type when logger '" <<
        options_.loggerName_ << "' is running!");
    return false;
  }

  if(isCollectingData_ && ( timeElement_->getBuffer().getBufferType() != BufferType::LOOPING ||
      (logElements_[name]->getOptions().getDivider() * logElements_[name]->getBuffer() .getBufferSize()) > timeElement_->getBuffer().getBufferSize() ) ) {
    MELO_WARN_STREAM("[SignalLogger::enableElement]: Can not enable element " << name << " when logger '"
      << options_.loggerName_ << "' is running and time buffer is too small! " << "(Des: "
      << logElements_[name]->getOptions().getDivider() * logElements_[name]->getBuffer() .getBufferSize()
      << " / Time: " << timeElement_->getBuffer().getBufferSize() <<")");
    return false;
  }

  boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLockLogger(lockLogger);
  logElements_[name]->setIsEnabled(true);
  enabledElements_.push_back(logElements_.find(name));

  return true;
}

bool SignalLoggerBase::disableElement(const std::string & name)
{
  auto isEnabled = [](const std::unique_ptr<LogElementInterface> & e) { return e->isEnabled(); };
  auto setIsEnabled = [this, name](const std::unique_ptr<LogElementInterface> & e, const bool p) {
    e->setIsEnabled(p);
    auto it =  std::find(enabledElements_.begin(), enabledElements_.end(), logElements_.find(name));
    if( it != enabledElements_.end() ) { enabledElements_.erase(it); }
  };
  return setElementProperty<bool>(name, false, "enabled(false)", isEnabled, setIsEnabled);
}

bool SignalLoggerBase::setElementBufferSize(const std::string & name, const std::size_t size)
{
  auto getBufferSize = [](const std::unique_ptr<LogElementInterface> & e) { return e->getBuffer().getBufferSize(); };
  auto setBufferSize = [](const std::unique_ptr<LogElementInterface> & e, const std::size_t p) { e->getBuffer().setBufferSize(p); };
  return setElementProperty<std::size_t>(name, size, "buffer size", getBufferSize, setBufferSize);
}

bool SignalLoggerBase::setElementBufferType(const std::string & name, const BufferType type)
{
  auto getBufferType = [](const std::unique_ptr<LogElementInterface> & e) { return e->getBuffer().getBufferType(); };
  auto setBufferType = [](const std::unique_ptr<LogElementInterface> & e, const BufferType p) { e->getBuffer().setBufferType(p); };
  return setElementProperty<BufferType>(name, type, "buffer type", getBufferType, setBufferType);
}

bool SignalLoggerBase::setElementDivider(const std::string & name, const std::size_t divider)
{
  auto getDivider = [](const std::unique_ptr<LogElementInterface> & e) { return e->getOptions().getDivider(); };
  auto setDivider = [](const std::unique_ptr<LogElementInterface> & e, const std::size_t p) { e->getOptions().setDivider(p); };
  return setElementProperty<std::size_t>(name, divider, "divider", getDivider, setDivider);
}

bool SignalLoggerBase::setElementAction(const std::string & name, const LogElementAction action)
{
  auto getAction = [](const std::unique_ptr<LogElementInterface> & e) { return e->getOptions().getAction(); };
  auto setAction = [](const std::unique_ptr<LogElementInterface> & e, const LogElementAction p) { e->getOptions().setAction(p); };
  return setElementProperty<LogElementAction>(name, action, "action", getAction, setAction);
}

bool SignalLoggerBase::enableNamespace(const std::string & ns)
{
  auto f = [this](const std::string & name) { return enableElement(name); };
  return setElementPropertyForNamespace(ns, f);
}

bool SignalLoggerBase::disableNamespace(const std::string & ns)
{
  auto f = [this](const std::string & name) { return disableElement(name); };
  return setElementPropertyForNamespace(ns, f);
}

bool SignalLoggerBase::setNamespaceBufferSize(const std::string & ns, const std::size_t size)
{
  auto f = [size, this](const std::string & name) { return setElementBufferSize(name, size); };
  return setElementPropertyForNamespace(ns, f);
}

bool SignalLoggerBase::setNamespaceBufferType(const std::string & ns, const BufferType type)
{
  auto f = [type, this](const std::string & name) { return setElementBufferType(name, type); };
  return setElementPropertyForNamespace(ns, f);
}

bool SignalLoggerBase::setNamespaceDivider(const std::string & ns, const std::size_t divider)
{
  auto f = [divider, this](const std::string & name) { return setElementDivider(name, divider); };
  return setElementPropertyForNamespace(ns, f);
}

bool SignalLoggerBase::setNamespaceAction(const std::string & ns, const LogElementAction action)
{
  auto f = [action, this](const std::string & name) { return setElementAction(name, action); };
  return setElementPropertyForNamespace(ns, f);
}

bool SignalLoggerBase::readDataCollectScript(const std::string & scriptName)
{
  if(!isInitialized_ || isSavingData_)
  {
    MELO_WARN("[Signal logger] Could not read data collect script for logger '%s'! %s%s", options_.loggerName_.c_str(),
              isCollectingData_?" Collecting data!":"", !isInitialized_?" Not initialized!":"");
    return false;
  }

  // Check filename size and ending
  std::string ending = ".yaml";
  if ( ( (ending.size() + 1) > scriptName.size() ) || !std::equal(ending.rbegin(), ending.rend(), scriptName.rbegin()) ) {
    MELO_ERROR("[Signal logger] Script for logger '%s' must be a yaml file : *.yaml. Enable all elements!", options_.loggerName_.c_str());
    enabledElements_.clear();
    for(auto it = logElements_.begin(); it != logElements_.end(); ++it) {
      it->second->setIsEnabled(true);
      enabledElements_.push_back(it);
    }
    return false;
  }

  // Check file existance
  struct stat buffer;
  if(stat(scriptName.c_str(), &buffer) != 0) {
    std::fstream fs;
    fs.open(scriptName, std::fstream::out);
    fs.close();
  }

  // Disable all log data and reallocate buffer
  // Thread-safe since these methods are only called by update logger which owns a unique lock of the elements map
  for(auto & elem : enabledElements_) { elem->second->setIsEnabled(false); }
  enabledElements_.clear();

  // Get set of numbers from 0...(size-1)
  std::set<unsigned int> iteratorOffsets(boost::counting_iterator<unsigned int>(0),
                                         boost::counting_iterator<unsigned int>(logElements_.size()));

  // Save loading of yaml config file
  try {
    YAML::Node config = YAML::LoadFile(scriptName);
    if (config["log_elements"].IsSequence())
    {
      YAML::Node logElementsNode = config["log_elements"];
      for( size_t i = 0; i < logElementsNode.size(); ++i)
      {
        if (YAML::Node parameter = logElementsNode[i]["name"])
        {
          std::string name = logElementsNode[i]["name"].as<std::string>();
          auto elem = logElements_.find(name);
          if(elem != logElements_.end())
          {
            // Erase the iterator offset that belongs to elem since it was found in the yaml
            iteratorOffsets.erase( std::distance(logElements_.begin(), elem) );

            // Overwrite defaults if specified in yaml file
            if (YAML::Node parameter = logElementsNode[i]["enabled"])
            {
              elem->second->setIsEnabled(parameter.as<bool>());
            }
            else {
              // Disable element if nothing is specified
              elem->second->setIsEnabled(false);
            }
            // Overwrite defaults if specified in yaml file
            if (YAML::Node parameter = logElementsNode[i]["divider"])
            {
              elem->second->getOptions().setDivider(parameter.as<int>());
            }
            // Check for action
            if (YAML::Node parameter = logElementsNode[i]["action"])
            {
              elem->second->getOptions().setAction( static_cast<LogElementAction>(parameter.as<int>()) );
            }
            // Check for buffer size
            if (YAML::Node parameter = logElementsNode[i]["buffer"]["size"])
            {
              elem->second->getBuffer().setBufferSize(parameter.as<int>());
            }
            // Check for buffer looping
            if (YAML::Node parameter = logElementsNode[i]["buffer"]["type"])
            {
              elem->second->getBuffer().setBufferType( static_cast<BufferType>(parameter.as<int>()) );
            }
            // Insert element
            if(elem->second->isEnabled()) {
              enabledElements_.push_back(elem);
            }
          }
          else {
            MELO_DEBUG_STREAM("[Signal logger] Could not load " << name << " from config file for logger '" <<
                              options_.loggerName_ << "'. Var not logged.");
          }
        }
        else {
          MELO_DEBUG_STREAM("[Signal logger] Could not load get name from config file for logger '" <<
                            options_.loggerName_ << "'. Ignore entry nr: " << i << "!");
        }
      }
    }
    else {
      MELO_DEBUG("[Signal logger] Param file of logger '%s' is ill-formatted. Log elements is no sequence.", options_.loggerName_.c_str());
    }
  }
  catch(YAML::Exception & e) {
    MELO_ERROR_STREAM("[Signal logger] Could not load config file for logger '" << options_.loggerName_ <<
                      "', because exception occurred: "<< e.what());
    return false;
  }

  // Noisily add all elements that were not in the configuration file
  for(auto iteratorOffset : iteratorOffsets) {
    LogElementMapIterator elem = std::next(logElements_.begin(), iteratorOffset);
    elem->second->setIsEnabled(true);
    enabledElements_.push_back(elem);
    MELO_DEBUG("[Signal logger] Enable logger element %s. It was not specified in the logger file of logger '%s'!", elem->first.c_str(),
               options_.loggerName_.c_str());
  }

  return true;
}

bool SignalLoggerBase::saveDataCollectScript(const std::string & scriptName)
{
  if(!isInitialized_ || isSavingData_)
  {
    MELO_WARN("[Signal logger] Could not save data collect script for logger '%s'! %s%s", options_.loggerName_.c_str(),
              isCollectingData_?" Collecting data!":"", !isInitialized_?" Not initialized!":"");
    return false;
  }

  // Check filename size and ending
  std::string ending = ".yaml";
  if ( ( (ending.size() + 1) > scriptName.size() ) || !std::equal(ending.rbegin(), ending.rend(), scriptName.rbegin()) ) {
    MELO_ERROR("[Signal logger] Script for logger '%s' must be a yaml file : *.yaml", options_.loggerName_.c_str());
    return false;
  }

  // Push back all data to node
  YAML::Node node;
  std::size_t j = 0;

  // Update logger owns a unique lock for the log element map -> thread-safe
  for (auto & elem : logElements_)
  {
    node["log_elements"][j]["name"] = elem.second->getOptions().getName();
    node["log_elements"][j]["enabled"] = elem.second->isEnabled();
    node["log_elements"][j]["divider"] = elem.second->getOptions().getDivider();
    node["log_elements"][j]["action"] = static_cast<int>(elem.second->getOptions().getAction());
    node["log_elements"][j]["buffer"]["size"] = elem.second->getBuffer().getBufferSize();
    node["log_elements"][j]["buffer"]["type"] = static_cast<int>(elem.second->getBuffer().getBufferType());
    j++;
  }

  // If there are logged elements save them to file
  if(j != 0) {
    std::ofstream outfile(scriptName);
    yaml_helper::writeYamlOrderedMaps(outfile, node);
    outfile.close();
  }

  return (j != 0);
}

void SignalLoggerBase::initTimeLogElement() {
  // Reset time element
  LogElementOptions options(options_.loggerPrefix_ + std::string{"/time"}, "[s/ns]", 1, LogElementAction::SAVE);
  timeElement_.reset( new LogElementBase<TimestampPair>( &logTime_, BufferType::FIXED_SIZE, 0, std::move(options) ) );
}

signal_logger::TimestampPair SignalLoggerBase::getCurrentTime() {
  signal_logger::TimestampPair timeStamp;

  // Get time in seconds and nanoseconds
  auto duration = std::chrono::system_clock::now().time_since_epoch();
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
  timeStamp.first = seconds.count();
  timeStamp.second = std::chrono::duration_cast<std::chrono::nanoseconds>(duration-seconds).count();

  return timeStamp;
}

bool SignalLoggerBase::workerSaveDataWrapper(const LogFileTypeSet & logfileTypes) {
  //-- Produce file name, this can be done in series to every other process

  // Read suffix number from file
  int suffixNumber = 0;
  std::string fileNumberFile = "." + options_.loggerName_ + "_fileNumber";
  std::ifstream ifs(fileNumberFile, std::ifstream::in);
  if(ifs.is_open()) { ifs >> suffixNumber; }
  ifs.close();
  ++suffixNumber;

  // To string with format 00000 (e.g 00223)
  std::string suffixString = std::to_string(suffixNumber);
  while(suffixString.length() != 5 ) { suffixString.insert(0, "0"); }

  // Get local time
  char dateTime[21];
  std::time_t now = std::chrono::system_clock::to_time_t ( std::chrono::system_clock::now() );
  strftime(dateTime, sizeof dateTime, "%Y%m%d_%H-%M-%S_", std::localtime(&now));

  // Filename format (e.g. #loggerName_13Sep2016_12-13-49_00011)
  std::string filename = options_.loggerName_ + "_" + std::string{dateTime} + suffixString;

  {
    // Lock the logger (blocking!)
    boost::unique_lock<boost::shared_mutex> workerSaveDataWrapperLock(loggerMutex_);

    // Copy general stuff
    noCollectDataCallsCopy_ = noCollectDataCalls_.load();

    // Copy data from buffer
    for(auto & elem : enabledElements_)
    {
      if(elem->second->getOptions().isSaved())
      {
        elem->second->copy();
      }
    }

    // Copy time
    timeElement_->copy();

    // Reset buffers and counters
    noCollectDataCalls_ = 0;

    // Clear buffer for log elements
    for(auto & elem : logElements_) {
      elem.second->reset();
    }

    // Clear buffer for time elements
    timeElement_->reset();

    // Set flag -> collection can restart
    isCopyingBuffer_ = false;
  }

  // Start saving copy to file
  bool success = this->workerSaveData(filename, logfileTypes);

  // Set flag, notify user
  isSavingData_ = false;

  if(success) {
    // Write next suffix number to file
    std::ofstream ofs(fileNumberFile, std::ofstream::out | std::ofstream::trunc);
    if(ofs.is_open()) { ofs << suffixNumber; }
    ofs.close();

    MELO_INFO_STREAM( "[Signal logger] All done, captain! Stored logging data for logger '" << options_.loggerName_ << "' to file " << filename );
  }

  return success;
}

bool SignalLoggerBase::workerStartLogger(bool updateLogger) {
  isStarting_ = true;

  while(true) {
    if(!isCopyingBuffer_ && (!updateLogger || !isSavingData_)) {
      isStarting_ = false;
      MELO_INFO("[Signal logger] Delayed logger start of logger '%s'!", options_.loggerName_.c_str());
      return startLogger(updateLogger);
    }
    // Sleep for one timestep (init is only allowed once. access of updateFrequency is ok)
    usleep( (1.0/options_.updateFrequency_) * 1e6 );
  }
}

}
