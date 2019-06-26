/*
 * Copyright (c) 2014, Christian Gehring
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL  Christian Gehring BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*!
 * @file    ControllerManager.hpp
 * @author  Christian Gehring, Gabriel Hottiger
 * @date    Oct, 2014
 */

// rocoma
#include "rocoma/ControllerManager.hpp"

// Message logger
#include "message_logger/message_logger.hpp"

// STL
#include <limits>
#include <algorithm>

namespace rocoma {

ControllerManager::ControllerManager():
    ControllerManager(ControllerManagerOptions{})
{
  // Hack
  isInitialized_ = false;
}

ControllerManager::ControllerManager(const ControllerManagerOptions & options):
  isInitialized_(true),
  options_(options),
  state_{State::FAILURE},
  clearedEmergencyStop_{!options.emergencyStopMustBeCleared},
  workerManager_(),
  controllers_(),
  emergencyControllers_(),
  sharedModules_(),
  controllerPairs_(),
  activeControllerPair_(nullptr, nullptr),
  failproofController_(nullptr),
  controllerMutex_(),
  emergencyStopMutex_(),
  updateControllerMutex_(),
  switchControllerMutex_()
{

}

void ControllerManager::init(const ControllerManagerOptions & options)
{
  if (isInitialized_) {
    MELO_WARN("[Rocoma] Controller Manager was already initialized. Do nothing.");
    return;
  }
  options_ = options;
  clearedEmergencyStop_ = !options.emergencyStopMustBeCleared;

  isInitialized_ = true;
}

bool ControllerManager::addControllerPair(ControllerPtr&& controller,
                                          EmgcyControllerPtr&& emergencyController) {

  //--- Add controller
  if(!checkInitializationAndFailproofController("Could not add controller pair.") || controller == nullptr || !createController(controller)) {
    return false;
  }

  // Local helpers (controllers are moved and therefore not safe to access)
  const std::string controllerName = controller->getControllerName();

  // insert controller (move ownership to controller / controller is set to nullptr)
  controllers_.insert( std::make_pair(controllerName, std::move(controller) ) );
  MELO_DEBUG_STREAM("[Rocoma][" << controllerName << "] Successfully added controller!");

  //--- Add emergency controller
  const std::string emgcyControllerName =
          emergencyController == nullptr ? failproofController_->getControllerName() : emergencyController->getControllerName();
  if(emergencyController == nullptr)
  {
    controllerPairs_.insert( std::make_pair( controllerName, ControllerSetPtr(controllers_.at(controllerName).get(), nullptr ) ) );
    MELO_INFO_STREAM("[Rocoma][" << controllerName << " / " << emgcyControllerName << "] Successfully added controller pair.");
    return true;
  }

  // set properties
  emergencyController->setIsRealRobot(options_.isRealRobot);

  // check if emergency controller already exists
  if(emergencyControllers_.find(emgcyControllerName) != emergencyControllers_.end()) {
    MELO_INFO_STREAM("[Rocoma][" << emgcyControllerName << "] An emergency controller with the name already exists. Using same instance.");
  }
  else {
    // create emergency controller
    if (!emergencyController->createController(options_.timeStep)) {
      MELO_WARN_STREAM("[Rocoma][" << emgcyControllerName << "] Could not be created! Use failproof controller on emergency stop!");
      controllerPairs_.insert( std::make_pair( controllerName, ControllerSetPtr(controllers_.at(controllerName).get(), nullptr) ) );
      return false;
    }

    // insert emergency controller (move ownership to controller / controller is set to nullptr)
    emergencyControllers_.insert( std::make_pair(emgcyControllerName, std::move(emergencyController) ) );
    MELO_DEBUG_STREAM("[Rocoma][" << emgcyControllerName << "] Successfully added emergency controller!");
  }

  // Add controller pair
  controllerPairs_.insert(std::make_pair(
          controllerName, ControllerSetPtr(controllers_.at(controllerName).get(), emergencyControllers_.at(emgcyControllerName).get() ) ) );
  MELO_INFO_STREAM("[Rocoma][" << controllerName << " / " << emgcyControllerName << "] Successfully added controller pair.");

  return true;
}

bool ControllerManager::addControllerPairWithExistingEmergencyController(ControllerPtr&& controller,
                                                                         const std::string & emgcyControllerName)
{
  // Add controller
  if(!checkInitializationAndFailproofController("Can not add controller with existing emergency controller.") || controller == nullptr ||  !createController(controller)) {
    return false;
  }

  // Local helpers (controllers are moved and therefore not safe to access)
  const std::string controllerName = controller->getControllerName();

  // insert controller (move ownership to controller / controller is set to nullptr)
  controllers_.insert( std::make_pair(controllerName, std::move(controller) ) );
  MELO_DEBUG_STREAM("[Rocoma][" << controllerName << "] Successfully added controller!");

  // check if emergency controller already exists
  if(emergencyControllers_.find(emgcyControllerName) != emergencyControllers_.end()) {
    MELO_INFO_STREAM("[Rocoma][" << emgcyControllerName << "] An emergency controller with the name already exists. Using same instance.");
    controllerPairs_.insert( std::make_pair( controllerName,
            ControllerSetPtr(controllers_.at(controllerName).get(), emergencyControllers_.at(emgcyControllerName).get()) ) );
    MELO_INFO_STREAM("[Rocoma][" << controllerName << " / " << emgcyControllerName << "] Successfully added controller pair.");
  }
  else {
    MELO_WARN_STREAM("[Rocoma][" << emgcyControllerName << "] Does not exist in list! Use failproof controller on emergency stop!");
    controllerPairs_.insert( std::make_pair( controllerName, ControllerSetPtr(controllers_.at(controllerName).get(), nullptr) ) );
    MELO_INFO_STREAM("[Rocoma][" << controllerName << " / ] Successfully added controller pair.");
  }

  return true;
}


bool ControllerManager::setFailproofController(FailproofControllerPtr&& controller)
{
  if(!isInitialized_ || controller == nullptr) {
    MELO_ERROR("[Rocoma] Could not set failproof controller. Abort!");
    exit(-1);
  }

  // controller name (controller is moved within this function, therefore not safe to access everywhere)
  std::string controllerName = controller->getControllerName();
  MELO_DEBUG_STREAM("[Rocoma][" << controllerName << "] Adding failproof controller!");

  // create controller
  if (!controller->createController(options_.timeStep)) {
    MELO_ERROR_STREAM("[Rocoma][" << controllerName << "] Could not create failproof controller. Abort!");
    exit(-1);
  }

  // move controller
  failproofController_ = std::move(controller);
  MELO_INFO_STREAM("[Rocoma][" << controllerName << "] Successfully added failproof controller!");

  return true;
}

bool ControllerManager::updateController() {
  // Calls to updateController are queued
  std::unique_lock<std::mutex> lockUpdate(updateControllerMutex_);
  if(!checkInitializationAndFailproofController("Can not advance controller manager.")) {
    return false;
  }


  // Run controller
  bool successfullyAdvanced = false;
  {
    boost::shared_lock<boost::shared_mutex> lockControllersForAdvance(controllerMutex_);
    if(state_ == State::OK) {
      successfullyAdvanced = activeControllerPair_.controller_->advanceController(options_.timeStep);
    } else if(state_ == State::EMERGENCY) {
      successfullyAdvanced = activeControllerPair_.emgcyController_->advanceController(options_.timeStep);
    } else if(state_ == State::FAILURE) {
      failproofController_->advanceController(options_.timeStep);
      successfullyAdvanced = true;
    }
  }

  // E-stop if advance returned false
  return (successfullyAdvanced ? true : emergencyStop());
}

bool ControllerManager::emergencyStop(EmergencyStopType eStopType) {
  // A list of controllers that were running during the estop procedure (can be both if emgcy controller fails)
  std::vector<roco::ControllerAdapterInterface*> controllersToStop;
  std::string newControllerName;

  // This section can only be executed simultaneously once!
  {
    // Cannot call emergency stop twice simultaneously
    std::unique_lock<std::mutex> lockEmergencyStop(emergencyStopMutex_);

    // Only E-stop holds upgradable lock controllers
    boost::upgrade_lock<boost::shared_mutex> lockControllers(controllerMutex_);

    // Set flag that emergency stop occurred
    if (options_.emergencyStopMustBeCleared) {
      clearedEmergencyStop_ = false;
    }

    // Failproof estop if no emgcy controller available
    const bool emgcyStopControllerNotAvailable = activeControllerPair_.emgcyController_ == nullptr ||
                                                 activeControllerPair_.emgcyController_->isBeingStopped();
    const bool inEmergencyOrFailproof = (state_ == State::EMERGENCY) || (state_ == State::FAILURE);

    if (inEmergencyOrFailproof || emgcyStopControllerNotAvailable) {
      eStopType = EmergencyStopType::FAILPROOF;
    }

    // Notify emergency stop
    MELO_ERROR_STREAM("[Rocoma] " << (eStopType == EmergencyStopType::FAILPROOF ? "Failproof" : "Emergency") << " Stop!");
    notifyEmergencyStop(eStopType);

    // Check if controller is in failproof state already
    if (state_ == State::FAILURE) {
      MELO_DEBUG("[Rocoma] Failproof controller is already running on emergency stop!");
      this->notifyControllerManagerStateChanged(state_, clearedEmergencyStop_);
      return true;
    }

    // Stop logger and save logger data (Saving in separate thread)
    if (options_.loggerOptions.enable) {
      signal_logger::logger->stopAndSaveLoggerData(options_.loggerOptions.fileTypes);
    }

    // If state ok and emergency controller registered -> try to switch to emergency controller
    if (state_ == State::OK) {
      // Add to controllers that must be stopped
      controllersToStop.push_back(activeControllerPair_.controller_);

      if (eStopType == EmergencyStopType::EMERGENCY) {
        // Init emergency controller fast and only advance if correctly initialized
        if (activeControllerPair_.emgcyController_->initializeControllerFast(options_.timeStep) &&
            activeControllerPair_.emgcyController_->advanceController(options_.timeStep)) {
          activeControllerPair_.controller_->setIsRunning(false);
          activeControllerPair_.emgcyController_->setIsRunning(true);
          newControllerName = activeControllerPair_.emgcyControllerName_;
          {
            // Switch to emergency state
            boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLockControllers(lockControllers);
            state_ = State::EMERGENCY;
          }
          // Start logger
          if (options_.loggerOptions.enable) {
            signal_logger::logger->startLogger(options_.loggerOptions.updateOnStart);
          }
        } else {
          // No success, move on to failproof controller
          eStopType = EmergencyStopType::FAILPROOF;
          controllersToStop.push_back(activeControllerPair_.emgcyController_);
        }
      }
    }

    if (eStopType == EmergencyStopType::FAILPROOF) {
      if(state_ == State::EMERGENCY) {
        controllersToStop.push_back(activeControllerPair_.emgcyController_);
      }

      // Add to controllers that must be stopped
      activeControllerPair_.controller_->setIsRunning(false);
      if (activeControllerPair_.emgcyController_ != nullptr) {
        activeControllerPair_.emgcyController_->setIsRunning(false);
      }

      // Switch to failure state
      {
        boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLockControllers(lockControllers);
        state_ = State::FAILURE;
      }

      // Advance failproof controller
      {
        MELO_INFO("[Rocoma] Switched to failproof controller!");
        failproofController_->advanceController(options_.timeStep);
        newControllerName = failproofController_->getControllerName();
      }
    }
  }

  // Notify caller
  this->notifyControllerChanged(newControllerName);
  this->notifyControllerManagerStateChanged(state_, clearedEmergencyStop_);

  // Stop running controllers
  for(auto ctrl : controllersToStop) {
    this->stopController(ctrl);
  }

  return true;
}

void ControllerManager::clearEmergencyStop() {
  //! Important order (deadlocks!!!)
  std::unique_lock<std::mutex> lockEmergencyStop(emergencyStopMutex_);
  boost::shared_lock<boost::shared_mutex> lockControllers(controllerMutex_);

  if(!clearedEmergencyStop_) {
    clearedEmergencyStop_ = true;
    MELO_INFO("[Rocoma] Cleared Emergency Stop.");
    notifyControllerManagerStateChanged(state_, clearedEmergencyStop_);
  }
}

bool ControllerManager::hasClearedEmergencyStop() const {
  //! Important order (deadlocks!!!)
  std::unique_lock<std::mutex> lockEmergencyStop(emergencyStopMutex_);
  return clearedEmergencyStop_;
}

ControllerManager::SwitchResponse ControllerManager::switchController(const std::string & controllerName) {
  // init promise and future
  std::promise<SwitchResponse> switch_promise;
  std::future<SwitchResponse> switch_future = switch_promise.get_future();

  // switch controller
  this->switchController(controllerName, std::ref(switch_promise));

  // wait for future result
  switch_future.wait();

  return switch_future.get();
}

void ControllerManager::switchController(const std::string & controllerName,
                                         std::promise<SwitchResponse> & response_promise) {

  // Allow only sequential calls to switch controller
  std::unique_lock<std::mutex> lockSwitchController(switchControllerMutex_, std::try_to_lock);
  if(!lockSwitchController.owns_lock()){
    MELO_ERROR_STREAM("[Rocoma] Can not switch controller! Already switching!");
    response_promise.set_value(SwitchResponse::ERROR);
    return;
  }

  // Emergency stop must be cleared
  if(!hasClearedEmergencyStop()) {
    MELO_ERROR_STREAM("[Rocoma] Can not switch controller! Emergency stop was not cleared!");
    response_promise.set_value(SwitchResponse::ERROR);
    return;
  }

  // Make sure were not in emergency stop procedure when getting state
  State currentState;
  {
    std::unique_lock<std::mutex> lockEmergencyStop(emergencyStopMutex_);
    boost::shared_lock<boost::shared_mutex> lockControllers(controllerMutex_);
    currentState = state_;
  }

  // Check if controller is already active
  {
    if (currentState == State::OK && controllerName == activeControllerPair_.controllerName_) {
      MELO_INFO("[Rocoma] Controller %s is already running!", controllerName.c_str());
      response_promise.set_value(SwitchResponse::RUNNING);
      return;
    }
  }

  // Find controller
  auto controllerPair = controllerPairs_.find(controllerName);
  if(controllerPair != controllerPairs_.end()) {
    // Switch controller worker
    any_worker::WorkerOptions switchControllerWorkerOptions;
    switchControllerWorkerOptions.timeStep_ = std::numeric_limits<double>::infinity();
    switchControllerWorkerOptions.defaultPriority_ = 0;
    switchControllerWorkerOptions.destructWhenDone_ = true;

    // Define callback name and controllers to be switched
    switch(currentState) {
      case State::OK:
      {
        this->switchFromOldToNewController(activeControllerPair_.controller_, controllerPair->second.controller_, currentState, response_promise);
        break;
      }
      case State::EMERGENCY:
      {
        this->switchFromOldToNewController(activeControllerPair_.emgcyController_, controllerPair->second.controller_, currentState, response_promise);
        break;
      }
      case State::FAILURE:
      {
        this->switchFromOldToNewController(nullptr, controllerPair->second.controller_, currentState, response_promise);
        break;
      }
      case State::NA:
      {
        MELO_ERROR_STREAM("[Rocoma] Can not switch controller! In State NA, should never happen!");
        response_promise.set_value(SwitchResponse::NA);
        return;
      }
    }
    return;
  }
  else {
    // controller is not part of controller map
    MELO_INFO("[Rocoma] Controller %s not found!", controllerName.c_str());
    response_promise.set_value(SwitchResponse::NOTFOUND);
    return;
  }
}

std::vector<std::string> ControllerManager::getAvailableControllerNames() const {
  // Fill vector of controller names
  std::vector<std::string> controllerNames;
  for( auto & controller : controllers_ )
  {
    controllerNames.push_back(controller.first);
  }

  // Sort them alphabetically
  std::sort(controllerNames.begin(), controllerNames.end(), std::less<std::string>());

  return controllerNames;
}

std::string ControllerManager::getActiveControllerName() const {
  std::string controllerName = "-";
  boost::shared_lock<boost::shared_mutex> lockControllers(controllerMutex_);

  switch (state_) {
    case State::OK: {
      controllerName = activeControllerPair_.controllerName_;
      break;
    }
    case State::EMERGENCY: {
      controllerName = activeControllerPair_.emgcyControllerName_;
      break;
    }
    case State::FAILURE: {
      controllerName = failproofController_ != nullptr ? failproofController_->getControllerName() : "";
      break;
    }
    case State::NA: {
      controllerName = "Not available";
      break;
    }
  }

  return controllerName;
}

ControllerManager::State ControllerManager::getControllerManagerState() const {
  boost::shared_lock<boost::shared_mutex> lockControllers(controllerMutex_);
  return state_;
}

bool ControllerManager::cleanup() {
  bool success = true;

  // Move to failproof controller
  std::unique_lock<std::mutex> lockEmergencyController(emergencyStopMutex_);
  if(state_ != State::FAILURE) {
    lockEmergencyController.unlock();
    success = failproofStop();
  }

  boost::unique_lock<boost::shared_mutex> lockControllers(controllerMutex_);

  // stop all workers
  MELO_DEBUG("[Rocoma] Stopping all workers.");
  workerManager_.stopWorkers(true);

  // cleanup all controllers
  // TODO wait for controllers to be finished initializing
  MELO_DEBUG("[Rocoma] Cleaning all controllers up.");
  for (auto& controller : controllers_) {
    while(controller.second->isBeingStopped()) { MELO_INFO_THROTTLE_STREAM(1.0, "Stopping controller " << controller.first ); }
    success = controller.second->cleanupController() && success;
    // clean up unique ptrs here.
    // They are managed by ControllerManager and are pointing to instances classes found in dynamically loaded libraries.
    // The libraries are loaded and managed by the child class ControllerManagerRos. The destructor of ControllerManagerRos is called before
    // the destructor of ControllerManager, cleaning up the loaded libraries. This leaves these unique_ptrs pointing to an instance of an unknown class.
    controller.second.reset(nullptr);
  }

  MELO_DEBUG("[Rocoma] Cleaning all emergency controllers up.");
  for (auto& emergency_controller : emergencyControllers_ ) {
    while(emergency_controller.second->isBeingStopped()) { }
    success = emergency_controller.second->cleanupController() && success;
    emergency_controller.second.reset(nullptr); // clean up unique ptrs here, see above
  }

  MELO_DEBUG("[Rocoma] Reset fail proof controller.");
  failproofController_->cleanupController();
  failproofController_.reset(nullptr); // clean up unique ptrs here, see above

  return success;
}

bool ControllerManager::createController(const ControllerPtr & controller) {

  // Check for invalid controller
  if(controller == nullptr) {
    MELO_ERROR_STREAM("[Rocoma] Could not add controller pair. Controller is nullptr.");
    return false;
  }

  // Local helpers (controllers are moved and therefore not safe to access)
  const std::string controllerName = controller->getControllerName();

  // check if controller already exists
  if(controllers_.find(controllerName) != controllers_.end()) {
    MELO_WARN_STREAM("[Rocoma][" << controllerName << "] Could not add controller. A controller with the same name already exists.");
    return false;
  }

  //--- Add controller
  MELO_DEBUG_STREAM("[Rocoma][" << controllerName << "] Adding controller!");

  // Set controller properties
  controller->setIsRealRobot(options_.isRealRobot);

  // create controller
  if (!controller->createController(options_.timeStep)) {
    MELO_ERROR_STREAM("[Rocoma][" << controllerName << "] Could not create controller!");
    return false;
  }

  return true;
}

bool ControllerManager::stopController(roco::ControllerAdapterInterface * controller)
{
  bool success = true;

  if(!controller->isBeingStopped()) {
    // Stop controller and block -> switch controller can not happen while controller is stopped
    controller->setIsBeingStopped(true);
    success = controller->preStopController();
    success = controller->stopController() && success;
    controller->setIsBeingStopped(false);
  }

  return success;
}

bool ControllerManager::switchFromOldToNewController(roco::ControllerAdapterInterface * oldController,
                                                     roco::ControllerAdapterInterface * newController,
                                                     State previousState,
                                                     std::promise<SwitchResponse> & response_promise) {
  /** NOTE:
   * 1. The active controller is not blocked -> by definition there can be no data races between advance and preStop
   */
  // shutdown communication for active controller
  if(oldController != nullptr) {
    oldController->setIsBeingStopped(true);
    if(!oldController->preStopController()) {
      emergencyStop(); // Estop will not stop this controller
      oldController->stopController();
      oldController->setIsBeingStopped(false);
      response_promise.set_value(SwitchResponse::ERROR);
      return false;
    }
  }

  // Stop logger if running
  if(options_.loggerOptions.enable && signal_logger::logger->isRunning()) {
    signal_logger::logger->stopAndSaveLoggerData(options_.loggerOptions.fileTypes);
  }

  /** NOTE:
   * 1. newController is not running (we would have returned in switchController already)
   * 2. newController can not be an emergency controller of the currently running controller
   * 3. newController could be being stopped by a different thread at the moment (wait for completion)
   */
  if(newController->isBeingStopped()) {
    MELO_WARN_STREAM("[Rocoma][" << newController->getControllerName() <<
                                 "] Controller is currently being stopped. Wait for completion before switching.")
  }
  while(newController->isBeingStopped()){}

  //! initialize new controller
  roco::ControllerSwapStateInterfacePtr state(nullptr);
  if(oldController != nullptr) { oldController->getControllerSwapState(state); }

  if(!newController->swapController(options_.timeStep, state)) {
    emergencyStop(); // Estop will not stop this controller
    if(oldController != nullptr) {
      oldController->stopController();
      oldController->setIsBeingStopped(false);
    }

    // Note controller will continue to run in prestop mode, assuming that this is still valid
    // Stop new controller could have messed up internal state
    newController->preStopController();
    newController->stopController();

    // Start Logging
    if(options_.loggerOptions.enable){
      signal_logger::logger->startLogger(options_.loggerOptions.updateOnStart);
    }

    MELO_ERROR_STREAM("[Rocoma][" << newController->getControllerName() << "] Could not swap. E-stop.");
    response_promise.set_value(SwitchResponse::ERROR);
    return false;
  }

  // Start Logging
  if(options_.loggerOptions.enable){
    signal_logger::logger->startLogger(options_.loggerOptions.updateOnStart);
  }

  // Set the newController as active controller as soon as the controller is initialized
  if ( newController->isControllerInitialized() ) {
    {
      //! This step has to be done when no update nor emergency stop is performed
      boost::unique_lock<boost::shared_mutex> lockControllers(controllerMutex_);
      if(state_ == previousState) {
        // Protect also service calls accessing the active controller pair at the same time
        if(oldController != nullptr) { oldController->setIsRunning(false); }
        newController->setIsRunning(true);
        activeControllerPair_ = controllerPairs_.at(newController->getControllerName());
        state_ = State::OK;
        MELO_INFO("[Rocoma] Switched to controller %s", activeControllerPair_.controllerName_.c_str());
      } else {
        lockControllers.unlock();
        MELO_ERROR_STREAM("[Rocoma][" << newController->getControllerName() << "] Could not switch. Emergency stop detected.");

        // Stop old controller
        if(oldController != nullptr) {
          oldController->stopController();
          oldController->setIsBeingStopped(false);
        }
        // Stop the new controller
        newController->preStopController();
        newController->stopController();

        response_promise.set_value(SwitchResponse::ERROR);
        return false;
      }
    }

    this->notifyControllerChanged(activeControllerPair_.controllerName_);
    this->notifyControllerManagerStateChanged(State::OK, clearedEmergencyStop_);

    // stop old controller
    if(oldController != nullptr) {
      oldController->stopController();
      oldController->setIsBeingStopped(false);
    }

    response_promise.set_value(SwitchResponse::SWITCHING);
    return true;
  }
  else {
    // Stop the new controller
    newController->preStopController();
    newController->stopController();
    // switch to freeze controller
    emergencyStop(); // Estop will not stop this controller
    if(oldController != nullptr) {
      oldController->stopController();
      oldController->setIsBeingStopped(false);
    }
    MELO_ERROR_STREAM("[Rocoma][" << newController->getControllerName() <<
                                  "] Controller initialization was unsuccessful. Not switching.");
    response_promise.set_value(SwitchResponse::ERROR);
    return false;
  }
}

bool ControllerManager::addSharedModule(roco::SharedModulePtr&& sharedModule) {
  std::string name = sharedModule->getName();

  if( hasSharedModule(name) ) {
    MELO_WARN_STREAM("[Rocoma][" << name << "] A shared module with this name was already added! Do nothing.");
    return true;
  }
  sharedModules_.emplace(name, std::move(sharedModule));
  return true;
}

bool ControllerManager::hasSharedModule(const std::string & moduleName) const {
  return sharedModules_.find(moduleName) != sharedModules_.end();
}

bool ControllerManager::checkInitializationAndFailproofController(const char* message) const {
  if(!isInitialized_) {
    MELO_ERROR("[Rocoma] Controller Manager is not initialized. %s", message);
    return false;
  }
  if(failproofController_ == nullptr) {
    MELO_ERROR("[Rocoma] Failproof controller is null. %s", message);
    return false;
  }
  return true;
}


} /* namespace rocoma */
