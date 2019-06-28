/*!
* @file 	  SmbEmergencyController.cpp
* @author   Johannes Pankert
* @date		  10/04/2019
* @version 	1.0
* @brief    A controller that ...
*/

// smb_emergency_controller
#include "smb_emergency_controller/SmbEmergencyController.hpp"

namespace smb_emergency_controller {

SmbEmergencyController::SmbEmergencyController()
    : Base()
{
  setName("SmbEmergencyController");
}

SmbEmergencyController::~SmbEmergencyController() {

}

bool SmbEmergencyController::create(double dt) {
  return true;
}

bool SmbEmergencyController::initialize(double dt) {
  return true;
}

bool SmbEmergencyController::advance(double dt) {
  boost::lock_guard<boost::shared_mutex> lockGuard(getCommandMutex());
  getCommand().freeze();
  return true;
}

bool SmbEmergencyController::reset(double dt) {
  return SmbEmergencyController::initialize(dt);
}

bool SmbEmergencyController::preStop() {
  return true;
}

bool SmbEmergencyController::stop() {
  return true;
}

bool SmbEmergencyController::cleanup() {
  return true;
}

bool SmbEmergencyController::swap(double dt, const roco::ControllerSwapStateInterfacePtr& swapState) {
  // Call current class reset / initialize
  return isInitialized() ? SmbEmergencyController::reset(dt) : SmbEmergencyController::initialize(dt);
}

bool SmbEmergencyController::getSwapState(roco::ControllerSwapStateInterfacePtr& swapState) {
  swapState.reset(nullptr);
  return true;
}

bool SmbEmergencyController::addSharedModule(const roco::SharedModulePtr& module) {
  return false;
}

bool SmbEmergencyController::initializeFast(double dt) {
  return true; 
}

} /* namespace smb_emergency_controller */
