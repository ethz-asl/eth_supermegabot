/*!
* @file 	  SmbFailproofController.cpp
* @author   Johannes Pankert
* @date		  09/04/2019
* @version 	1.0
* @brief    A controller that ...
*/

// smb_failproof_controller
#include "smb_failproof_controller/SmbFailproofController.hpp"

// rocoma plugin
#include <rocoma_plugin/rocoma_plugin.hpp>

namespace smb_failproof_controller {

SmbFailproofController::SmbFailproofController()
    : Base()
{
  this->setName("SmbFailproofController");
}

SmbFailproofController::~SmbFailproofController() {

}

bool SmbFailproofController::create(double dt) {
  return true;
}

void SmbFailproofController::advance(double dt) {
  boost::lock_guard<boost::shared_mutex> lockGuard(getCommandMutex());
  getCommand().freeze();
  return;
}

bool SmbFailproofController::cleanup() {
  return true;
}


} /* namespace smb_failproof_controller */
