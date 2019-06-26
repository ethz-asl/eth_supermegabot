/*!
* @file 	  SmbEmergencyController.hpp
* @author   Johannes Pankert
* @date		  10/04/2019
* @version 	1.0
* @brief    A controller that ...
*/

#pragma once

// roco
#include "roco/controllers/controllers.hpp"
// state and command
#include "smb_roco/RocoState.hpp"
#include "smb_roco/RocoCommand.hpp"

namespace smb_emergency_controller {

class SmbEmergencyController: virtual public roco::Controller<smb_roco::RocoState, smb_roco::RocoCommand>, public roco::EmergencyControllerAdapteeInterface {

 public:
  typedef roco::Controller<smb_roco::RocoState, smb_roco::RocoCommand> Base;

  //! Construct SmbEmergencyController.
  SmbEmergencyController();

  //! Destruct SmbEmergencyController.
  virtual ~SmbEmergencyController();

 protected:
  //! Create controller SmbEmergencyController.
  virtual bool create(double dt);

  //! Initialize controller SmbEmergencyController.
  virtual bool initialize(double dt);

  //! Advance controller SmbEmergencyController.
  virtual bool advance(double dt);

  //! Reset controller SmbEmergencyController.
  virtual bool reset(double dt);

  //! Pre-stop controller SmbEmergencyController.
  virtual bool preStop();

  //! Stop controller SmbEmergencyController.
  virtual bool stop();

  //! Cleanup controller SmbEmergencyController.
  virtual bool cleanup();

  //! Swap to controller SmbEmergencyController with state 'swap'.
  virtual bool swap(double dt, const roco::ControllerSwapStateInterfacePtr& swapState);

  //! Get swap state 'swapState' of controller SmbEmergencyController.
  virtual bool getSwapState(roco::ControllerSwapStateInterfacePtr& swapState);

  //! Add shared module 'module' to controller SmbEmergencyController.
  virtual bool addSharedModule(const roco::SharedModulePtr& module);

  /**
   * @brief Initialize controller SmbEmergencyController fast. (computation time << dt)
   * @param dt  Controller time step
   */
  virtual bool initializeFast(double dt);


};

} /* namespace smb_emergency_controller */
