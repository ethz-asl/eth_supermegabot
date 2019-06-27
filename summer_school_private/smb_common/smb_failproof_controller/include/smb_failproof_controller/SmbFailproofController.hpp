/*!
* @file 	  SmbFailproofController.hpp
* @author   Johannes Pankert
* @date		  09/04/2019
* @version 	1.0
* @brief    A controller that ...
*/

#pragma once

// roco
#include "roco/controllers/controllers.hpp"

// state and command
#include "smb_roco/RocoState.hpp"
#include "smb_roco/RocoCommand.hpp"

namespace smb_failproof_controller {

class SmbFailproofController: virtual public roco::FailproofController<smb_roco::RocoState, smb_roco::RocoCommand> {

 public:
  typedef roco::FailproofController<smb_roco::RocoState, smb_roco::RocoCommand> Base;

  //! Construct SmbFailproofController.
  SmbFailproofController();

  //! Destruct SmbFailproofController.
  virtual ~SmbFailproofController() override;

 protected:
  virtual bool create(double dt) override;
  virtual void advance(double dt) override;
  virtual bool cleanup() override;


};

} /* namespace smb_failproof_controller */
