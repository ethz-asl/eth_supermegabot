/*!
 * @file     SmbLowLevelControllerInterface.cpp
 * @author   Tim Sandy, Koen Kraemer
 * @date     Oct, 2018
 */

namespace smb_lowlevel_controller {

  template<typename ConcreteDescription_>
  SmbLowLevelControllerInterface<ConcreteDescription_>::SmbLowLevelControllerInterface()
  {

  }

  template<typename ConcreteDescription_>
  bool SmbLowLevelControllerInterface<ConcreteDescription_>::init(std::string port, ros::NodeHandle &nh, bool commandSmb)
  {
    std::unique_lock<std::mutex> lock(smbDriverMutex_);

    lastControlMode_ = SmbMode::FREEZE;

    try
    {
      // Initializes communication to the smb
      smb_ = std::make_shared<smb_driver::SmbController>(port, nh, 10, commandSmb);
      smb_->startAcquisition();
      smb_->setMode(smb_driver::CLOSED_LOOP_SPEED);
      lastControlMode_ = SmbMode::MODE_WHEEL_VELOCITY;

      MELO_INFO_STREAM("[SmbLowLevelControllerInterfaceInterface]: Initialized successfully.");
    }
    catch (...)
    {
      MELO_ERROR_STREAM("[SmbLowLevelControllerInterface]: Caught an unspecified exception.");
      return false;
    }

    return true;
  }

  template<typename ConcreteDescription_>
  bool SmbLowLevelControllerInterface<ConcreteDescription_>::updateCommands(const SmbCommandsShm& smbCommands)
  {
    std::unique_lock<std::mutex> lock(smbDriverMutex_);

    double leftWheelVelocity, rightWheelVelocity;
    smb_common::SmbModeType currentControlMode;
    bool firstActuator = true;
    for (const auto& actuatorKey : ConcreteDescription_::template getKeys<typename RT::SmbActuatorEnum >()) {
      const auto& actuatorEnum = actuatorKey.getEnum();
      const int actuatorIndex = actuatorKey.getId();
      const auto& smbActuatorCommand = smbCommands.wheelCommands_[actuatorEnum];

      if (!firstActuator && smbActuatorCommand.getMode() != currentControlMode) {
        MELO_ERROR("[SmbLowLevelControllerInterface] Actuator commands received with conflicting modes. The same mode should be commanded for all wheels.");
      }
      else if (firstActuator) {
        currentControlMode = smbActuatorCommand.getMode();

        if (currentControlMode != lastControlMode_) {
          //Set the desired command mode
          switch (currentControlMode) {
            case SmbMode::MODE_WHEEL_VELOCITY:
              smb_->setMode(smb_driver::CLOSED_LOOP_SPEED);
              break;
//          case SmbMode::MODE_WHEEL_TORQUE:
//            smb_->setMode(TORQUE);
//            break;
            case SmbMode::WHEEL_DC_CMD:
              smb_->setMode(smb_driver::OPEN_LOOP);
              break;
            default:
              MELO_WARN_THROTTLE(1.0, "[SmbLowLevelControllerInterface] Specified SmbMode cannot be handled by the controller. mode=%d", currentControlMode);
          }

          lastControlMode_ = currentControlMode;
        }
      }
      firstActuator = false;

      //Set freeze on both motors when a freeze command is first received
      if (currentControlMode == SmbMode::FREEZE) {
        smb_->setFreeze();
        break;
      }

      //Only use the front wheel commands since both front and back wheels are wired to the same motor controller output
      int wheelNumber;
      if (strcmp(actuatorKey.getName(),"LF_WHEEL") == 0) // || strcmp(actuatorKey.getName(),"LH_WHEEL") == 0 )
        wheelNumber = 1;
      else if (strcmp(actuatorKey.getName(),"RF_WHEEL") == 0) // || strcmp(actuatorKey.getName(),"RH_WHEEL") == 0 )
        wheelNumber = 2;
      else {
//      MELO_WARN_STREAM("[SmbLowLevelControllerInterface] Unidentified actuatorKey name found! " << actuatorKey.getName());
        continue;
      }

      switch(currentControlMode) {
        case SmbMode::MODE_WHEEL_VELOCITY:
          smb_->setVelocity(smbActuatorCommand.getWheelVelocity(), wheelNumber);
          break;
          //todo Torque command mode is diabled for now because the PI gains needs to be tuned separately for this mode and we can't yet set the gains through the Roboteq driver
//      case SmbMode::MODE_WHEEL_TORQUE:
//        smb_->setTorque(smbActuatorCommand.getWheelTorque(), wheelNumber);
//        break;
        case SmbMode::WHEEL_DC_CMD:
          smb_->setMotorPower(smbActuatorCommand.getCurrent(), wheelNumber);
          break;
        default:
          MELO_WARN("[SmbLowLevelControllerInterface] Specified SmbMode cannot be handled by controller. mode=%d", currentControlMode);
          break;
      }
    }
//  MELO_INFO_THROTTLE(1.0, "[SmbLowLevelControllerInterface] leftWheelVelocity=%f, rightWheelVelocity=%f", leftWheelVelocity, rightWheelVelocity);

    std::chrono::time_point<std::chrono::steady_clock> startSub, endSub;
    int64_t elapsedSub;
    double durationSubinterval;

    return true;
  }

  template<typename ConcreteDescription_>
  bool SmbLowLevelControllerInterface<ConcreteDescription_>::updateMeasurements(SmbReadingsShm& smbReadings)
  {
    std::unique_lock<std::mutex> lock(smbDriverMutex_);

    double leftWheelSpeed, rightWheelSpeed;
    if (!smb_->getWheelSpeeds(leftWheelSpeed, rightWheelSpeed, motorControllerTimeoutUs_)) {
      MELO_WARN_STREAM("[SmbLowLevelControllerInterface] Failed to get wheel speeds.");
      return false;
    }
//  smb_->getBatteryVoltage(baseBatteryVoltage, motorControllerTimeoutUs);

    bool isOk = true;
    const ros::Time rosStamp = ros::Time::now(); //todo Use timestamp from when the measurements were read in the driver?
    const any_measurements::Time stamp  = any_measurements_ros::fromRos(rosStamp);

    for (const auto& actuatorKey : ConcreteDescription_::template getKeys<typename RT::SmbActuatorEnum >()) {
      const auto& actuatorEnum = actuatorKey.getEnum();
      const int actuatorIndex = actuatorKey.getId();
      auto& smbActuatorReading = smbReadings.wheelReadings_[actuatorEnum];

      int wheelNumber;
      if (strcmp(actuatorKey.getName(),"LF_WHEEL") == 0 || strcmp(actuatorKey.getName(),"LH_WHEEL") == 0 )
        smbActuatorReading.setWheelVelocity(leftWheelSpeed);
      else if (strcmp(actuatorKey.getName(),"RF_WHEEL") == 0 || strcmp(actuatorKey.getName(),"RH_WHEEL") == 0 )
        smbActuatorReading.setWheelVelocity(rightWheelSpeed);
      else {
        MELO_WARN_STREAM("[SmbLowLevelControllerInterface] Unidentified actuatorKey name found when setting wheel speeds! " << actuatorKey.getName());
        continue;
      }
    }

    return true;
  }

  template<typename ConcreteDescription_>
  void SmbLowLevelControllerInterface<ConcreteDescription_>::preCleanup()
  {
    // Set the base estop
//  smb_->eStop(); //TODO what to do with this?
  }

  template<typename ConcreteDescription_>
  void SmbLowLevelControllerInterface<ConcreteDescription_>::cleanup()
  {

  }

  template<typename ConcreteDescription_>
  void SmbLowLevelControllerInterface<ConcreteDescription_>::setFreeze()
  {
    smb_->setFreeze();
  }

} // smb_lowlevel_controller

