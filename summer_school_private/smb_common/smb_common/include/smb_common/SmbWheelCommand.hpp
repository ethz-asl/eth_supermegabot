/*!
 * @file    SmbWheelCommand.hpp
 * @author  Koen Kraemer
 * @date    Aug 2, 2018
 *
 */
#pragma once

// c++
#include <limits>
#include <ostream>
#include <string>

// any measurements
#include <any_measurements/Time.hpp>

// smb_common
#include <smb_common/SmbModes.hpp>


namespace smb_common {


class SmbWheelCommand
{
public:
//  typedef int16_t SmbModeType;
//  enum SmbMode : SmbModeType {
//    MODE_WHEEL_VELOCITY                           = 0,  // Track wheel velocity
//    MODE_WHEEL_TORQUE                             = 1,  // Track wheel torque
//  };
//  typedef SmbMode Mode;
//  using Mode = smb_common::SmbMode 

public:
  SmbWheelCommand();
  SmbWheelCommand(SmbModeType mode);
  virtual ~SmbWheelCommand();

  const any_measurements::Time& getStamp() const;
  void setStamp(const any_measurements::Time& stamp);

  static std::string getModeName(SmbModeType mode);
  std::string getModeName() const;

  const SmbModeType& getMode() const;
  void setMode(const SmbModeType& mode);

  SmbMode getModeEnum() const;
  void setModeEnum(const SmbMode& modeEnum);

  const double& getCurrent() const;
  void setCurrent(double current);

  const double& getWheelVelocity() const;
  void setWheelVelocity(double wheelVelocity);

  const double& getWheelTorque() const;
  void setWheelTorque(double wheelTorque);

  // const double& getCurrentMin() const;
  // void setCurrentMin(double currentMin);
  // const double& getCurrentMax() const;
  // void setCurrentMax(double currentMax);

  // const double& getMotorPositionMin() const;
  // void setMotorPositionMin(double motorPositionMin);
  // const double& getMotorPositionMax() const;
  // void setMotorPositionMax(double motorPositionMax);

  // const double& getMotorVelocityMin() const;
  // void setMotorVelocityMin(double motorVelocityMin);
  // const double& getMotorVelocityMax() const;
  // void setMotorVelocityMax(double motorVelocityMax);

  // const double& getGearPositionMin() const;
  // void setGearPositionMin(double gearPositionMin);
  // const double& getGearPositionMax() const;
  // void setGearPositionMax(double gearPositionMax);

  // const double& getGearVelocityMin() const;
  // void setGearVelocityMin(double gearVelocityMin);
  // const double& getGearVelocityMax() const;
  // void setGearVelocityMax(double gearVelocityMax);

  // const double& getJointPositionMin() const;
  // void setJointPositionMin(double jointPositionMin);
  // const double& getJointPositionMax() const;
  // void setJointPositionMax(double jointPositionMax);

  // const double& getJointVelocityMin() const;
  // void setJointVelocityMin(double jointVelocityMin);
  // const double& getJointVelocityMax() const;
  // void setJointVelocityMax(double jointVelocityMax);

  // const double& getJointTorqueMin() const;
  // void setJointTorqueMin(double jointTorqueMin);
  // const double& getJointTorqueMax() const;
  // void setJointTorqueMax(double jointTorqueMax);


  //! Limits all values.
  void limit();

  //! @returns true if all values are finite.
  bool isFinite() const;
  //! @returns true if all values within the limits.
  bool isWithinLimits() const;
  //! @returns true if all values are finite and within the limits.
  bool isValid() const;

  friend std::ostream& operator<<(std::ostream& out, const SmbWheelCommand& command);

protected:
  any_measurements::Time stamp_;

  SmbModeType mode_ = SmbMode::FREEZE;

  double current_ = 0.0;
  double wheelVelocity_ = 0.0;
  double wheelTorque_ = 0.0;

  // double currentMin_ = -std::numeric_limits<double>::max();
  // double currentMax_ = std::numeric_limits<double>::max();
  // double motorPositionMin_ = -std::numeric_limits<double>::max();
  // double motorPositionMax_ = std::numeric_limits<double>::max();
  // double motorVelocityMin_ = -std::numeric_limits<double>::max();
  // double motorVelocityMax_ = std::numeric_limits<double>::max();
  // double gearPositionMin_ = -std::numeric_limits<double>::max();
  // double gearPositionMax_ = std::numeric_limits<double>::max();
  // double gearVelocityMin_ = -std::numeric_limits<double>::max();
  // double gearVelocityMax_ = std::numeric_limits<double>::max();
  // double jointPositionMin_ = -std::numeric_limits<double>::max();
  // double jointPositionMax_ = std::numeric_limits<double>::max();
  // double jointVelocityMin_ = -std::numeric_limits<double>::max();
  // double jointVelocityMax_ = std::numeric_limits<double>::max();
  // double jointTorqueMin_ = -std::numeric_limits<double>::max();
  // double jointTorqueMax_ = std::numeric_limits<double>::max();
};


} // smb_common
