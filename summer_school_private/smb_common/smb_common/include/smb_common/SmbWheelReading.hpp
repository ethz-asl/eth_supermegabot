/*!
 * @file    SmbWheelReading.hpp
 * @author  Koen Kraemer
 * @date    Aug 2, 2018
 *
 */
#pragma once

// c++
#include <ostream>
#include <stdint.h>

// any measurements
#include <any_measurements/Time.hpp>

namespace smb_common {


class SmbWheelReading
{
 public:
  SmbWheelReading();
  virtual ~SmbWheelReading();

  const any_measurements::Time& getStamp() const;
  void setStamp(const any_measurements::Time& stamp);

  const double& getCurrent() const;
  void setCurrent(double current);

  const double& getWheelVelocity() const;
  void setWheelVelocity(double wheelVelocity);

  const double& getWheelTorque() const;
  void setWheelTorque(double wheelTorque);

  friend std::ostream& operator<<(std::ostream& out, const SmbWheelReading& reading);

 protected:
   //! Stamp
   any_measurements::Time stamp_;

   //! Motor current [A]
   double current_ = 0.0;
   //! Wheel velocity [rad/s]
   double wheelVelocity_ = 0.0;
   //! Wheel torque [Nm]
   double wheelTorque_ = 0.0;
};

} // smb_common
