/*!
 * @file    SeActuatorState.hpp
 * @author  Christian Gehring
 




   // DELETE








//  * @date    Mar 18, 2015
//  * @version 0.0
//  *
//  */
// #pragma once


// // c++
// #include <ostream>
// #include <stdint.h>

// // any measurements
// #include <any_measurements/Time.hpp>
// #include <any_measurements/Imu.hpp>

// namespace series_elastic_actuator {


// class SeActuatorState
// {
// public:
//   SeActuatorState();
//   virtual ~SeActuatorState();

//   const any_measurements::Time& getStamp() const;
//   void setStamp(const any_measurements::Time& stamp);

//   const uint32_t& getStatusword() const;
//   void setStatusword(uint32_t statusword);

//   const double& getCurrent() const;
//   void setCurrent(double current);

//   const double& getGearPosition() const;
//   void setGearPosition(double gearPosition);

//   const double& getGearVelocity() const;
//   void setGearVelocity(double gearVelocity);

//   const double& getJointPosition() const;
//   void setJointPosition(double jointPosition);

//   const double& getJointVelocity() const;
//   void setJointVelocity(double jointVelocity);

//   const double& getJointAcceleration() const;
//   void setJointAcceleration(double jointAcceleration);

//   const double& getJointTorque() const;
//   void setJointTorque(double jointTorque);

//   const any_measurements::Imu& getImu() const;
//   void setImu(const any_measurements::Imu& imu);

//   friend std::ostream& operator<<(std::ostream& out, const SeActuatorState& state);

// protected:
//   //! Stamp
//   any_measurements::Time stamp_;

//   //! Statusword
//   uint32_t statusword_ = 0;
//   //! Motor current [A]
//   double current_ = 0.0;
//   //! Gear position [rad]
//   double gearPosition_ = 0.0;
//   //! Gear velocity [rad/s]
//   double gearVelocity_ = 0.0;
//   //! Joint position [rad]
//   double jointPosition_ = 0.0;
//   //! Joint velocity [rad/s]
//   double jointVelocity_ = 0.0;
//   //! Joint velocity [rad/s²]
//   double jointAcceleration_ = 0.0;
//   //! Joint torque [Nm]
//   double jointTorque_ = 0.0;
//   //! Imu measurement
//   any_measurements::Imu imu_;
// };

// } // series_elastic_actuator
