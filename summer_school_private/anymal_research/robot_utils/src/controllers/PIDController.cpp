//  PIDController.cpp
//  Created by Philipp Leemann on 08.03.15.

#include <algorithm>

#include "robot_utils/controllers/PIDController.hpp"

namespace robot_utils {

PIDController::PIDController():
	        PIDController(10.0, 1.0, 0.0, 0.0, 0.0)
{
}

PIDController::PIDController(const double maxEffort,
                             const double kp,
                             const double ki,
                             const double kd,
                             const double kf,
                             const double maxIntegratorInput)
: previousMeasured_(0.0),
  integral_(0.0),
  maxEffort_("Pid/MaxEffort", maxEffort, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max()),
  kp_("Pid/Kp", kp, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max()),
  ki_("Pid/Ki", ki, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max()),
  kd_("Pid/Kd", kd, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max()),
  kf_("Pid/Kf", kf, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max()),
  maxIntegratorInput_("Pid/MaxIntegratorInput", maxIntegratorInput, 0, std::numeric_limits<double>::max())
{
}

void PIDController::Reset()
{
  previousMeasured_ = 0.0;
  integral_ = 0.0;
}

double PIDController::Update(const double dt, const double desired, const double measured)
{
  const double measuredDerivative = (measured - previousMeasured_)/dt;
  return Update(dt, desired, measured, 0.0, measuredDerivative);
}

double PIDController::Update(const double dt, const double desired, const double measured, const double desiredDerivative)
{
  const double measuredDerivative = (measured - previousMeasured_)/dt;
  return Update(dt, desired, measured, desiredDerivative, measuredDerivative);
}

double PIDController::Update(const double dt, const double desired, const double measured, const double desiredDerivative, const double measuredDerivative)
{
  double out = 0.0;
  const double error = desired - measured;
  const double derivative = desiredDerivative - measuredDerivative;
  if (std::abs(error) < maxIntegratorInput_.getValue()) {
    const double newIntegral = integral_ + error * dt;
    if ((newIntegral < maxEffort_.getValue()) && (newIntegral > -maxEffort_.getValue())) {
      integral_ = newIntegral;
    }
  }
  out = kp_.getValue()*error + ki_.getValue()*integral_ + kd_.getValue()*derivative + kf_.getValue()*desired;
  out = std::max(std::min(out, maxEffort_.getValue()), -maxEffort_.getValue());
  previousMeasured_ = measured;
  return out;
}

bool PIDController::addParametersToHandler(const std::string& pidName)
{
  parameter_handler::handler->addParam(pidName + std::string("/MaxEffort"), maxEffort_);
  parameter_handler::handler->addParam(pidName + std::string("/Kp"), kp_);
  parameter_handler::handler->addParam(pidName + std::string("/Ki"), ki_);
  parameter_handler::handler->addParam(pidName + std::string("/Kd"), kd_);
  parameter_handler::handler->addParam(pidName + std::string("/Kf"), kf_);
  parameter_handler::handler->addParam(pidName + std::string("/MaxIntegratorInput"), maxIntegratorInput_);
  return true;
}

}
