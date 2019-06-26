//  PIDController.hpp
//  Created by Philipp Leemann on Jan 2018

#pragma once

#include "parameter_handler/parameter_handler.hpp"

#include "robot_utils/controllers/PIDController.hpp"

namespace robot_utils{

class PIDControllerFeedForward : public PIDController
{
 public:

  PIDControllerFeedForward();
  PIDControllerFeedForward(const double maxEffort,
                           const double kp,
                           const double ki=0.0,
                           const double kd=0.0,
                           const double kf=0.0,
                           const double feedForwardGain=0.0);
  virtual ~PIDControllerFeedForward() = default;

  double Update(const double dt, const double desired, const double measured, const double feedForward);
  double Update(const double dt, const double desired, const double measured, const double desiredDerivative, const double feedForward);
  double Update(const double dt, const double desired, const double measured, const double desiredDerivative, const double measuredDerivative, const double feedForward);

  inline void setFeedForwardGain(const double gain) { feedForwardGain_.setValue(gain); }

  inline double getFeedForwardGain() const { return feedForwardGain_.getValue(); }

  bool addParametersToHandler(const std::string& pidName);

 protected:
  parameter_handler::Parameter<double> feedForwardGain_;
};

}
