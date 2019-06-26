/*
 * GaussianBaseTrajectory.hpp
 *
 *  Created on: Oct 17, 2014
 *      Author: huub
 */

#ifndef GAUSSIANBASETRAJECTORY_HPP_
#define GAUSSIANBASETRAJECTORY_HPP_

#include <vector>

namespace robot_utils {

class GaussianBaseTrajectory {

 public:
  typedef std::vector<double> Parameters;
  typedef std::vector<double> BaseValues;

 public:
  GaussianBaseTrajectory();
  ~GaussianBaseTrajectory();

  void initialize(const Parameters& par, double timeHorizon, double dt);
  double advance();
  double next() const;
  double previous() const;
  double getCurrentSecondDerivative() const;
  double getCurrentDerivative() const;
  double getCurrentValue() const;
  double getValueAtTimeFromNow(double time, double dt) const;
  void reset();

  void setParameters(const Parameters& par);
  void setSigmaOfBasisFunctions(double sigma);
  void setNumberOfBasisFunctions(double numberOfBasisFunctions);
  void setDurationOfTrajectory(double timeHorizon);
  void setTimeStep(double dt);

  double getLengthOfTrajectory() const;
  double getSigma() const;
  int getNumberOfBasisFunctions() const;
  Parameters getParameters() const;

 protected:
  double sigma_;
  double timeBetweenFirstAndLastParametrizedBaseFunction_;
  int numberOfBasisfunctions_;
  double basisScaling_;
  double timeStep_;
  Parameters parameters_;
  double currentTime_;

  double previousValue_;
  double currentValue_;
  double currentDerivativeValue_;
  double currentSecondDerivativeValue_;

  std::vector<double> values_;
  std::vector<double> derivativeValues_;
  std::vector<double> secondDerivativeValues_;

  void setScalingOfBasisFunctions();

  void generateValueVectors();
  double getValues(double& value, double& derivativeValue, double& secondDerivativeValue, double t) const;
  void generateBasisFunctionVectors(BaseValues& baseValues, BaseValues& baseDerivatives, BaseValues& baseSecondDerivatives, double time, double sigma, double timeBetweenFirstAndLastParametrizedBaseFunction, int numberOfBasisFunctions) const;
  double getValueOfGaussian(double x, double mu, double sigma) const;
  double calculateValuesOfTrajectory(const BaseValues& basisVector,const Parameters& parameters) const;

  Parameters elementWiseVectorMultiplication(const BaseValues& vecB, const Parameters& vecP) const;
  double sum(const Parameters& vec) const;
};
}



#endif /* GaussianBaseTrajectory_HPP_ */
