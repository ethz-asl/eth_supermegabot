/*
 * GaussianBaseTrajectory.cpp
 *
 *  Created on: Oct 17, 2014
 *      Author: huub
 */

#include "robot_utils/function_approximators/gaussianBaseTrajectory/GaussianBaseTrajectory.hpp"

#include <numeric>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace robot_utils {

GaussianBaseTrajectory::GaussianBaseTrajectory() :
      sigma_(0.1),
      timeBetweenFirstAndLastParametrizedBaseFunction_(1.0),
      numberOfBasisfunctions_(5),
      basisScaling_(1.0),
      previousValue_(0.0),
      currentValue_(0.0)
{

}

GaussianBaseTrajectory::~GaussianBaseTrajectory() {

}

void GaussianBaseTrajectory::initialize(const Parameters& par, double timeHorizon, double dt) {
  currentTime_ = 0.0;

  setParameters(par);
  setDurationOfTrajectory(timeHorizon);
  setNumberOfBasisFunctions(par.size());
  setTimeStep(dt);
  setSigmaOfBasisFunctions(timeBetweenFirstAndLastParametrizedBaseFunction_/((double) 2*(numberOfBasisfunctions_ - 1)));
  setScalingOfBasisFunctions();

  generateValueVectors();
}

void GaussianBaseTrajectory::reset() {
  currentTime_ = 0.0;
}

void GaussianBaseTrajectory::generateValueVectors()
{
  double value;
  double derivativeValue;
  double secondDerivativeValue;

  values_.clear();
//  int numberOfTimeSteps = (int) (timeBetweenFirstAndLastParametrizedBaseFunction_ + 3*sigma_)/timeStep_;
  for (double t = 0; t < (timeBetweenFirstAndLastParametrizedBaseFunction_ + 3*sigma_); t += timeStep_) {
    getValues(value, derivativeValue, secondDerivativeValue, t);
    values_.push_back(value);
    derivativeValues_.push_back(derivativeValue);
    secondDerivativeValues_.push_back(secondDerivativeValue);
  }
}

double GaussianBaseTrajectory::advance() {
  double value = 0.0;
  currentTime_ += timeStep_;
  previousValue_ = currentValue_;

  if (!values_.empty()) {
    value = *values_.begin();
    values_.erase(values_.begin());
    currentDerivativeValue_ = *derivativeValues_.begin();
    derivativeValues_.erase(derivativeValues_.begin());
    currentSecondDerivativeValue_ = *secondDerivativeValues_.begin();
    secondDerivativeValues_.erase(secondDerivativeValues_.begin());
  }
  else {
    currentSecondDerivativeValue_ = 0.0;
    currentDerivativeValue_ = 0.0;
    value = 0.0;
  }

  currentValue_ = value;
  return value;
}

double GaussianBaseTrajectory::next() const{
  double nextValue = 0.0;
  if (!values_.empty()) {
    nextValue = *values_.begin();
  }
  else {
    nextValue = 0.0;
  }
  return nextValue;
}

double GaussianBaseTrajectory::previous() const {
  return previousValue_;
}

double GaussianBaseTrajectory::getCurrentSecondDerivative() const {
  return currentSecondDerivativeValue_;
}

double GaussianBaseTrajectory::getCurrentDerivative() const {
  return currentDerivativeValue_;
}

double GaussianBaseTrajectory::getCurrentValue() const {
  return currentValue_;
}

double GaussianBaseTrajectory::getValueAtTimeFromNow(double time, double dt) const {
  return values_[(int) time/dt];
}

void GaussianBaseTrajectory::setSigmaOfBasisFunctions(double sigma)
{
  sigma_ = sigma;
}

void GaussianBaseTrajectory::setNumberOfBasisFunctions(double numberOfBasisFunctions)
{
  numberOfBasisfunctions_ = numberOfBasisFunctions;
}
void GaussianBaseTrajectory::setDurationOfTrajectory(double timeHorizon)
{
  timeBetweenFirstAndLastParametrizedBaseFunction_ = timeHorizon;
}
void GaussianBaseTrajectory::setParameters(const Parameters& par)
{
  parameters_.clear();
  parameters_.push_back(par[0] + (par[0]-par[2]));
  parameters_.push_back(par[0] + (par[0]-par[1]));
  for (unsigned int i = 0; i < par.size(); i++) {
    parameters_.push_back(par[i]);
  }
//  parameters_.push_back(par[par.size()-1]);
}

double GaussianBaseTrajectory::getValues(double& value, double& derivativeValue, double& secondDerivativeValue, double t) const {
    BaseValues baseFunctionVector;
    BaseValues baseFunctionDerivativeVector;
    BaseValues baseFunctionSecondDerivativeVector;
  generateBasisFunctionVectors(baseFunctionVector, baseFunctionDerivativeVector, baseFunctionSecondDerivativeVector,
      t, sigma_, timeBetweenFirstAndLastParametrizedBaseFunction_, numberOfBasisfunctions_);
//  std::cout << baseFunctionVector[0] << std::endl;
  value = calculateValuesOfTrajectory(baseFunctionVector, parameters_);
  derivativeValue = calculateValuesOfTrajectory(baseFunctionDerivativeVector, parameters_);
  secondDerivativeValue = calculateValuesOfTrajectory(baseFunctionSecondDerivativeVector, parameters_);
  return value;
}


double GaussianBaseTrajectory::getLengthOfTrajectory() const {
  return timeBetweenFirstAndLastParametrizedBaseFunction_;
}

double GaussianBaseTrajectory::getSigma() const {
  return sigma_;
}

int GaussianBaseTrajectory::getNumberOfBasisFunctions() const {
  return numberOfBasisfunctions_;
}

GaussianBaseTrajectory::Parameters GaussianBaseTrajectory::getParameters() const {
  return parameters_;
}

double GaussianBaseTrajectory::calculateValuesOfTrajectory(const BaseValues& basisVector,const Parameters& parameters) const {
  Parameters valueVector;
  double output;
  valueVector = elementWiseVectorMultiplication(basisVector,parameters);
  output = sum(valueVector)/basisScaling_;

  return output;
}

void GaussianBaseTrajectory::generateBasisFunctionVectors(BaseValues& baseValues, BaseValues& baseDerivatives, BaseValues& baseSecondDerivatives,
                                                         double time, double sigma, double timeBetweenFirstAndLastParametrizedBaseFunction,
                                                         int numberOfBasisFunctions) const {
//  BaseValues output;
  double mu;
  double muSpacing = timeBetweenFirstAndLastParametrizedBaseFunction/((double) (numberOfBasisFunctions-1));
  for (int i = 0; i < (numberOfBasisFunctions+2); i++) {
    mu = ((double) i)*muSpacing - 2*muSpacing;
    baseValues.push_back(getValueOfGaussian(time, mu, sigma));
    baseDerivatives.push_back(-(time - mu)/(sigma*sigma) * getValueOfGaussian(time, mu, sigma));
    baseSecondDerivatives.push_back(-(1/(sigma*sigma)) * getValueOfGaussian(time, mu, sigma) + pow((time - mu)/(sigma*sigma), 2) * getValueOfGaussian(time, mu, sigma));
  }
//  return output;
}

void GaussianBaseTrajectory::setScalingOfBasisFunctions() {
  if (numberOfBasisfunctions_ != 1) {
    BaseValues baseFunctionVector;
    BaseValues dummyVector;
    BaseValues dummyVector2;
    generateBasisFunctionVectors(baseFunctionVector, dummyVector, dummyVector2,
                                timeBetweenFirstAndLastParametrizedBaseFunction_/2 - timeBetweenFirstAndLastParametrizedBaseFunction_/((double) 4*(numberOfBasisfunctions_ - 1)),
                                                        sigma_,timeBetweenFirstAndLastParametrizedBaseFunction_,numberOfBasisfunctions_);
    basisScaling_ = sum(baseFunctionVector);
  }
  else {
    basisScaling_ = 1.0;
  }
}

double GaussianBaseTrajectory::getValueOfGaussian(double x, double mu, double sigma) const {
  double value = (1/(sqrt(2*M_PI)*sigma))*exp(- pow(x - mu ,2) / (2*pow(sigma,2)) );
  return value;
}

void GaussianBaseTrajectory::setTimeStep(double dt) {
  timeStep_ = dt;
}

GaussianBaseTrajectory::Parameters GaussianBaseTrajectory::elementWiseVectorMultiplication(const BaseValues& vecB, const Parameters& vecP) const {
  Parameters output;
  if (vecB.size() != vecP.size()) {
    throw std::runtime_error("ERROR: Vectors to be multiplied are not the same length");
  }

  for (unsigned int i = 0; i < vecB.size(); i++) {
    output.push_back(vecB[i]*vecP[i]);
  }

  return output;
}

double GaussianBaseTrajectory::sum(const Parameters& vec) const {
  double output = 0;
  for (unsigned int i = 0; i < vec.size(); i++) {
    output += vec[i];
  }
  return output;
}

}
