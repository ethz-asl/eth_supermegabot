/*
 * FunctionGeneratorBase.cpp
 *
 *  Created on: Nov 12, 2014
 *      Author: Dario Bellicoso
 */

#include "robot_utils/function_generators/FunctionGeneratorBase.hpp"


namespace robot_utils {


FunctionGeneratorBase::FunctionGeneratorBase()
    : paramAmplitude_(0.0),
      paramMinFrequencyHz_(0.0),
      paramMaxFrequencyHz_(0.0),
      paramTimeInteval_(0.0),
      currentFrequencyHz_(0.0),
      computedTimeInteval_(0.0)
{

}

FunctionGeneratorBase::~FunctionGeneratorBase() {

}


void FunctionGeneratorBase::setParamAmplitude(double paramAmplitude) {
  paramAmplitude_ = paramAmplitude;
}


void FunctionGeneratorBase::setParamMinFrequencyHz(double minFreq) {
  paramMinFrequencyHz_ = minFreq;
}


void FunctionGeneratorBase::setParamMaxFrequencyHz(double maxFreq) {
  paramMaxFrequencyHz_ = maxFreq;
}


void FunctionGeneratorBase::setParamTimeInteval(double timeInterval) {
  paramTimeInteval_ = timeInterval;
}


void FunctionGeneratorBase::setCurrentFrequencyHz(double currentFreq) {
  currentFrequencyHz_ = currentFreq;
}


void FunctionGeneratorBase::setComputedTimeInteval(double computedTimeInterval) {
  computedTimeInteval_ = computedTimeInterval;
}


const double FunctionGeneratorBase::getParamAmplitude() const {
  return paramAmplitude_;
}


const double FunctionGeneratorBase::getParamMinFrequencyHz() const {
  return paramMinFrequencyHz_;
}


const double FunctionGeneratorBase::getParamMaxFrequencyHz() const {
  return paramMaxFrequencyHz_;
}


const double FunctionGeneratorBase::getParamTimeInteval() const {
  return paramTimeInteval_;
}


const double FunctionGeneratorBase::getCurrentFrequencyHz() const {
  return currentFrequencyHz_;
}


const double FunctionGeneratorBase::getComputedTimeInteval() const {
  return computedTimeInteval_;
}

double& FunctionGeneratorBase::getCurrentFrequencyHz() {
  return currentFrequencyHz_;
}

} /* namespace robot_utils */
