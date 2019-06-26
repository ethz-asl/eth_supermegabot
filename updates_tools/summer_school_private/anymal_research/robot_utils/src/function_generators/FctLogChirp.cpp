/*!
* @file 	FctLogChirp.cpp
* @author 	Christian Gehring
* @date 	Jun 7, 2012
* @version 	1.0
* @ingroup 	robot_utils
* @brief
*/

#include "robot_utils/function_generators/FctLogChirp.hpp"
#include <cmath>

namespace robot_utils {


FctLogChirp::FctLogChirp():
    FunctionGeneratorBase(),
    paramOffset_(0.0)
{

}


FctLogChirp::~FctLogChirp() {

}


double FctLogChirp::getUpSweepValue(double time) {

  computedTimeInteval_ = paramTimeInteval_;
	currentFrequencyHz_ = paramMinFrequencyHz_*pow((paramMaxFrequencyHz_/paramMinFrequencyHz_), time/paramTimeInteval_);
	const double L = paramTimeInteval_ / log(paramMaxFrequencyHz_/paramMinFrequencyHz_);
	const double K = paramTimeInteval_*(2.0*M_PI*paramMinFrequencyHz_) / log(paramMaxFrequencyHz_/paramMinFrequencyHz_);
	return paramAmplitude_*sin(K*(exp(time/L)-1)) + paramOffset_;
}

double FctLogChirp::getUpSweepFirstDerivativeValue(double time) {

  computedTimeInteval_ = paramTimeInteval_;
  currentFrequencyHz_ = paramMinFrequencyHz_*pow((paramMaxFrequencyHz_/paramMinFrequencyHz_), time/paramTimeInteval_);
  const double L = paramTimeInteval_ / log(paramMaxFrequencyHz_/paramMinFrequencyHz_);
  const double K = paramTimeInteval_*(2.0*M_PI*paramMinFrequencyHz_) / log(paramMaxFrequencyHz_/paramMinFrequencyHz_);
  return paramAmplitude_*K/L*exp(time/L)*cos(K*(exp(time/L)-1));
}


double FctLogChirp::getUpAndDownSweepValue(double time) {

	double t;
	double A = paramAmplitude_;
	const double f0 = paramMinFrequencyHz_;
	double k  = (paramMaxFrequencyHz_ - paramMinFrequencyHz_)/(paramTimeInteval_/2.0);

	const double t_near_middle = paramTimeInteval_/2.0; //-(1.0/paramMaxFrequencyHz_)/2.0;
	const double k_middle = ceil(((f0 + k*t_near_middle)*(f0 + k*t_near_middle) - f0*f0)/k);
	const double t_middle = -(f0 - sqrt(f0*f0 + k*k_middle))/k;

	computedTimeInteval_ = 2.0*t_middle;
	//k  = (paramMaxFrequencyHz_ - paramMinFrequencyHz_)/(t_middle);

	if (time <= t_middle) {
		t = time;
		A = paramAmplitude_;
	} else {
		t = 2.0*t_middle - time;
		A = -paramAmplitude_;
	}


	currentFrequencyHz_ = (f0 + k*t);
	return A*sin(2.0*M_PI*(f0 + k/2.0*t)*t) + paramOffset_;
}


double FctLogChirp::getParamOffset() {
  return paramOffset_;
}
void FctLogChirp::setParamOffset(double offset) {
  paramOffset_ = offset;
}

} /* namespace robot_utils */
