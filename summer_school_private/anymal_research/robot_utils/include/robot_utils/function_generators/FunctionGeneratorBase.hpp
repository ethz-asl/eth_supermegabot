/*
 * FunctionGeneratorBase.hpp
 *
 *  Created on: Nov 12, 2014
 *      Author: Dario Bellicoso
 */

#ifndef FUNCTIONGENERATORBASE_HPP_
#define FUNCTIONGENERATORBASE_HPP_


namespace robot_utils {

class FunctionGeneratorBase {
 public:
  FunctionGeneratorBase();
  virtual ~FunctionGeneratorBase();

  /*! Up and Down Sweep Function
   * @param time time [s]
   * @return  value
   */
  virtual double getUpAndDownSweepValue(double time) = 0;

  /*! Up Sweep Function
   * @param time time [s]
   * @return  value
   */
  virtual double getUpSweepValue(double time) = 0;

  virtual void setParamAmplitude(double paramAmplitude);
  virtual void setParamMinFrequencyHz(double minFreq);
  virtual void setParamMaxFrequencyHz(double maxFreq);
  virtual void setParamTimeInteval(double timeInterval);
  virtual void setCurrentFrequencyHz(double currentFreq);
  virtual void setComputedTimeInteval(double computedTimeInterval);

  virtual const double getParamAmplitude() const;
  virtual const double getParamMinFrequencyHz() const;
  virtual const double getParamMaxFrequencyHz() const;
  virtual const double getParamTimeInteval() const;
  virtual const double getCurrentFrequencyHz() const;
  virtual const double getComputedTimeInteval() const;

  virtual double& getCurrentFrequencyHz();
 protected:
   //! amplitude of sine
   double paramAmplitude_;

   //! minimum frequency [Hz]
   double paramMinFrequencyHz_;

   //! maximum frequency [Hz]
   double paramMaxFrequencyHz_;

   //! time interval of the sweep (estimated time interval for up and down sweep)
   double paramTimeInteval_;

   //! current frequency for logging
   double currentFrequencyHz_;

   //! exact time interval for up and down sweep
   double computedTimeInteval_;

};

} /* namespace robot_utils */

#endif /* FUNCTIONGENERATORBASE_HPP_ */
