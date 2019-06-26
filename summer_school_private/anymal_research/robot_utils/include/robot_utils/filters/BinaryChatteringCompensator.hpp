/*
 * BinaryChatteringCompensator.hpp
 *
 *  Created on: Jun 15, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#pragma once

namespace robot_utils {

class BinaryChatteringCompensator {
 public:

  BinaryChatteringCompensator();
  virtual ~BinaryChatteringCompensator();

  void initialize(double timeBound);
  bool advance(double dt, bool state);
  void print();

 private:
  void resetTimerTo(bool state);
  void updateTimer(double dt);
  bool state_;
  bool active_;
  double time_;
  double timeBound_;

};

} /* namespace robot_utils */


