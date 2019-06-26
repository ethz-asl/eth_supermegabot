/*
 * BinaryChatteringCompensator.cpp
 *
 *  Created on: Jun 15, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "robot_utils/filters/BinaryChatteringCompensator.hpp"

#include <iostream>

namespace robot_utils {

BinaryChatteringCompensator::BinaryChatteringCompensator()
    : state_(false),
      active_(false),
      time_(0.0),
      timeBound_(0.0)
{

}

BinaryChatteringCompensator::~BinaryChatteringCompensator()
{

}

void BinaryChatteringCompensator::initialize(double timeBound)
{
  timeBound_ = timeBound;
  time_ = 0.0;
  active_ = false;
}

bool BinaryChatteringCompensator::advance(double dt, bool state)
{
  updateTimer(dt);

  if (state != state_ && !active_) {
    // Switch, but not active.
    resetTimerTo(state);
  }

  return state_;
}

void BinaryChatteringCompensator::resetTimerTo(bool state)
{
  state_ = state;
  time_ = 0.0;
  active_ = true;
}

void BinaryChatteringCompensator::updateTimer(double dt)
{
  if (!active_) return;

  time_ += dt;
  if (time_ > timeBound_) {
    active_ = false;
    time_ = 0.0;
  }
}

void BinaryChatteringCompensator::print() {
  std::cout << "BinaryChatteringCompensator: " << std::endl;
  std::cout << "   state: " << state_ << std::endl;
  std::cout << "    time: " << time_ << std::endl;
}

} /* namespace loco */
