/*
 * GaussianKernelJumpPropagator.cpp
 *
 *  Created on: Jul 13, 2014
 *      Author: wko
 */

#include "robot_utils/function_approximators/dynamicMovementPrimitive/GaussianKernelJumpPropagator.hpp"

GaussianKernelJumpPropagator::GaussianKernelJumpPropagator()
    : currentTime_(0.0) {
}

GaussianKernelJumpPropagator::~GaussianKernelJumpPropagator() {

}

bool GaussianKernelJumpPropagator::initialize(double dt) {
  timeStep_ = dt;
  return true;
}

/**
 * Ensures that a number is clamped between a lower bound and an upper bound.
 * @param number: Number to be clamped.
 * @param lower: Lower bound.
 * @param upper: Upper bound.
 */
inline double clamp(double number, double lower, double upper) {
  if (number > upper) {
    number = upper;
  } else if (number < lower) {
    number = lower;
  }
  return number;
}

/**
 * Returns max duration of jump.
 */
double GaussianKernelJumpPropagator::getMaxDuration() {
  return maxDuration_;
}

/**
 * Resets the timer.
 */
void GaussianKernelJumpPropagator::resetTime() {
  currentTime_ = 0.0;
}

/**
 * Increments the timer with timestep dt.
 */
void GaussianKernelJumpPropagator::incrementTime() {
  currentTime_ += timeStep_;
}

/**
 * Returns value of trajectory at current point of time.
 */
Eigen::VectorXd GaussianKernelJumpPropagator::predict() {

  Eigen::VectorXd result;
  result.resize(desiredTrajectories_.size());

  for (int i = 0; i < result.size(); i++) {
    double value;
    desiredTrajectories_.at(i).predict(getProgress(), value, false);
    result(i) = value;
  }
  incrementTime();

  return result;
}

/**
 * Keeps track of how much of the trajectory has already been followed in terms of progress (between 0 and 1).
 */
double GaussianKernelJumpPropagator::getProgress() {
  return clamp(currentTime_ / maxDuration_, 0.0, 1.0);
}

/*
 * Checks whether we are in velocitymode or not.
 */
bool GaussianKernelJumpPropagator::inVelocityMode() {
  return velocityMode_;
}

/**
 * Loads parameters for the trajectory to follow.
 */
bool GaussianKernelJumpPropagator::loadTrajectory(const TiXmlHandle &hJump) {
  TiXmlHandle hTrajectory(hJump.FirstChild("Trajectory"));

  if (!loadMovement(hTrajectory)) {
    return false;
  }

  TiXmlElement* child = hTrajectory.FirstChild("GaussianKernel").ToElement();
  for (; child; child = child->NextSiblingElement()) {
    dmp::GaussianKernel desTrajectory;
    if (!loadGaussianKernel(child, &desTrajectory)) {
      return false;
    }
    desiredTrajectories_.push_back(desTrajectory);
  }

  return true;
}

/**
 * Loads parameters for the GaussianKernel to shape the trajectory.
 * @param hTrajectory: Parent XML tag of the GaussianKernel tag.
 */
bool GaussianKernelJumpPropagator::loadGaussianKernel(
    TiXmlElement* pElem, dmp::GaussianKernel* desiredTrajectory) {
  int numBasisFunctions;
  double activation, canSysCutOff;
  bool exponentiallySpaced;

  if (!pElem) {

    printf("Could not find Jump:Trajectory:GaussianKernel\n");
    return false;
  }

  TiXmlHandle hGaussianKernel(pElem);
  TiXmlElement* child = hGaussianKernel.FirstChild().ToElement();

  TiXmlHandle hThetas(child);

  if (!loadThetas(hThetas)) {
    return false;
  }

  numBasisFunctions = thetas_.size();

  if (pElem->QueryDoubleAttribute("activation", &activation) != TIXML_SUCCESS) {
    printf("Could not find activation parameter!\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("canSysCutOff", &canSysCutOff)
      != TIXML_SUCCESS) {
    printf("Could not find parameter canSysCutOff!\n");
    return false;
  }

  if (pElem->QueryBoolAttribute("exponentiallySpaced", &exponentiallySpaced)
      != TIXML_SUCCESS) {
    printf("Could not find parameter exponentiallySpaced!\n");
    return false;
  }

  desiredTrajectory->initialize(numBasisFunctions, activation,
                                exponentiallySpaced, canSysCutOff);

  /* Print out thetas to check correctness */
//  for (int i = 0; i < thetas_->size(); i++) {
//    std::cout << "theta[" << i << "] = " << (*thetas_)(i) << std::endl;
//  }
  desiredTrajectory->setThetas(thetas_);

  return true;
}

/**
 * Loads parameter for the duration of the jump.
 * @param hTrajectory: Parent XML tag of the Movement tag.
 */
bool GaussianKernelJumpPropagator::loadMovement(
    const TiXmlHandle &hTrajectory) {
  double maxDuration;
  bool velocityMode;

  TiXmlElement *pElem;
  pElem = hTrajectory.FirstChild("Movement").Element();
  if (!pElem) {
    printf("Could not find Jump:Trajectory:Movement\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("maxDuration", &maxDuration)
      != TIXML_SUCCESS) {
    printf("Could not find parameter maxDuration!\n");
    return false;
  }

  if (pElem->QueryBoolAttribute("velocityMode", &velocityMode)
      != TIXML_SUCCESS) {
    printf("Could not find parameter velocityMode!\n");
    return false;
  }

  velocityMode_ = velocityMode;
  maxDuration_ = maxDuration;
  return true;
}

/**
 * Loads thetas for GaussianKernel.
 * @param hTrajectory: Parent XML tag of the Movement tag.
 */
bool GaussianKernelJumpPropagator::loadThetas(const TiXmlHandle &hThetas) {
  double value;
  std::vector<double> values;

  TiXmlElement* child = hThetas.FirstChild().ToElement();
  for (; child; child = child->NextSiblingElement()) {
    if (child->QueryDoubleAttribute("v", &value) != TIXML_SUCCESS) {
      printf("Could not find value of theta!\n");
      return false;
    }
    values.push_back(value);
  }

  thetas_.resize(values.size());

  for (unsigned int i = 0; i < values.size(); i++) {
    thetas_[i] = values.at(i);
  }

  return true;
}
