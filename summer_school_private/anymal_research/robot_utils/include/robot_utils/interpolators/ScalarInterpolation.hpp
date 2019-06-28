/*
 * ScalarInterpolation.hpp
 *
 *  Created on: Jan 28, 2019
 *      Author: Perry Franklin
 */

#pragma once

#include <string>

namespace robot_utils {

/**
 * Provides an interface for interpolation methods.
 * Children must define 'void interpolate(const double& t, double& s, double& sDot) const'
 */
class ScalarInterpolation {
 protected:
  /**
   * Constructor (only callable from derived classes)
   * @param name: the name of this interpolation
   */
  ScalarInterpolation(const std::string& name) : name_(name){};

 public:
  virtual ~ScalarInterpolation() = default;

  /**
   * Interpolates between 0 and 1, for 0<t<1. At t = 0, s = 0; at t 1, s =1.
   * For instance, if this is a linear interpolation, s = t, dsdt = 1.
   * If t < 0, or t > 1, this function should extrapolate.
   * @param t : input variable, between 0 and 1 for interpolation
   * @param s : output variable, between 0 and 1 for interpolation
   * @param dsdt : output variable, derivative of s
   */
  virtual void interpolate(double t, double* s, double* dsdt) const = 0;

  /**
   * Gets the name of this type of interpolation
   * @return name
   */
  const std::string& getName() const { return name_; }

 private:
  const std::string name_;
};

}  // namespace robot_utils
