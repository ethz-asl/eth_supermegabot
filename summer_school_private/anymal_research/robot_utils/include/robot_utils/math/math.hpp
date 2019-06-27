/*
 * math.hpp
 *
 *  Created on: Feb 23, 2014
 *      Author: gech
 */

#pragma once


// stl
#include <cmath>

// eigen
#include <Eigen/Core>

// kindr
#include <kindr/phys_quant/PhysicalQuantities.hpp>

// boost
#include <boost/math/special_functions/pow.hpp>


namespace robot_utils {


inline void boundToRange(double* v, double min, double max) {
  if (*v < min) *v = min;
  if (*v > max) *v = max;
}

inline void boundToRange(int* v, int min, int max) {
  if (*v < min) *v = min;
  if (*v > max) *v = max;
}


inline double boundToRange(double v, double min, double max) {
  if (v < min) return min;
  if (v > max) return max;
  return v;
}

/*
  if v < min, this method returns 0. If v > max, it returns 1. For everything else it returns some interpolated value;
*/
inline double mapTo01Range(double v, double min, double max) {
  double t = v;
  if (std::abs(min - max) < 0.000000001) return 1.0;
  boundToRange(&t, min, max);
  t = (t-min)/(max-min);
  return t;
}


inline double mapToUniformRange(double v, double min, double max){
  double t = v;
  if (std::abs(min - max) < 0.000000001) return 1.0;
  t = (t-min)/(max-min);
  return t;
}


inline double linearlyInterpolate(double v1, double v2, double t1, double t2, double t){
  if (v1 == v2)
    return v2;
  return (t-t1)/(t2-t1) * v2 + (t2-t)/(t2-t1) * v1;
}


inline Eigen::Vector3d linearlyInterpolate(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, double t1, double t2, double t) {
  return (t-t1)/(t2-t1) * v2 + (t2-t)/(t2-t1) * v1;
}


inline kindr::Position3D linearlyInterpolate(const kindr::Position3D& v1, const kindr::Position3D& v2, double t1, double t2, double t) {
  return (t-t1)/(t2-t1) * v2 + (t2-t)/(t2-t1) * v1;
}


template<typename T>
T mapInRange (T x, T in_min, T in_max, T out_min, T out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//! Map an integer n to the periodic interval (0, b-1).
inline int intmod(int n, int b) {
  assert(b>0);
  const int result = ( n>=0 ? n%b : ((n%b + b)%b) );
  assert(result>=0);
  assert(result<b);
  return result;
}

//! Map an integer n to the interval [a, b]
inline int intmodRange(int n, int a, int b) {
  assert(b>a);
  if (n>=a) {
    return (n-a)%(b-a);
  } else if (n>=0 && n<a) {
    return intmodRange(n + (b-a), a, b);
  } else {
    // todo: fixme !!
    // n<0
    return (n-a + b)%(b-a + b) + b - a;
  }
}

//! Check if two values are numerically close.
inline bool areNear(double a, double b, double tol = 1e-6) {
  return (std::fabs(a-b) <= tol);
}

//! True if a is equal or larger than the b.
inline bool isEqualOrLargerThan(double a, double b, double tol = 1e-6) {
  return ((a > b) || areNear(a, b, tol));
}

//! True if a is equal or smaller than the b.
inline bool isEqualOrSmallerThan(double a, double b, double tol = 1e-6) {
  return ((a < b) || areNear(a, b, tol));
}

//! True if a is larger than the b.
inline bool isLargerThan(double a, double b, double tol = 1e-6) {
  return ((a > b) && !areNear(a, b, tol));
}

//! True if a smaller than the b.
inline bool isSmallerThan(double a, double b, double tol = 1e-6) {
  return ((a < b) && !areNear(a, b, tol));
}


inline double computeNorm(double x, double y) {
  return std::sqrt(
        boost::math::pow<2>(x) +
        boost::math::pow<2>(y));
}

inline double computeNorm(double x, double y, double z) {
  return std::sqrt(
        boost::math::pow<2>(x) +
        boost::math::pow<2>(y) +
        boost::math::pow<2>(z));
}

//! map a phase event to the interval [0,1).
inline void mapToUnitInterval(double& phase, double tol=1e-6) {
  constexpr double phaseToMap = 0.0;
  // Check, if we are close to the bounds.
  if (areNear(phase, 1.0, tol)) {
    phase = phaseToMap;
    return;
  }

  if (areNear(phase, 0.0, tol)) {
    phase = phaseToMap;
    return;
  }

  // Start checking whether phase is greater than 1.0.
  while (phase > 1.0) {
    phase -= 1.0;
  }


  // Start checking whether phase is smaller than 0.0.
  while (phase < 0.0) {
    phase += 1.0;
  }


  // Check, if we are close to the bounds.
  if (areNear(phase, 1.0, tol)) {
    phase = phaseToMap;
    return;
  }

  if (areNear(phase, 0.0, tol)) {
    phase = phaseToMap;
    return;
  }
}

//! map a phase event to the interval [start,start+1.0].
inline void mapToInterval(double& phase, double interval_start, double tol=1e-6) {
  phase -= interval_start;
  mapToUnitInterval(phase, tol);
}

//! Assuming that phase1 happens before phase2, this function will return
// the phase difference in the interval [0,1]
inline double getPhaseDifference(double phase1, double phase2, double tol=1e-6) {
  double difference = phase2-phase1;
  mapToUnitInterval(difference, tol);
  return difference;
}


inline void alphaFilter(
    Eigen::Vector3d& vector,
    const Eigen::Vector3d& vectorRef,
    double weight, double weightRef) {
  if (weight>0.0 && weightRef>=0.0) {
    vector = weight*vector + weightRef*vectorRef;
  }
}

template<typename Scalar>
Scalar fastPow(const Scalar x, int y) {
  Scalar out = 1.0;
  while(y > 0) {
    out *= x;
    --y;
  }
  return out;
}

} // namespace robot_utils
