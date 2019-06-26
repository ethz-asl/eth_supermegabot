/*
 * Ellipsoid.hpp
 *
 *  Created on: Mar 21, 2015
 *      Author: C. Dario Bellicoso
 */

#pragma once

// robot utils
#include "robot_utils/surfaces/Surface.hpp"


namespace robot_utils {


class Ellipsoid : public Surface
{
 public:
  explicit Ellipsoid(const Surface::Vector& dimensions);
  Ellipsoid(const double axisA, const double axisB, const double axisC);
  virtual ~Ellipsoid();

  void setSemiPrincipalAxisA(double semiAxisA);
  void setSemiPrincipalAxisB(double semiAxisB);
  void setSemiPrincipalAxisC(double semiAxisC);

  /*
   * Given a vector v = [x y z]^T, constrain it to be bounded by the ellipsoid surface. The vector v will represent a point
   * external to the ellipsoid if and only if:
   *
   *        (v-v0)^T * A * (v-v0) > 1
   *
   * Considering the unit norm direction of v:
   *
   *          v
   * vn = --------
   *        ||v||
   *
   *
   * the intersection x of v and the ellipsoid can be found by solving the system of equations:
   *
   *        (x)^T * A * (x) = 1
   *
   *        x = t*vn
   *
   * where t is an unknown scalar value. Solving for t yields:
   *
   *        t = sqrt( inv(vn^T*A*vn) )
   *
   * which implies:
   *
   *    x = sqrt( inv(vn^T*A*vn) ) * vn
   *
   */
  bool constrainVectorToVolume(Surface::Vector& vector);

  bool isPointExternal(const Surface::Vector& vector);

 private:
  double ellipsoidEquation(const Surface::Vector& vector);

  Eigen::Matrix3d ellipsCore_;
  Surface::Vector ellipseCenter_;
};

} /* namespace robot_utils */
