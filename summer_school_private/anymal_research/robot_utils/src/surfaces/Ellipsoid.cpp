/*
 * Ellipsoid.cpp
 *
 *  Created on: Mar 21, 2015
 *      Author: C. Dario Bellicoso
 */

#include "robot_utils/surfaces/Ellipsoid.hpp"


namespace robot_utils {

Ellipsoid::Ellipsoid(const Surface::Vector& dimensions)
{
  ellipsCore_.setZero();
  ellipseCenter_.setZero();

  setSemiPrincipalAxisA(static_cast<double>(dimensions.x()));
  setSemiPrincipalAxisB(static_cast<double>(dimensions.y()));
  setSemiPrincipalAxisC(static_cast<double>(dimensions.z()));
}

Ellipsoid::Ellipsoid(const double axisA, const double axisB, const double axisC) {
  ellipsCore_.setZero();
  ellipseCenter_.setZero();

  setSemiPrincipalAxisA(axisA);
  setSemiPrincipalAxisB(axisB);
  setSemiPrincipalAxisC(axisC);
}

Ellipsoid::~Ellipsoid()
{

}

bool Ellipsoid::constrainVectorToVolume(Surface::Vector& vector) {
  if ( isPointExternal(vector) ) {
    Surface::Vector vn = vector.normalized();
    vector = sqrt( 1.0 / ellipsoidEquation(vn) ) * vn;
  } else {
  }

  return true;
}

bool Ellipsoid::isPointExternal(const Surface::Vector& point) {
  return (ellipsoidEquation(point) > 1.0);
}

double Ellipsoid::ellipsoidEquation(const Surface::Vector& vector) {
  return (vector-ellipseCenter_).toImplementation().transpose() *ellipsCore_*(vector-ellipseCenter_).toImplementation();
}

void Ellipsoid::setSemiPrincipalAxisA(double semiAxisA) {
  if (semiAxisA) {
    ellipsCore_(0,0) = 1.0/(semiAxisA*semiAxisA);
  } else {
    ellipsCore_(0,0) = 0.0;
  }
}
void Ellipsoid::setSemiPrincipalAxisB(double semiAxisB) {
  if (semiAxisB) {
    ellipsCore_(1,1) = 1.0/(semiAxisB*semiAxisB);
  } else {
    ellipsCore_(1,1) = 0.0;
  }
}
void Ellipsoid::setSemiPrincipalAxisC(double semiAxisC) {
  if (semiAxisC) {
    ellipsCore_(2,2) = 1.0/(semiAxisC*semiAxisC);
  } else {
    ellipsCore_(2,2) = 0.0;
  }
}

} /* namespace robot_utils */
