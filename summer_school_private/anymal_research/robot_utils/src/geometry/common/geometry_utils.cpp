/*
 * geometry_utils.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: Dario Bellicoso
 */

// robot utils
#include <robot_utils/geometry/common/geometry_utils.hpp>

namespace robot_utils {
namespace geometry {


bool getLineCoefficientsFromVertices(const Position2d& v1,
                                     const Position2d& v2,
                                     std::vector<double>& coefficients,
                                     const VertexOrder order) {

  assert(coefficients.size() == 3);

  double l_a = 0.0;
  double l_b = 0.0;
  double l_c = 0.0;

  switch (order) {
    case(VertexOrder::CounterClockWise): {
      l_a = v1[1] - v2[1];
      l_b = v2[0] - v1[0];
    } break;
    case(VertexOrder::ClockWise): {
      l_a = - (v1[1] - v2[1]);
      l_b = - (v2[0] - v1[0]);
    } break;
    default: throw std::runtime_error("[getLineCoefficientsFromVertices] unhandled vertex order case!"); break;
  }

  const double param_norm = std::sqrt(l_a*l_a + l_b*l_b);
  if (param_norm == 0.0) {
    coefficients[0] = 0.0;
    coefficients[1] = 0.0;
    coefficients[2] = 0.0;
    return false;
  } else {
    l_a /= param_norm;
    l_b /= param_norm;
    l_c = -l_a*v1[0]-l_b*v1[1];
    coefficients[0] = l_a;
    coefficients[1] = l_b;
    coefficients[2] = l_c;
  }

  return true;
}


double computeInRadiusOfTriangle(const Position2d& vA,
                                 const Position2d& vB,
                                 const Position2d& vC) {
  // Radius of incircle = Area / (0.5* perimeter) = det(v1, v2) / (perimeter)
  const Position2d vAB = vB-vA;
  const Position2d vAC = vC-vA;
  const Position2d vBC = vC-vB;
  return std::fabs(det(vAB, vAC)) / (vAB.norm() + vAC.norm() + vBC.norm());
}


}
}
