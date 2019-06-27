/*
 * geometry_utils.hpp
 *
 *  Created on: Apr 7, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// eigen
#include <Eigen/Core>

// kindr
#include <kindr/Core>

namespace robot_utils {
namespace geometry {

enum class VertexOrder : unsigned int {
  ClockWise = 0,
  CounterClockWise,
  Undefined
};

using Position2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;

/*
 * Given two vertices v1 and vw2 which lie on a line in R^2 modeled by:
 *
 *        ax + by + c = 0
 *
 *  we wish to find the vector of coefficients [a b c].
 *  The 2d vector [a b] defines the normal to the line. Depending on the vertex order (clockwise or counter-clockwise),
 *  one of the two normal directions is computed.
 */
bool getLineCoefficientsFromVertices(const Position2d& v1,
                                     const Position2d& v2,
                                     std::vector<double>& coefficients,
                                     const VertexOrder order = VertexOrder::CounterClockWise);


double computeInRadiusOfTriangle(const Position2d& vA,
                                 const Position2d& vB,
                                 const Position2d& vC);

/*
 * Given two points p1 and p2 in R^2, compare the phase of the two w.r.t. to a center point.
 * Return true if the phase of p1 is less than that of p2.
 */
inline bool less(const Position2d& p1, const Position2d& p2, const Position2d& center) {
  return   std::atan2(static_cast<double>(p1.y()-center.y()), static_cast<double>(p1.x()-center.x()))
         < std::atan2(static_cast<double>(p2.y()-center.y()), static_cast<double>(p2.x()-center.x()));
}

inline double det(const Position2d& v1, const Position2d& v2) {
  return (v1.x()*v2.y() - v1.y()*v2.x());
}

} /* namespace geometry */
} /* namespace robot_utils */
