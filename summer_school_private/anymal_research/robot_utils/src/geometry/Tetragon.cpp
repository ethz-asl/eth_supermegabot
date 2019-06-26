/*
 * Tetragon.cpp
 *
 *  Created on: Jan 27, 2017
 *      Author: dbellicoso
 */

#include "robot_utils/geometry/Tetragon.hpp"

namespace robot_utils {
namespace geometry {

Tetragon::Tetragon(const VertexList& vertices,
                   robot_utils::geometry::VertexOrder vertexOrder)
  : Polygon(vertices, vertexOrder)
{
  assert(vertices_.size() == 4);
  lineCoefficients_ = Polygon::LineCoefficientList(4, {0.0, 0.0, 0.0});
}

} /* namespace geometry */
} /* namespace robot_utils */
