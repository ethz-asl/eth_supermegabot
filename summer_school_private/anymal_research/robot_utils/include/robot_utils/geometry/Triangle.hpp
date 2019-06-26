/*
 * Triangle.hpp
 *
 *  Created on: Apr 7, 2016
 *      Author: dbellicoso
 */

#pragma once

// robot utils
#include <robot_utils/geometry/Polygon.hpp>
#include <robot_utils/geometry/common/geometry_utils.hpp>

namespace robot_utils {
namespace geometry {

class Triangle : public Polygon
{
 public:
  Triangle(const VertexList& vertices,
           robot_utils::geometry::VertexOrder vertexOrder);
  ~Triangle() override = default;
};

} /* namespace robot_utils */
} /* namespace geometry */
