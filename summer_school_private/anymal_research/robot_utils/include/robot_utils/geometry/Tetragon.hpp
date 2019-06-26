/*
 * Tetragon.hpp
 *
 *  Created on: Jan 27, 2017
 *      Author: Dario Bellicoso
 */

#pragma once

// robot utils
#include "robot_utils/geometry/Polygon.hpp"
#include <robot_utils/geometry/common/geometry_utils.hpp>

namespace robot_utils {
namespace geometry {

class Tetragon : public Polygon {
 public:
  Tetragon(const VertexList& vertices,
           robot_utils::geometry::VertexOrder vertexOrder);
  ~Tetragon() override = default;

};

}
} /* namespace robot_utils */
