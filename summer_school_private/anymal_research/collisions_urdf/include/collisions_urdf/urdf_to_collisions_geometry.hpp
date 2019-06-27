/*
 * urdf_to_collisions_geometry.hpp
 *
 *  Created on: Aug 24, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include <collisions_geometry/CollisionGeometry.hpp>

#include <urdf_model/model.h>
#include <urdf_model/link.h>

using namespace collisions_geometry;

namespace collisions_urdf{

using Position = typename CollisionGeometry::Position;
using Orientation = typename CollisionGeometry::Rotation;

bool setCollisionsGeometryFromLink(const urdf::Link&, std::shared_ptr<CollisionGeometry>& geom_ptr, 
                                  const Position& position_offset,
                                  const Orientation& orientation_offset);

} //  namespace collisions_urdf
