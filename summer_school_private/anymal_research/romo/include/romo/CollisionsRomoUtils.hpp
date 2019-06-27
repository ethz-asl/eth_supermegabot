/*
 * CollisionRomoUtils.hpp
 *
 *  Created on: Aug 24, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include <collisions_geometry/CollisionGeometry.hpp>

#include <romo/common/containers.hpp>

#include <urdf_model/model.h>
#include <urdf_model/link.h>

using namespace collisions_geometry;

namespace romo{

using Position = typename CollisionGeometry::Position;
using Orientation = typename CollisionGeometry::Rotation;

template<typename ConcreteDescription_>
bool setCollisionsGeometryFromFile(const std::string& filename,
                                    romo::RigidBodyShPtrContainer <ConcreteDescription_>& rigid_body_ptr_container);

template<typename ConcreteDescription_>
bool setCollisionsGeometryFromString(const char* model_xml_string,
                                    romo::RigidBodyShPtrContainer <ConcreteDescription_>& rigid_body_ptr_container);


template<typename ConcreteDescription_>
bool setCollisionsGeometryFromModel(const urdf::ModelInterfaceSharedPtr& model,
                                    romo::RigidBodyShPtrContainer <ConcreteDescription_>& rigid_body_ptr_container);

} //  namespace romo

#include <romo/CollisionsRomoUtils.tpp>
