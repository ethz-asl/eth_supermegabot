/*
 * CollisionRomoUtils.tpp
 *
 *  Created on: Aug 24, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include "collisions_urdf/urdf_to_collisions_geometry.hpp"

#include <urdf_parser/urdf_parser.h>
#include <iostream>
#include <fstream>

namespace romo{

template<typename ConcreteDescription_>
bool setCollisionsGeometryFromFile(const std::string& filename,
                                    romo::RigidBodyShPtrContainer <ConcreteDescription_>& rigid_body_ptr_container){

  std::ifstream model_file (filename);
    if (!model_file) {
      return false;
    }

    // reserve memory for the contents of the file
    std::string model_xml_string;
    model_file.seekg(0, std::ios::end);
    model_xml_string.reserve(model_file.tellg());
    model_file.seekg(0, std::ios::beg);
    model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());
    model_file.close();

    return setCollisionsGeometryFromString(model_xml_string.c_str(), rigid_body_ptr_container);
}

template<typename ConcreteDescription_>
bool setCollisionsGeometryFromString(const char* model_xml_string,
                                    romo::RigidBodyShPtrContainer <ConcreteDescription_>& rigid_body_ptr_container){

  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF (model_xml_string);

  return setCollisionsGeometryFromModel(urdf_model, rigid_body_ptr_container);

}

template<typename ConcreteDescription_>
bool findBodyFromUrdfLink(const urdf::Link& urdf_link,
                          typename ConcreteDescription_::ConcreteTopology::BodyEnum& body_enum_out,
                          Position& position_offset,
                          Orientation& orientation_offset){

  using RD = romo::RobotDescription<ConcreteDescription_>;

  bool found_name_in_body_keys = false;

    try {
      body_enum_out = RD::getBodyKeys().atName(urdf_link.name).getEnum();
      found_name_in_body_keys = true;
    }
    catch (const std::out_of_range& e) {
      found_name_in_body_keys = false;
    }

    if (found_name_in_body_keys){
      position_offset = Position();
      orientation_offset = Orientation();
      return true;
    } else {
      const auto& parent_link = urdf_link.getParent();
      if (parent_link){

        Position child_position;
        Orientation child_orientation;

        if (findBodyFromUrdfLink<ConcreteDescription_>(*parent_link,
                                                       body_enum_out,
                                                       child_position, child_orientation)){

          const urdf::Joint& parent_joint = *(urdf_link.parent_joint);

          const urdf::Vector3 joint_position = parent_joint.parent_to_joint_origin_transform.position;
          Position my_position(joint_position.x, joint_position.y, joint_position.z);

          const urdf::Rotation joint_rotation = parent_joint.parent_to_joint_origin_transform.rotation;
          kindr::RotationQuaternionD eigen_joint_rotation(joint_rotation.w,joint_rotation.x,joint_rotation.y,joint_rotation.z);
          Orientation my_orientation(eigen_joint_rotation);

          position_offset = my_position + my_orientation.rotate(child_position);
          orientation_offset = my_orientation*child_orientation;

          return true;
        } else {
          return false;
        }
      } else {
        return false;
      }
    }

}


template<typename ConcreteDescription_>
bool setCollisionsGeometryFromModel(const urdf::ModelInterfaceSharedPtr& model,
                                    romo::RigidBodyShPtrContainer <ConcreteDescription_>& rigid_body_ptr_container){

  bool success = true;

  using RD = romo::RobotDescription<ConcreteDescription_>;

  // We iterate through the links, try to find a corresponding link in our rigid_body_ptr_container and set the collision geometry
  for (auto link_name_ptr_pair : model->links_){

    typename RD::BodyEnum body_enum;

    Position link_position_offset;
    Orientation link_orientation_offset;

    if (!findBodyFromUrdfLink<ConcreteDescription_>(*(link_name_ptr_pair.second),
                                                    body_enum,
                                                    link_position_offset,
                                                    link_orientation_offset)){
      success = false;
      continue;
    }

    if (link_name_ptr_pair.second){
      collisions_urdf::setCollisionsGeometryFromLink(*(link_name_ptr_pair.second), rigid_body_ptr_container[body_enum]->collision_geometry,
                                    link_position_offset, link_orientation_offset);
    }
  }

  return success;

}

} //  namespace romo
