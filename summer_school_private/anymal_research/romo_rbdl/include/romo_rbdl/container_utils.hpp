/*
 * container_utils.hpp
 *
 *  Created on: Apr 29, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

#include <romo_rbdl/containers.hpp>
#include <romo_rbdl/RobotModelRbdl.hpp>
#include <romo_rbdl/RigidBodyRbdl.hpp>
#include <romo/common/phys_typedefs.hpp>

namespace romo_rbdl {

template<typename ConcreteDescription_>
static void fillBodyContainer(romo::RigidBodyShPtrContainer<ConcreteDescription_>& rigidBodyContainer,
                              std::shared_ptr<RigidBodyDynamics::Model> model)
{
  for (auto& key : romo::RobotDescription<ConcreteDescription_>::getBodyKeys()) {
    rigidBodyContainer.at(key.getEnum()) = std::make_shared<RigidBodyRbdl<ConcreteDescription_>>(model, key.getEnum());
  }
}

template<typename ConcreteDescription_, typename RobotState_>
static void fillContactContainer(romo::ContactShPtrContainer<ConcreteDescription_>& container,
                                      romo_rbdl::RobotModelRbdl<ConcreteDescription_, RobotState_>* model)
{
  using RD = romo::RobotDescription<ConcreteDescription_>;
  using BodyEnum = typename RD::BodyEnum;
  using ContactEnum = typename RD::ContactEnum;

  for (auto& key : RD::getContactKeys()) {
    romo_std::ContactImpl<ConcreteDescription_, RobotState_>  item(model, key.getEnum());

    const unsigned int contactId = (model->getRbdlModel().GetBodyId(key.getName()));
    const auto& parentBodyName = RD::template mapKeyEnumToKeyName<ContactEnum, BodyEnum>(key.getEnum());
    const unsigned int parentBodyId = model->getRbdlModel().GetBodyId(parentBodyName);
    auto& rbdlModel = model->getRbdlModel();

    RigidBodyDynamics::Math::SpatialTransform transformFromReferenceToParent;
    if (parentBodyId != std::numeric_limits<unsigned int>::max()) {
      if (rbdlModel.IsFixedBodyId(parentBodyId)) {
        transformFromReferenceToParent =
            model->getRbdlModel().mFixedBodies[parentBodyId - model->getRbdlModel().fixed_body_discriminator]->mParentTransform;
      } else {
        transformFromReferenceToParent.E.setIdentity();
        transformFromReferenceToParent.r.setZero();
      }
    } else {
      MELO_WARN_STREAM("[romo_rbdl::fillContactContainer]: Did not find parent body with name " << parentBodyName << "! Skipping contact.");
      continue;
    }

    RigidBodyDynamics::Math::SpatialTransform transformFromReferenceToContact;
    if (contactId != std::numeric_limits<unsigned int>::max()) {
      if (model->getRbdlModel().IsFixedBodyId(contactId)) {
        transformFromReferenceToContact = model->getRbdlModel().mFixedBodies[contactId
            - model->getRbdlModel().fixed_body_discriminator]->mParentTransform;
      } else {
        transformFromReferenceToContact.E.setIdentity();
        transformFromReferenceToContact.r.setZero();
      }
    } else {
      MELO_WARN_STREAM("[romo_rbdl::fillContactContainer]: Did not find body belonging to " << key.getName() << "! Skipping contact.");
      continue;
    }

    // Set the contact transformation.
    RigidBodyDynamics::Math::SpatialTransform spatialTransformParentToContact =
        transformFromReferenceToContact*transformFromReferenceToParent.inverse();
    item.setPositionMovableParentToContactInMovableParentFrame( romo::Position(spatialTransformParentToContact.r) );
    item.setOrientationMovableParentToContact( romo::RotationQuaternion( romo::RotationMatrix(spatialTransformParentToContact.E) ) );

    // Add the item to the container.
    container.at(key.getEnum()) = std::make_shared<romo_std::ContactImpl<ConcreteDescription_, RobotState_>>(item);
  }
}

} /* namespace romo_std */
