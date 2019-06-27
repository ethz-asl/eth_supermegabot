/*
 * Body.cc
 *
 *  Created on: 6 May 2019
 *      Author: Perry Franklin
 */

#include "any_rbdl/Body.h"
#include "any_rbdl/Model.h"

namespace RigidBodyDynamics {

void Body::UpdateInertialProperties(){
	if (fixedBodyChildren_.size() == 0){
		combinedMass_ = individualMass_;
		combinedCenterOfMass_ = individualCenterOfMass_;
		combinedInertia_ = individualInertia_;
		spatialRigidBodyInertia_ = Math::SpatialRigidBodyInertia::createFromMassComInertiaC(combinedMass_, combinedCenterOfMass_, combinedInertia_);
	} else {
		double totalMass(individualMass_);
		Math::Vector3d weightedCenterOfMassSum = individualMass_*individualCenterOfMass_;
		Math::Matrix3d inertiaSummedAtBodyFrame = Math::parallel_axis(individualInertia_, individualMass_, individualCenterOfMass_);
		for (const FixedBody* fixedBody : fixedBodyChildren_){
			// If a fixed body has zero mass, skip it
			if (fixedBody->GetIndividualMass() == 0. && fixedBody->GetIndividualInertia().isZero()) {
				continue;
			}
			totalMass += fixedBody->GetIndividualMass();

			Math::Vector3d fixedBodyComInThisBodyFrame = fixedBody->mParentTransform.E.transpose() * fixedBody->GetIndividualCenterOfMass() + fixedBody->mParentTransform.r;

			weightedCenterOfMassSum += fixedBody->GetIndividualMass() * fixedBodyComInThisBodyFrame;

			Math::Matrix3d fixedBodyInertiaAtFixedBodyCoM = fixedBody->GetIndividualInertia();

			// Rotate the inertia that it is aligned to the frame of this body
			Math::Matrix3d fixedBodyInertiaAtFixedBodyComInThisBodyFrame = fixedBody->mParentTransform.E.transpose() * fixedBodyInertiaAtFixedBodyCoM * fixedBody->mParentTransform.E;

			// Transform inertia of other_body to the origin of the frame of this body
			Math::Matrix3d fixedBodyInertiaAtThisBodyFrameInThisBodyFrame = Math::parallel_axis (fixedBodyInertiaAtFixedBodyComInThisBodyFrame, fixedBody->GetIndividualMass(), fixedBodyComInThisBodyFrame);

			// Sum the two inertias now that they are in the same frame (this body frame)
			inertiaSummedAtBodyFrame += fixedBodyInertiaAtThisBodyFrameInThisBodyFrame;
		}
		Math::Vector3d summedCenterOfMass;
		if (totalMass != 0){
			summedCenterOfMass = weightedCenterOfMassSum/totalMass;
		} else {
			summedCenterOfMass = individualCenterOfMass_;
		}
		Math::Matrix3d summedCenterOfMassCrossMatrix = Math::VectorCrossMatrix (summedCenterOfMass);
		Math::Matrix3d inertiaSummedAtNewCoM = inertiaSummedAtBodyFrame - totalMass * summedCenterOfMassCrossMatrix * summedCenterOfMassCrossMatrix.transpose();

		combinedMass_ = totalMass;
		combinedCenterOfMass_ = summedCenterOfMass;
		combinedInertia_ = inertiaSummedAtNewCoM;
		spatialRigidBodyInertia_ = Math::SpatialRigidBodyInertia::createFromMassComInertiaC(totalMass, summedCenterOfMass, inertiaSummedAtNewCoM);
	}
}

void Body::AddChildFixedBody(FixedBody* fixedBody){
	fixedBodyChildren_.emplace_back(fixedBody);
	fixedBody->mMovableParentPtr = this;
	UpdateInertialProperties();
}

void Body::SetIndividualMass(double mass, bool updateAllInertiaProperties){
	individualMass_ = mass;
	if (updateAllInertiaProperties){
		UpdateInertialProperties();
	}
}

void Body::SetIndividualCenterOfMass(const Math::Vector3d& centerOfMass, bool updateAllInertiaProperties){
	individualCenterOfMass_ = centerOfMass;
	if (updateAllInertiaProperties){
		UpdateInertialProperties();
	}
}

void Body::SetIndividualInertia(const Math::Matrix3d& inertia, bool updateAllInertiaProperties){
	individualInertia_ = inertia;
	if (updateAllInertiaProperties){
		UpdateInertialProperties();
	}
}

void Body::SetIndividualInertialProperties(double mass, const Math::Vector3d& centerOfMass, const Math::Matrix3d& inertia, bool updateAllInertiaProperties){
	individualMass_ = mass;
	individualCenterOfMass_ = centerOfMass;
	individualInertia_ = inertia;
	if (updateAllInertiaProperties){
		UpdateInertialProperties();
	}
}

void Body::AbsorbFixedChildren( bool updateAllInertiaProperties){
	individualMass_ = combinedMass_;
	individualCenterOfMass_ = combinedCenterOfMass_;
	individualInertia_ = combinedInertia_;
	NullFixedChildren(false);
	if (updateAllInertiaProperties){
		UpdateInertialProperties();
	}
	childrenWereAbsorbed_ = true;
}

void Body::NullFixedChildren(bool updateAllInertiaProperties){
	for (FixedBody* fixedBody : fixedBodyChildren_){
		fixedBody->SetIndividualInertialProperties(0.0, Math::Vector3d::Zero(), Math::Matrix3d::Zero(), false);
	}
	if (updateAllInertiaProperties){
		UpdateInertialProperties();
	}
}

void FixedBody::SetIndividualInertialProperties(double mass, const Math::Vector3d& centerOfMass, const Math::Matrix3d& inertia, bool updateAllInertiaProperties){
	mMass = mass;
	mCenterOfMass = centerOfMass;
	mInertia = inertia;
	if (updateAllInertiaProperties){
		UpdateInertialProperties();
	}
}

void FixedBody::SetIndividualMass(double mass, bool updateAllInertiaProperties){
	mMass = mass;
	if (updateAllInertiaProperties){
		UpdateInertialProperties();
	}
}

void FixedBody::SetIndividualCenterOfMass(const Math::Vector3d& centerOfMass, bool updateAllInertiaProperties){
	mCenterOfMass = centerOfMass;
	if (updateAllInertiaProperties){
		UpdateInertialProperties();
	}
}

void FixedBody::SetIndividualInertia(const Math::Matrix3d& inertia, bool updateAllInertiaProperties){
	mInertia = inertia;
	if (updateAllInertiaProperties){
		UpdateInertialProperties();
	}
}

void FixedBody::UpdateInertialProperties(){
	mMovableParentPtr->UpdateInertialProperties();
}

}  // namespace RigidBodyDynamics
