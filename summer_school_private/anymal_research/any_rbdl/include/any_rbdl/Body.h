/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2015 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_BODY_H
#define RBDL_BODY_H

#include "any_rbdl/rbdl_math.h"
#include "any_rbdl/rbdl_mathutils.h"
#include <assert.h>
#include <iostream>
#include "any_rbdl/Logging.h"

namespace RigidBodyDynamics {

struct FixedBody;

/** \brief Describes all properties of a single body
 *
 * A Body contains information about mass, the location of its center of
 * mass, and the ineria tensor in the center of mass. This class is
 * designed to use the given information and transform it such that it can
 * directly be used by the spatial algebra.
 */
struct ANY_RBDL_DLLAPI Body {
	Body() :
		mIsVirtual (false),
		combinedMass_ (0.),
		combinedCenterOfMass_ (0., 0., 0.),
		combinedInertia_ (Math::Matrix3d::Zero(3,3)),
		individualMass_(0.),
		individualCenterOfMass_(0., 0., 0.),
		individualInertia_ (Math::Matrix3d::Zero(3,3))
		{
			UpdateInertialProperties();
		};

	Body(const Body &body) = default;
	Body& operator= (const Body &body) = default;

	/** \brief Constructs a body from mass, center of mass and radii of gyration
	 *
	 * This constructor eases the construction of a new body as all the
	 * required parameters can be specified as parameters to the
	 * constructor. These are then used to generate the spatial inertia
	 * matrix which is expressed at the origin.
	 *
	 * \param mass the mass of the body
	 * \param com  the position of the center of mass in the bodies coordinates
	 * \param gyration_radii the radii of gyration at the center of mass of the body
	 */
	Body(const double &mass,
			const Math::Vector3d &com,
			const Math::Vector3d &gyration_radii) :
		mIsVirtual (false),
		combinedMass_ (mass),
		combinedCenterOfMass_(com),
		individualMass_(mass),
		individualCenterOfMass_(com){
			combinedInertia_ = Math::Matrix3d (
					gyration_radii[0], 0., 0.,
					0., gyration_radii[1], 0.,
					0., 0., gyration_radii[2]
					);
			individualInertia_ = combinedInertia_;
			UpdateInertialProperties();
		}

	/** \brief Constructs a body from mass, center of mass, and a 3x3 inertia matrix
	 *
	 * This constructor eases the construction of a new body as all the
	 * required parameters can simply be specified as parameters to the
	 * constructor. These are then used to generate the spatial inertia
	 * matrix which is expressed at the origin.
	 *
	 * \param mass the mass of the body
	 * \param com  the position of the center of mass in the bodies coordinates
	 * \param inertia_C the inertia at the center of mass
	 */
	Body(const double &mass,
			const Math::Vector3d &com,
			const Math::Matrix3d &inertia_C) :
		mIsVirtual (false),
		combinedMass_ (mass),
		combinedCenterOfMass_(com),
		combinedInertia_ (inertia_C),
		individualMass_(mass),
		individualCenterOfMass_(com),
		individualInertia_ (inertia_C){
			UpdateInertialProperties();
		}

	~Body() {};

	/** \brief Joins inertial parameters of two bodies to create a composite
	 * body.
	 *
	 * This function can be used to joint inertial parameters of two bodies
	 * to create a composite body that has the inertial properties as if the
	 * two bodies were joined by a fixed joint.
	 *
	 * \note Both bodies have to have their inertial parameters expressed in
	 * the same orientation.
	 *
	 * \param transform The frame transformation from the origin of the
	 * original body to the origin of the added body
	 * \param other_body The other body that will be merged with *this.
	 */
	/*
	 * This function is now deprecated, as instead a Body should only store information about itself, and not
	 * combine its mass values with its children.
	 * Instead, if you wish to add a FixedBody use the function addChildFixedBody to add a pointer to a child
	 * (most likely a ptr to the mFixedBodies member of Model), then call createSpatialBodyInertia, which returns
	 * a combined SpatialBodyInertia with all children added.
	 */
	void Join (const Math::SpatialTransform &transform, const Body &other_body) __attribute__ ((deprecated)) {
		// nothing to do if we join a massles body to the current.
		if (other_body.GetMass() == 0. && other_body.GetInertia() == Math::Matrix3d::Zero()) {
			return;
		}

		double other_mass = other_body.GetIndividualMass();
		double new_mass = GetIndividualMass() + other_mass;

		if (new_mass == 0.) {
			std::cerr << "Error: cannot join bodies as both have zero mass!" << std::endl;
			assert (false);
			abort();
		}

		Math::Vector3d other_com = transform.E.transpose() * other_body.GetIndividualCenterOfMass() + transform.r;
		Math::Vector3d new_com = (1 / new_mass ) * (GetIndividualMass() * GetIndividualCenterOfMass() + other_mass * other_com);

		LOG << "other_com = " << std::endl << other_com.transpose() << std::endl;
		LOG << "rotation = " << std::endl << transform.E << std::endl;

		// We have to transform the inertia of other_body to the new COM. This
		// is done in 4 steps:
		//
		// 1. Transform the inertia from other origin to other COM
		// 2. Rotate the inertia that it is aligned to the frame of this body
		// 3. Transform inertia of other_body to the origin of the frame of
		// this body
		// 4. Sum the two inertias
		// 5. Transform the summed inertia to the new COM

		Math::SpatialRigidBodyInertia other_rbi = Math::SpatialRigidBodyInertia::createFromMassComInertiaC (other_body.GetIndividualMass(), other_body.GetIndividualCenterOfMass(), other_body.GetIndividualInertia());
		Math::SpatialRigidBodyInertia this_rbi = Math::SpatialRigidBodyInertia::createFromMassComInertiaC (GetIndividualMass(), GetIndividualCenterOfMass(), GetIndividualInertia());

		Math::Matrix3d inertia_other = other_rbi.toMatrix().block<3,3>(0,0);
		LOG << "inertia_other = " << std::endl << inertia_other << std::endl;

		// 1. Transform the inertia from other origin to other COM
		Math::Matrix3d other_com_cross = Math::VectorCrossMatrix(other_body.GetIndividualCenterOfMass());
		Math::Matrix3d inertia_other_com = inertia_other - other_mass * other_com_cross * other_com_cross.transpose();
		LOG << "inertia_other_com = " << std::endl << inertia_other_com << std::endl;

		// 2. Rotate the inertia that it is aligned to the frame of this body
		Math::Matrix3d inertia_other_com_rotated = transform.E.transpose() * inertia_other_com * transform.E;
		LOG << "inertia_other_com_rotated = " << std::endl << inertia_other_com_rotated << std::endl;

		// 3. Transform inertia of other_body to the origin of the frame of this body
		Math::Matrix3d inertia_other_com_rotated_this_origin = Math::parallel_axis (inertia_other_com_rotated, other_mass, other_com);
		LOG << "inertia_other_com_rotated_this_origin = " << std::endl << inertia_other_com_rotated_this_origin << std::endl;

		// 4. Sum the two inertias
		Math::Matrix3d inertia_summed = Math::Matrix3d (this_rbi.toMatrix().block<3,3>(0,0)) + inertia_other_com_rotated_this_origin;
		LOG << "inertia_summed  = " << std::endl << inertia_summed << std::endl;

		// 5. Transform the summed inertia to the new COM
		Math::Matrix3d new_inertia = inertia_summed - new_mass * Math::VectorCrossMatrix (new_com) * Math::VectorCrossMatrix(new_com).transpose();

		LOG << "new_mass = " << new_mass << std::endl;
		LOG << "new_com  = " << new_com.transpose() << std::endl;
		LOG << "new_inertia  = " << std::endl << new_inertia << std::endl;

		individualMass_ = new_mass;
		individualCenterOfMass_ = new_com;
		individualInertia_ = new_inertia;

		std::copy(other_body.fixedBodyChildren_.begin(), other_body.fixedBodyChildren_.end(), this->fixedBodyChildren_.end());

		UpdateInertialProperties();
	}

	/// \brief Creates a SpatialRigidBodyInertia that combines this body with all of it's fixed body children, and updates the combined properties accordingly
	void  UpdateInertialProperties();

	/// \brief Adds a FixedBody as a child of this Body
	void AddChildFixedBody(FixedBody* fixedBody);

	/// \brief Gets the mass of this body (including its fixed children)
	const double& GetMass() const { return combinedMass_; }

	/// \brief Gets the center of mass of this body (including its fixed children)
	const Math::Vector3d& GetCenterOfMass() const { return combinedCenterOfMass_; }

	/// \brief Gets the inertia of this body (including its fixed children)
	const Math::Matrix3d& GetInertia() const { return combinedInertia_; }

	/// \brief Sets the individual inertial properties for this body (without its fixed children), and optionally updates the combined inertial properties
	void SetIndividualInertialProperties(double mass, const Math::Vector3d& centerOfMass, const Math::Matrix3d& inertia, bool updateAllInertiaProperties = true);

	/// \brief Sets the mass of this body without its fixed children, and optionally updates the combined inertial properties
	void SetIndividualMass(double mass, bool updateAllInertiaProperties = true);
	/// \brief Gets the mass of this body without its fixed children
	const double& GetIndividualMass() const { return individualMass_; }

	/// \brief Sets the center of mass of this body without its fixed children, and optionally updates the combined inertial properties
	void SetIndividualCenterOfMass(const Math::Vector3d& centerOfMass, bool updateAllInertiaProperties = true);
	/// \brief Gets the center of mass of this body without its fixed children
	const Math::Vector3d& GetIndividualCenterOfMass() const { return individualCenterOfMass_; }

	/// \brief Sets the inertia of this body without its fixed children, and optionally updates the combined inertial properties
	void SetIndividualInertia(const Math::Matrix3d& inertia, bool updateAllInertiaProperties = true);
	/// \brief Gets the inertia of this body without its fixed children
	const Math::Matrix3d& GetIndividualInertia() const { return individualInertia_; }

	/// \brief Gets the SpatialRigidBodyInertia of this body (including its fixed children)
	const Math::SpatialRigidBodyInertia& GetSpatialRigidBodyInertia() const { return spatialRigidBodyInertia_; }

	/// \brief Absorbs the mass properties of the fixed children of this body (ie sets the individual properties of this body as the combined inertial properties, and nulls the fixed children)
	void AbsorbFixedChildren(bool updateAllInertiaProperties = true);

	/// \brief Nulls the mass properties of the fixed children of this body
	void NullFixedChildren(bool updateAllInertiaProperties = true);

	/// \brief Gets if the children of this body were nulled
	bool GetChildrenWereAbsorbed() const { return childrenWereAbsorbed_; }

	bool mIsVirtual;

private:

	/// \brief The spatial rigid body inertia for this moveable body (including its fixed children)
	Math::SpatialRigidBodyInertia spatialRigidBodyInertia_;

	/// \brief The mass of this moveable body (including its fixed children)
	double combinedMass_;
	/// \brief The position of the center of mass of this moveable body (including its fixed children) in body coordinates
	Math::Vector3d combinedCenterOfMass_;
	/// \brief Inertia matrix of this moveable body (including its fixed children) at the center of mass
	Math::Matrix3d combinedInertia_;

	/// \brief The mass of this body (without its fixed children)
	double individualMass_;
	/// \brief The position of the center of mass of this body (without its fixed children) in body coordinates
	Math::Vector3d individualCenterOfMass_;
	/// \brief Inertia matrix of this body (without its fixed children) at the center of mass
	Math::Matrix3d individualInertia_;

	/// \brief Pointers to all the fixed body children of this link
	std::vector<FixedBody*> fixedBodyChildren_;

	/// \brief Flag indicating whether the children of this body were nulled (ie NullFixedChildren was called)
	bool childrenWereAbsorbed_{false};
};

/** \brief Keeps the information of a body and how it is attached to another body.
 *
 * When using fixed bodies, i.e. a body that is attached to anothe via a
 * fixed joint, the attached body is merged onto its parent. By doing so
 * adding fixed joints do not have an impact on runtime.
 */
struct ANY_RBDL_DLLAPI FixedBody {

	/// \brief Id of the movable body that this fixed body is attached to.
	unsigned int mMovableParent;
	/// \brief Ptr to the movable body that this fixed body is attached to.
	Body* mMovableParentPtr;
	/// \brief Transforms spatial quantities expressed for the parent to the
	// fixed body.
	Math::SpatialTransform mParentTransform;
	Math::SpatialTransform mBaseTransform;

	static FixedBody CreateFromBody (const Body& body) {
		FixedBody fbody;

		fbody.mMass = body.GetMass();
		fbody.mCenterOfMass = body.GetCenterOfMass();
		fbody.mInertia = body.GetInertia();

		return fbody;
	}

	/// \brief Sets the individual inertial properties for this fixed body, and optionally updates the inertia properties of its moveable parent
	void SetIndividualInertialProperties(double mass, const Math::Vector3d& centerOfMass, const Math::Matrix3d& inertia, bool updateAllInertiaProperties = true);

	/// \brief Sets the individual mass for this fixed body, and optionally updates the inertia properties of its moveable parent
	void SetIndividualMass(double mass, bool updateAllInertiaProperties = true);
	/// \brief Gets the individual mass for this fixed body
	double GetIndividualMass() const{ return mMass;	}

	/// \brief Sets the individual center of mass for this fixed body, and optionally updates the inertia properties of its moveable parent
	void SetIndividualCenterOfMass(const Math::Vector3d& centerOfMass, bool updateAllInertiaProperties = true);
	/// \brief Gets the individual mass for this fixed body
	const Math::Vector3d& GetIndividualCenterOfMass() const { return mCenterOfMass; }

	/// \brief Sets the individual inertia for this fixed body, and optionally updates the inertia properties of its moveable parent
	void SetIndividualInertia(const Math::Matrix3d& inertia, bool updateAllInertiaProperties = true);
	/// \brief Gets the individual inertia for this fixed body
	const Math::Matrix3d& GetIndividualInertia() const { return mInertia; }

	/// \brief Updates the inertial properties of this fixed body's moveable parent
	void UpdateInertialProperties();

	/// \brief Sets all of the inertial properties of this fixed body to zero
	void NullInertialProperties();

	/// \brief Get whether the inertial properties of this body were nulled (they may still be zero if it wasn't)
	bool GetIsNulled() const { return isNulled_; }

private:
	/// \brief The mass of the fixed body
	double mMass;
	/// \brief The position of the center of mass in body coordinates
	Math::Vector3d mCenterOfMass;
	/// \brief The spatial inertia that contains both mass and inertia information
	Math::Matrix3d mInertia;

	/// \brief Flag indicating if the inertial properties were nulled (ie NullInertialProperties was called)
	///        False does not mean that the properties are not zero.
	bool isNulled_{false};

};

}

/* RBDL_BODY_H */
#endif
