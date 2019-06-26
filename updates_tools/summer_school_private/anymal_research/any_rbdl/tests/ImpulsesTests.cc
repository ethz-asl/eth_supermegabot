#include <gtest/gtest.h>
#include <kindr/common/gtest_eigen.hpp>

#include <iostream>

#include "any_rbdl/Logging.h"

#include "any_rbdl/Model.h"
#include "any_rbdl/Contacts.h"
#include "any_rbdl/Dynamics.h"
#include "any_rbdl/Kinematics.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-14;

struct ImpulsesFixture : ::testing::Test{
	ImpulsesFixture () {
		ClearLogOutput();
		model = new Model;

		model->gravity = Vector3d (0., -9.81, 0.);

		// base body
		base = Body (
				1.,
				Vector3d (0., 1., 0.),
				Vector3d (1., 1., 1.)
				);
		joint_rotzyx = Joint (
				SpatialVector (0., 0., 1., 0., 0., 0.),
				SpatialVector (0., 1., 0., 0., 0., 0.),
				SpatialVector (1., 0., 0., 0., 0., 0.)
				);
		base_id = model->AddBody (0, Xtrans (Vector3d (0., 0., 0.)), joint_rotzyx, base);

		// child body (3 DoF)
		child = Body (
				1.,
				Vector3d (0., 1., 0.),
				Vector3d (1., 1., 1.)
				);
		child_id = model->AddBody (base_id, Xtrans (Vector3d (1., 0., 0.)), joint_rotzyx, child);

		Q = VectorNd::Zero(model->dof_count);
		QDot = VectorNd::Zero(model->dof_count);
		QDDot = VectorNd::Zero(model->dof_count);
		Tau = VectorNd::Zero(model->dof_count);

		contact_body_id = child_id;
		contact_point = Vector3d (0., 1., 0.);
		contact_normal = Vector3d (0., 1., 0.);

		ClearLogOutput();
	}

	~ImpulsesFixture () {
		delete model;
	}
	Model *model;

	unsigned int base_id, child_id;
	Body base, child;
	Joint joint_rotzyx;

	VectorNd Q;
	VectorNd QDot;
	VectorNd QDDot;
	VectorNd Tau;

	unsigned int contact_body_id;
	Vector3d contact_point;
	Vector3d contact_normal;
	ConstraintSet constraint_set;
};

TEST_F(ImpulsesFixture, TestContactImpulse) {
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (1., 0., 0.), NULL, 0.);
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (0., 1., 0.), NULL, 0.);
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (0., 0., 1.), NULL, 0.);

	constraint_set.Bind (*model);

	constraint_set.v_plus[0] = 0.;
	constraint_set.v_plus[1] = 0.;
	constraint_set.v_plus[2] = 0.;

	QDot[0] = 0.1;
	QDot[1] = -0.2;
	QDot[2] = 0.1;

	Vector3d point_velocity;
	{
		SUPPRESS_LOGGING;
		point_velocity = CalcPointVelocity (*model, Q, QDot, contact_body_id, contact_point, true);
	}

	// cout << "Point Velocity = " << point_velocity << endl;

	VectorNd qdot_post (QDot.size());
	ComputeContactImpulsesDirect (*model, Q, QDot, constraint_set, qdot_post);
	// cout << LogOutput.str() << endl;
	// cout << "QdotPost = " << qdot_post << endl;

	{
		SUPPRESS_LOGGING;
		point_velocity = CalcPointVelocity (*model, Q, qdot_post, contact_body_id, contact_point, true);
	}

	// cout << "Point Velocity = " << point_velocity << endl;
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (Vector3d (0., 0., 0.), point_velocity, TEST_PREC, TEST_PREC, "");
}

TEST_F(ImpulsesFixture, TestContactImpulseRotated) {
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (1., 0., 0.), NULL, 0.);
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (0., 1., 0.), NULL, 0.);
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (0., 0., 1.), NULL, 0.);

	constraint_set.Bind (*model);

	constraint_set.v_plus[0] = 0.;
	constraint_set.v_plus[1] = 0.;
	constraint_set.v_plus[2] = 0.;

	Q[0] = 0.2;
	Q[1] = -0.5;
	Q[2] = 0.1;
	Q[3] = -0.4;
	Q[4] = -0.1;
	Q[5] = 0.4;

	QDot[0] = 0.1;
	QDot[1] = -0.2;
	QDot[2] = 0.1;

	Vector3d point_velocity;
	{
		SUPPRESS_LOGGING;
		point_velocity = CalcPointVelocity (*model, Q, QDot, contact_body_id, contact_point, true);
	}

	// cout << "Point Velocity = " << point_velocity << endl;
	VectorNd qdot_post (QDot.size());
	ComputeContactImpulsesDirect (*model, Q, QDot, constraint_set, qdot_post);
	// cout << LogOutput.str() << endl;
	// cout << "QdotPost = " << qdot_post << endl;

	{
		SUPPRESS_LOGGING;
		point_velocity = CalcPointVelocity (*model, Q, qdot_post, contact_body_id, contact_point, true);
	}

	// cout << "Point Velocity = " << point_velocity << endl;
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (Vector3d (0., 0., 0.), point_velocity, TEST_PREC, TEST_PREC, "");
}

TEST_F(ImpulsesFixture, TestContactImpulseRotatedCollisionVelocity) {
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (1., 0., 0.), NULL, 1.);
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (0., 1., 0.), NULL, 2.);
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (0., 0., 1.), NULL, 3.);

	constraint_set.Bind (*model);

	constraint_set.v_plus[0] = 1.;
	constraint_set.v_plus[1] = 2.;
	constraint_set.v_plus[2] = 3.;

	Q[0] = 0.2;
	Q[1] = -0.5;
	Q[2] = 0.1;
	Q[3] = -0.4;
	Q[4] = -0.1;
	Q[5] = 0.4;

	QDot[0] = 0.1;
	QDot[1] = -0.2;
	QDot[2] = 0.1;

	Vector3d point_velocity;
	{
		SUPPRESS_LOGGING;
		point_velocity = CalcPointVelocity (*model, Q, QDot, contact_body_id, contact_point, true);
	}

	// cout << "Point Velocity = " << point_velocity << endl;

	VectorNd qdot_post (QDot.size());
	ComputeContactImpulsesDirect (*model, Q, QDot, constraint_set, qdot_post);

	// cout << LogOutput.str() << endl;
	// cout << "QdotPost = " << qdot_post << endl;

	{
		SUPPRESS_LOGGING;
		point_velocity = CalcPointVelocity (*model, Q, qdot_post, contact_body_id, contact_point, true);
	}

	// cout << "Point Velocity = " << point_velocity << endl;
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (Vector3d (1., 2., 3.), point_velocity, TEST_PREC, TEST_PREC, "");
}

TEST_F(ImpulsesFixture, TestContactImpulseRangeSpaceSparse) {
Q[0] = 0.2;
	Q[1] = -0.5;
	Q[2] = 0.1;
	Q[3] = -0.4;
	Q[4] = -0.1;
	Q[5] = 0.4;

	QDot[0] = 0.1;
	QDot[1] = -0.2;
	QDot[2] = 0.1;

	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (1., 0., 0.), NULL, 1.);
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (0., 1., 0.), NULL, 2.);
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (0., 0., 1.), NULL, 3.);

	constraint_set.Bind (*model);

	constraint_set.v_plus[0] = 1.;
	constraint_set.v_plus[1] = 2.;
	constraint_set.v_plus[2] = 3.;

	ConstraintSet constraint_set_rangespace;
	constraint_set_rangespace = constraint_set.Copy();
	constraint_set_rangespace.Bind (*model);

	VectorNd qdot_post_direct (QDot.size());
	ComputeContactImpulsesDirect (*model, Q, QDot, constraint_set, qdot_post_direct);

	VectorNd qdot_post_rangespace (QDot.size());
	ComputeContactImpulsesRangeSpaceSparse (*model, Q, QDot, constraint_set_rangespace, qdot_post_rangespace);

	Vector3d point_velocity_direct = CalcPointVelocity (*model, Q, qdot_post_direct, contact_body_id, contact_point, true);
	Vector3d point_velocity_rangespace = CalcPointVelocity (*model, Q, qdot_post_rangespace, contact_body_id, contact_point, true);

	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (qdot_post_direct, qdot_post_rangespace, TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (Vector3d (1., 2., 3.), point_velocity_rangespace, TEST_PREC, TEST_PREC, "");
}

TEST_F(ImpulsesFixture, TestContactImpulseNullSpace) {
Q[0] = 0.2;
	Q[1] = -0.5;
	Q[2] = 0.1;
	Q[3] = -0.4;
	Q[4] = -0.1;
	Q[5] = 0.4;

	QDot[0] = 0.1;
	QDot[1] = -0.2;
	QDot[2] = 0.1;

	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (1., 0., 0.), NULL, 1.);
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (0., 1., 0.), NULL, 2.);
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (0., 0., 1.), NULL, 3.);

	constraint_set.Bind (*model);

	constraint_set.v_plus[0] = 1.;
	constraint_set.v_plus[1] = 2.;
	constraint_set.v_plus[2] = 3.;

	ConstraintSet constraint_set_nullspace;
	constraint_set_nullspace = constraint_set.Copy();
	constraint_set_nullspace.Bind (*model);

	VectorNd qdot_post_direct (QDot.size());
	ComputeContactImpulsesDirect (*model, Q, QDot, constraint_set, qdot_post_direct);

	VectorNd qdot_post_nullspace (QDot.size());
	ComputeContactImpulsesNullSpace (*model, Q, QDot, constraint_set, qdot_post_nullspace);

	Vector3d point_velocity_direct = CalcPointVelocity (*model, Q, qdot_post_direct, contact_body_id, contact_point, true);
	Vector3d point_velocity_nullspace = CalcPointVelocity (*model, Q, qdot_post_nullspace, contact_body_id, contact_point, true);

	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (qdot_post_direct, qdot_post_nullspace, TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (Vector3d (1., 2., 3.), point_velocity_nullspace, TEST_PREC, TEST_PREC, "");
}
