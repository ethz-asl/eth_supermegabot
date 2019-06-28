
#include <gtest/gtest.h>
#include <kindr/common/gtest_eigen.hpp>

#include <iostream>

#include "any_rbdl/rbdl_mathutils.h"
#include "any_rbdl/Body.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-7;

/* Tests whether the spatial inertia matches the one specified by its
 * parameters
 */
TEST ( Body, TestComputeSpatialInertiaFromAbsoluteRadiiGyration ) {
	Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));

	Matrix3d inertia_C (
			1.4, 0., 0.,
			0., 2., 0.,
			0., 0., 3.);

	SpatialMatrix reference_inertia (
			4.843, -1.98, -2.145, 0, -1.43, 1.32,
			-1.98, 6.334, -1.716, 1.43, 0, -1.65,
			-2.145, -1.716, 7.059, -1.32, 1.65, 0,
			0, 1.43, -1.32, 1.1, 0, 0,
			-1.43, 0, 1.65, 0, 1.1, 0,
			1.32, -1.65, 0, 0, 0, 1.1
			);

//	cout << LogOutput.str() << endl;

	SpatialRigidBodyInertia body_rbi = SpatialRigidBodyInertia::createFromMassComInertiaC (body.GetMass(), body.GetCenterOfMass(), body.GetInertia());

	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (reference_inertia, body_rbi.toMatrix(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (inertia_C, body.GetInertia(), TEST_PREC, TEST_PREC, "");
}

TEST ( Body, TestBodyConstructorMassComInertia ) {
	double mass = 1.1;
	Vector3d com (1.5, 1.2, 1.3);
	Matrix3d inertia_C (
			8.286, -3.96, -4.29,
			-3.96, 10.668, -3.432,
			-4.29, -3.432, 11.118
			);

	Body body (mass, com, inertia_C);

	SpatialMatrix reference_inertia (
			11.729, -5.94, -6.435, 0, -1.43, 1.32,
			-5.94, 15.002, -5.148, 1.43, 0, -1.65,
			-6.435, -5.148, 15.177, -1.32, 1.65, 0,
			0, 1.43, -1.32, 1.1, 0, 0,
			-1.43, 0, 1.65, 0, 1.1, 0,
			1.32, -1.65, 0, 0, 0, 1.1
			);

	SpatialRigidBodyInertia body_rbi = SpatialRigidBodyInertia::createFromMassComInertiaC (body.GetMass(), body.GetCenterOfMass(), body.GetInertia());
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (reference_inertia, body_rbi.toMatrix(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (inertia_C, body.GetInertia(), TEST_PREC, TEST_PREC, "");
}

TEST ( Body, TestBodyJoinNullbody ) {
	ClearLogOutput();
	Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));
	Body nullbody (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));

	Body joined_body = body;
	joined_body.Join (Xtrans(Vector3d (0., 0., 0.)), nullbody);

	SpatialRigidBodyInertia body_rbi (body.GetMass(), body.GetCenterOfMass(), body.GetInertia());
	SpatialRigidBodyInertia joined_body_rbi (joined_body.GetMass(), joined_body.GetCenterOfMass(), joined_body.GetInertia());

	EXPECT_EQ (1.1, body.GetMass());
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (body.GetCenterOfMass(), joined_body.GetCenterOfMass(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (body_rbi.toMatrix(), joined_body_rbi.toMatrix(), TEST_PREC, TEST_PREC, "");
}

TEST ( Body, TestBodyJoinTwoBodies ) {
	ClearLogOutput();
	Body body_a(1.1, Vector3d (-1.1, 1.3, 0.), Vector3d (3.1, 3.2, 3.3));
	Body body_b(1.1, Vector3d (1.1, 1.3, 0.), Vector3d (3.1, 3.2, 3.3));

	Body body_joined (body_a);
	body_joined.Join (Xtrans(Vector3d (0., 0., 0.)), body_b);

	SpatialRigidBodyInertia body_joined_rbi = SpatialRigidBodyInertia::createFromMassComInertiaC (body_joined.GetMass(), body_joined.GetCenterOfMass(), body_joined.GetInertia());

	SpatialMatrix reference_inertia (
			9.918, 0, 0, 0, -0, 2.86,
			0, 9.062, 0, 0, 0, -0,
			0, 0, 12.98, -2.86, 0, 0,
			0, 0, -2.86, 2.2, 0, 0,
			-0, 0, 0, 0, 2.2, 0,
			2.86, -0, 0, 0, 0, 2.2
			);

	EXPECT_EQ (2.2, body_joined.GetMass());
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (Vector3d (0., 1.3, 0.), body_joined.GetCenterOfMass(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (reference_inertia, body_joined_rbi.toMatrix(), TEST_PREC, TEST_PREC, "");
}


TEST ( Body, TestBodyJoinTwoCubes ) {
	/*
	 * This test is for two adjacent cubes, with side length 1, mass 6. The origin of the cubes are at corners, and the cubes are next to each other along the x axis.
	 */
	ClearLogOutput();
	Math::Matrix3d inertia( 1.0, 0.0, 0.0,
							0.0, 1.0, 0.0,
							0.0, 0.0, 1.0);
	Body body_a(6.0, Vector3d (0.5,0.5,0.5), inertia);
	Body body_b(6.0, Vector3d (0.5,0.5,0.5), inertia);

	Body body_joined (body_a);
	body_joined.Join (Xtrans(Vector3d (1., 0., 0.)), body_b);

	Math::Matrix3d reference_inertia_at_com(2.0, 0.0, 0.0,
											0.0, 5.0, 0.0,
											0.0, 0.0, 5.0);

	EXPECT_NEAR (12.0, body_joined.GetMass(), TEST_PREC);
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (reference_inertia_at_com, body_joined.GetInertia(), TEST_PREC, TEST_PREC, "");

	// As a reminder, SpatialRigidBodyInertia is at the origin of the body, not the CoM
	SpatialRigidBodyInertia body_joined_rbia = SpatialRigidBodyInertia::createFromMassComInertiaC (body_joined.GetMass(), body_joined.GetCenterOfMass(), body_joined.GetInertia());

	SpatialMatrix reference_inertia (
			 8.0,  -6.0, -6.0,    0, -6.0,   6.0,
			-6.0,  20.0, -3.0,  6.0,    0, -12.0,
			-6.0,  -3.0, 20.0, -6.0, 12.0,     0,
			   0,   6.0, -6.0, 12.0,    0,     0,
			-6.0,     0, 12.0,    0, 12.0,     0,
			 6.0, -12.0,    0,    0,    0,  12.0
			);

	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (Vector3d (1.0, 0.5, 0.5), body_joined.GetCenterOfMass(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (reference_inertia, body_joined_rbia.toMatrix(), TEST_PREC, TEST_PREC, "");
}

TEST ( Body, TestBodyJoinTwoBodiesDisplaced ) {
	ClearLogOutput();
	Body body_a(1.1, Vector3d (-1.1, 1.3, 0.), Vector3d (3.1, 3.2, 3.3));
	Body body_b(1.1, Vector3d (0., 0., 0.), Vector3d (3.1, 3.2, 3.3));

	Body body_joined (body_a);
	body_joined.Join (Xtrans(Vector3d (1.1, 1.3, 0.)), body_b);

	SpatialRigidBodyInertia body_joined_rbi = SpatialRigidBodyInertia::createFromMassComInertiaC (body_joined.GetMass(), body_joined.GetCenterOfMass(), body_joined.GetInertia());

	SpatialMatrix reference_inertia (
			9.918, 0, 0, 0, -0, 2.86,
			0, 9.062, 0, 0, 0, -0,
			0, 0, 12.98, -2.86, 0, 0,
			0, 0, -2.86, 2.2, 0, 0,
			-0, 0, 0, 0, 2.2, 0,
			2.86, -0, 0, 0, 0, 2.2
			);

	EXPECT_EQ (2.2, body_joined.GetMass());
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (Vector3d (0., 1.3, 0.), body_joined.GetCenterOfMass(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (reference_inertia, body_joined_rbi.toMatrix(), TEST_PREC, TEST_PREC, "");


}

TEST ( Body, TestBodyJoinTwoBodiesRotated ) {
	ClearLogOutput();
	Body body_a(1.1, Vector3d (0., 0., 0.), Vector3d (3.1, 3.2, 3.3));
	Body body_b(1.1, Vector3d (0., 0., 0.), Vector3d (3.1, 3.3, 3.2));

	Body body_joined (body_a);
	body_joined.Join (Xrotx(-M_PI*0.5), body_b);

	SpatialRigidBodyInertia body_joined_rbi (body_joined.GetMass(), body_joined.GetCenterOfMass(), body_joined.GetInertia());

	SpatialMatrix reference_inertia (
			6.2, 0., 0., 0., 0., 0.,
			0., 6.4, 0., 0., 0., 0.,
			0., 0., 6.6, 0., 0., 0.,
			0., 0., 0., 2.2, 0., 0.,
			0., 0., 0., 0., 2.2, 0.,
			0., 0., 0., 0., 0., 2.2
				);

	EXPECT_EQ (2.2, body_joined.GetMass());
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (Vector3d (0., 0., 0.), body_joined.GetCenterOfMass(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (reference_inertia, body_joined_rbi.toMatrix(), TEST_PREC, TEST_PREC, "");
}

TEST ( Body, TestBodyJoinTwoBodiesRotatedAndTranslated ) {
	ClearLogOutput();
	Body body_a(1.1, Vector3d (0., 0., 0.), Vector3d (3.1, 3.2, 3.3));
	Body body_b(1.1, Vector3d (-1., 1., 0.), Vector3d (3.2, 3.1, 3.3));

	Body body_joined (body_a);
	body_joined.Join (Xrotz(M_PI*0.5) * Xtrans(Vector3d (1., 1., 0.)), body_b);

	SpatialRigidBodyInertia body_joined_rbi (body_joined.GetMass(), body_joined.GetCenterOfMass(), body_joined.GetInertia());

	SpatialMatrix reference_inertia (
			6.2, 0., 0., 0., 0., 0.,
			0., 6.4, 0., 0., 0., 0.,
			0., 0., 6.6, 0., 0., 0.,
			0., 0., 0., 2.2, 0., 0.,
			0., 0., 0., 0., 2.2, 0.,
			0., 0., 0., 0., 0., 2.2
				);

	EXPECT_EQ (2.2, body_joined.GetMass());
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (Vector3d (0., 0., 0.), body_joined.GetCenterOfMass(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (reference_inertia, body_joined_rbi.toMatrix(), TEST_PREC, TEST_PREC, "");
}

TEST ( Body, TestBodyConstructorSpatialRigidBodyInertiaMultiplyMotion ) {
	Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));

	SpatialRigidBodyInertia rbi = SpatialRigidBodyInertia(
				body.GetMass(),
				body.GetCenterOfMass() * body.GetMass(),
				body.GetInertia()
				);

	SpatialVector mv (1.1, 1.2, 1.3, 1.4, 1.5, 1.6);
	SpatialVector fv_matrix = rbi.toMatrix() * mv;
	SpatialVector fv_rbi = rbi * mv;

	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (
			fv_matrix,
			fv_rbi,
			TEST_PREC, TEST_PREC, ""
			);
}

TEST ( Body, TestBodyConstructorSpatialRigidBodyInertia ) {
	Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));

	SpatialRigidBodyInertia rbi = SpatialRigidBodyInertia(
				body.GetMass(),
				body.GetCenterOfMass() * body.GetMass(),
				body.GetInertia()
				);
	SpatialMatrix spatial_inertia = rbi.toMatrix();

	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (
			spatial_inertia,
			rbi.toMatrix(),
			TEST_PREC, TEST_PREC, ""
			);
}

TEST ( Body, TestBodyConstructorCopySpatialRigidBodyInertia ) {
	Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));

	SpatialRigidBodyInertia rbi = SpatialRigidBodyInertia(
				body.GetMass(),
				body.GetCenterOfMass() * body.GetMass(),
				body.GetInertia()
				);

	SpatialRigidBodyInertia rbi_from_matrix;
	rbi_from_matrix.createFromMatrix (rbi.toMatrix());

//	cout << "Spatial Inertia = " << endl << spatial_inertia << endl;
//	cout << "rbi = " << endl << rbi.toMatrix() << endl;
//	cout << "rbi.m = " << rbi.m << endl;
//	cout << "rbi.h = " << rbi.h.transpose() << endl;
//	cout << "rbi.I = " << endl << rbi.I << endl;

	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (
			rbi.toMatrix(),
			rbi_from_matrix.toMatrix(),
			TEST_PREC, TEST_PREC, ""
			);
}

/* Tests setting of the individual inertial properties of a body
 */
TEST ( Body, TestSetIndividualInertialProperties ) {
	Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));

	Matrix3d inertia_C (
			1.4, 0., 0.,
			0., 2., 0.,
			0., 0., 3.);

	SpatialMatrix reference_inertia (
			4.843, -1.98, -2.145, 0, -1.43, 1.32,
			-1.98, 6.334, -1.716, 1.43, 0, -1.65,
			-2.145, -1.716, 7.059, -1.32, 1.65, 0,
			0, 1.43, -1.32, 1.1, 0, 0,
			-1.43, 0, 1.65, 0, 1.1, 0,
			1.32, -1.65, 0, 0, 0, 1.1
			);

	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (reference_inertia, body.GetSpatialRigidBodyInertia().toMatrix(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (inertia_C, body.GetInertia(), TEST_PREC, TEST_PREC, "");

	double new_mass = 1.1;
	Vector3d new_com (1.5, 1.2, 1.3);
	Matrix3d new_inertia_C (
			8.286, -3.96, -4.29,
			-3.96, 10.668, -3.432,
			-4.29, -3.432, 11.118
			);

	body.SetIndividualInertialProperties(new_mass, new_com, new_inertia_C);

	SpatialMatrix new_reference_inertia (
			11.729, -5.94, -6.435, 0, -1.43, 1.32,
			-5.94, 15.002, -5.148, 1.43, 0, -1.65,
			-6.435, -5.148, 15.177, -1.32, 1.65, 0,
			0, 1.43, -1.32, 1.1, 0, 0,
			-1.43, 0, 1.65, 0, 1.1, 0,
			1.32, -1.65, 0, 0, 0, 1.1
			);

	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (new_reference_inertia, body.GetSpatialRigidBodyInertia().toMatrix(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (new_inertia_C, body.GetInertia(), TEST_PREC, TEST_PREC, "");
}

/* Tests setting of the individual mass of a body
 */
TEST ( Body, TestSetIndividualMass ) {
	double mass = 1.5;
	Vector3d com (1.5, 1.2, 1.3);
	Matrix3d inertia_C (
			8.286, -3.96, -4.29,
			-3.96, 10.668, -3.432,
			-4.29, -3.432, 11.118
			);

	Body body (mass, com, inertia_C);

	double new_mass = 1.1;

	body.SetIndividualMass(new_mass);

	SpatialMatrix new_reference_inertia (
			11.729, -5.94, -6.435, 0, -1.43, 1.32,
			-5.94, 15.002, -5.148, 1.43, 0, -1.65,
			-6.435, -5.148, 15.177, -1.32, 1.65, 0,
			0, 1.43, -1.32, 1.1, 0, 0,
			-1.43, 0, 1.65, 0, 1.1, 0,
			1.32, -1.65, 0, 0, 0, 1.1
			);

	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (new_reference_inertia, body.GetSpatialRigidBodyInertia().toMatrix(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (inertia_C, body.GetInertia(), TEST_PREC, TEST_PREC, "");
}

/* Tests setting of the individual center of mass of a body
 */
TEST ( Body, TestSetIndividualCenterOfMass ) {
	double mass = 1.1;
	Vector3d com (2.5, 2.2, 2.3);
	Matrix3d inertia_C (
			8.286, -3.96, -4.29,
			-3.96, 10.668, -3.432,
			-4.29, -3.432, 11.118
			);

	Body body (mass, com, inertia_C);

	Vector3d new_com (1.5, 1.2, 1.3);

	body.SetIndividualCenterOfMass(new_com);

	SpatialMatrix new_reference_inertia (
			11.729, -5.94, -6.435, 0, -1.43, 1.32,
			-5.94, 15.002, -5.148, 1.43, 0, -1.65,
			-6.435, -5.148, 15.177, -1.32, 1.65, 0,
			0, 1.43, -1.32, 1.1, 0, 0,
			-1.43, 0, 1.65, 0, 1.1, 0,
			1.32, -1.65, 0, 0, 0, 1.1
			);

	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (new_reference_inertia, body.GetSpatialRigidBodyInertia().toMatrix(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (inertia_C, body.GetInertia(), TEST_PREC, TEST_PREC, "");
}

/* Tests setting of the individual inertia of a body
 */
TEST ( Body, TestSetIndividualInertia ) {
	double mass = 1.1;
	Vector3d com (1.5, 1.2, 1.3);
	Matrix3d inertia_C (
			1.286, -1.96, -1.29,
			-1.96, 1.668, -1.432,
			-1.29, -1.432, 1.118
			);

	Body body (mass, com, inertia_C);

	Matrix3d new_inertia_C (
			8.286, -3.96, -4.29,
			-3.96, 10.668, -3.432,
			-4.29, -3.432, 11.118
			);

	body.SetIndividualInertia(new_inertia_C);

	SpatialMatrix new_reference_inertia (
			11.729, -5.94, -6.435, 0, -1.43, 1.32,
			-5.94, 15.002, -5.148, 1.43, 0, -1.65,
			-6.435, -5.148, 15.177, -1.32, 1.65, 0,
			0, 1.43, -1.32, 1.1, 0, 0,
			-1.43, 0, 1.65, 0, 1.1, 0,
			1.32, -1.65, 0, 0, 0, 1.1
			);

	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (new_reference_inertia, body.GetSpatialRigidBodyInertia().toMatrix(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (new_inertia_C, body.GetInertia(), TEST_PREC, TEST_PREC, "");
}

/* Tests joining of a body to a fixed body, as is the new structure for fixed bodies
 */
TEST ( Body, TestBodyJoinBodyAndFixedBody ) {
	ClearLogOutput();
	Body body_a(1.1, Vector3d (-1.1, 1.3, 0.), Vector3d (3.1, 3.2, 3.3));
	Body body_b(1.1, Vector3d (1.1, 1.3, 0.), Vector3d (3.1, 3.2, 3.3));
	FixedBody fixed_body_b = FixedBody::CreateFromBody(body_b);

	body_a.AddChildFixedBody(&fixed_body_b);

	SpatialRigidBodyInertia body_joined_rbi = SpatialRigidBodyInertia::createFromMassComInertiaC (body_a.GetMass(), body_a.GetCenterOfMass(), body_a.GetInertia());

	SpatialMatrix reference_inertia (
			9.918, 0, 0, 0, -0, 2.86,
			0, 9.062, 0, 0, 0, -0,
			0, 0, 12.98, -2.86, 0, 0,
			0, 0, -2.86, 2.2, 0, 0,
			-0, 0, 0, 0, 2.2, 0,
			2.86, -0, 0, 0, 0, 2.2
			);

	EXPECT_EQ (2.2, body_a.GetMass());
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (Vector3d (0., 1.3, 0.), body_a.GetCenterOfMass(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (reference_inertia, body_a.GetSpatialRigidBodyInertia().toMatrix(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (reference_inertia, body_joined_rbi.toMatrix(), TEST_PREC, TEST_PREC, "");
}

/* Tests joining of a body to a fixed body, and modifying the movable body inertial properties
 */
TEST ( Body, TestBodyJoinBodyAndFixedBodySetMovableInertialProperties ) {
	ClearLogOutput();
	Body body_a(0.5, Vector3d (-0.1, 0, 0.), Vector3d (0.1, 0.1, 0.5));
	Body body_b(1.1, Vector3d (1.1, 1.3, 0.), Vector3d (3.1, 3.2, 3.3));
	FixedBody fixed_body_b = FixedBody::CreateFromBody(body_b);

	body_a.AddChildFixedBody(&fixed_body_b);

	double new_mass = 1.1;
	Vector3d new_com(-1.1, 1.3, 0.);
	Math::Matrix3d new_inertia_C(
					3.1, 0., 0.,
					0., 3.2, 0.,
					0., 0., 3.3
					);

	body_a.SetIndividualInertialProperties(new_mass, new_com, new_inertia_C);

	SpatialRigidBodyInertia body_joined_rbi = SpatialRigidBodyInertia::createFromMassComInertiaC (body_a.GetMass(), body_a.GetCenterOfMass(), body_a.GetInertia());

	SpatialMatrix reference_inertia (
			9.918, 0, 0, 0, -0, 2.86,
			0, 9.062, 0, 0, 0, -0,
			0, 0, 12.98, -2.86, 0, 0,
			0, 0, -2.86, 2.2, 0, 0,
			-0, 0, 0, 0, 2.2, 0,
			2.86, -0, 0, 0, 0, 2.2
			);

	EXPECT_EQ (2.2, body_a.GetMass());
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (Vector3d (0., 1.3, 0.), body_a.GetCenterOfMass(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (reference_inertia, body_a.GetSpatialRigidBodyInertia().toMatrix(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (reference_inertia, body_joined_rbi.toMatrix(), TEST_PREC, TEST_PREC, "");
}

/* Tests joining of a body to a fixed body, and modifying the fixed body inertial properties
 */
TEST ( Body, TestBodyJoinBodyAndFixedBodySetFixedInertialProperties ) {
	ClearLogOutput();
	Body body_a(1.1, Vector3d (-1.1, 1.3, 0.), Vector3d (3.1, 3.2, 3.3));
	Body body_b(0.0, Vector3d (0.0, 0.0, 0.0), Vector3d (0.0, 0.0, 0.0));
	FixedBody fixed_body_b = FixedBody::CreateFromBody(body_b);

	body_a.AddChildFixedBody(&fixed_body_b);

	double new_mass = 1.1;
	Vector3d new_com(1.1, 1.3, 0.);
	Math::Matrix3d new_inertia_C(
					3.1, 0., 0.,
					0., 3.2, 0.,
					0., 0., 3.3
					);

	fixed_body_b.SetIndividualInertialProperties(new_mass, new_com, new_inertia_C);

	SpatialRigidBodyInertia body_joined_rbi = SpatialRigidBodyInertia::createFromMassComInertiaC (body_a.GetMass(), body_a.GetCenterOfMass(), body_a.GetInertia());

	SpatialMatrix reference_inertia (
			9.918, 0, 0, 0, -0, 2.86,
			0, 9.062, 0, 0, 0, -0,
			0, 0, 12.98, -2.86, 0, 0,
			0, 0, -2.86, 2.2, 0, 0,
			-0, 0, 0, 0, 2.2, 0,
			2.86, -0, 0, 0, 0, 2.2
			);

	EXPECT_EQ (2.2, body_a.GetMass());
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (Vector3d (0., 1.3, 0.), body_a.GetCenterOfMass(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (reference_inertia, body_a.GetSpatialRigidBodyInertia().toMatrix(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (reference_inertia, body_joined_rbi.toMatrix(), TEST_PREC, TEST_PREC, "");
}

/* Tests joining of a body to a fixed body and absorbing the fixed body
 */
TEST ( Body, TestBodyJoinBodyAndFixedBodyAbsorb ) {
	ClearLogOutput();
	Body body_a(1.1, Vector3d (-1.1, 1.3, 0.), Vector3d (3.1, 3.2, 3.3));
	Body body_b(1.1, Vector3d (1.1, 1.3, 0.), Vector3d (3.1, 3.2, 3.3));
	FixedBody fixed_body_b = FixedBody::CreateFromBody(body_b);

	body_a.AddChildFixedBody(&fixed_body_b);
	body_a.AbsorbFixedChildren();

	SpatialRigidBodyInertia body_joined_rbi = SpatialRigidBodyInertia::createFromMassComInertiaC (body_a.GetMass(), body_a.GetCenterOfMass(), body_a.GetInertia());

	SpatialMatrix reference_inertia (
			9.918, 0, 0, 0, -0, 2.86,
			0, 9.062, 0, 0, 0, -0,
			0, 0, 12.98, -2.86, 0, 0,
			0, 0, -2.86, 2.2, 0, 0,
			-0, 0, 0, 0, 2.2, 0,
			2.86, -0, 0, 0, 0, 2.2
			);

	EXPECT_EQ (2.2, body_a.GetMass());
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (Vector3d (0., 1.3, 0.), body_a.GetCenterOfMass(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (reference_inertia, body_a.GetSpatialRigidBodyInertia().toMatrix(), TEST_PREC, TEST_PREC, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (reference_inertia, body_joined_rbi.toMatrix(), TEST_PREC, TEST_PREC, "");
	EXPECT_EQ (2.2, body_a.GetIndividualMass());
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (Vector3d (0., 1.3, 0.), body_a.GetIndividualCenterOfMass(), TEST_PREC, TEST_PREC, "");
	EXPECT_EQ (0.0, fixed_body_b.GetIndividualMass());
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (Vector3d (0., 0.0, 0.0), fixed_body_b.GetIndividualCenterOfMass(), TEST_PREC, TEST_PREC, "");
}
