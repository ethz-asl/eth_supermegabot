#include <gtest/gtest.h>
#include <kindr/common/gtest_eigen.hpp>

#include <iostream>

#include "any_rbdl/Logging.h"

#include "any_rbdl/Model.h"
#include "any_rbdl/Dynamics.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-14;

struct InverseDynamicsFixture : public ::testing::Test {
	InverseDynamicsFixture () {
		ClearLogOutput();
		model = new Model;
		model->gravity = Vector3d  (0., -9.81, 0.);
	}
	~InverseDynamicsFixture () {
		delete model;
	}
	Model *model;
};

#ifndef USE_SLOW_SPATIAL_ALGEBRA
TEST_F(InverseDynamicsFixture, TestInverseForwardDynamicsFloatingBase) {
	Body base_body(1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));

	model->SetFloatingBaseBody(base_body);

	// Initialization of the input vectors
	VectorNd Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd Tau = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd TauInv = VectorNd::Constant ((size_t) model->dof_count, 0.);

	Q[0] = 1.1;
	Q[1] = 1.2;
	Q[2] = 1.3;
	Q[3] = 0.1;
	Q[4] = 0.2;
	Q[5] = 0.3;

	QDot[0] = 1.1;
	QDot[1] = -1.2;
	QDot[2] = 1.3;
	QDot[3] = -0.1;
	QDot[4] = 0.2;
	QDot[5] = -0.3;

	Tau[0] = 2.1;
	Tau[1] = 2.2;
	Tau[2] = 2.3;
	Tau[3] = 1.1;
	Tau[4] = 1.2;
	Tau[5] = 1.3;

	ForwardDynamics(*model, Q, QDot, Tau, QDDot);
	InverseDynamics(*model, Q, QDot, QDDot, TauInv);

	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL (Tau, TauInv, TEST_PREC, TEST_PREC, "");
}
#endif
