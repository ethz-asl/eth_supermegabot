/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2015 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <limits>
#include <assert.h>
#include <string.h>

#include "any_rbdl/rbdl_mathutils.h"
#include "any_rbdl/Logging.h"

#include "any_rbdl/Model.h"
#include "any_rbdl/Joint.h"
#include "any_rbdl/Body.h"
#include "any_rbdl/Dynamics.h"
#include "any_rbdl/Kinematics.h"

namespace RigidBodyDynamics {

using namespace Math;

ANY_RBDL_DLLAPI
void ForwardDynamics (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		VectorNd &QDDot,
		std::vector<SpatialVector> *f_ext
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	SpatialVector spatial_gravity (0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);

	unsigned int i = 0;

	LOG << "Q          = " << Q.transpose() << std::endl;
	LOG << "QDot       = " << QDot.transpose() << std::endl;
	LOG << "Tau        = " << Tau.transpose() << std::endl;
	LOG << "---" << std::endl;

	// Reset the velocity of the root body
	model.v[0].setZero();

	for (i = 1; i < model.mBodies.size(); i++) {
		unsigned int lambda = model.lambda[i];

		jcalc (model, i, Q, QDot);

		if (lambda != 0)
			model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
		else
			model.X_base[i] = model.X_lambda[i];

		model.v[i] = model.X_lambda[i].apply( model.v[lambda]) + model.v_J[i];

		/*
		LOG << "X_J (" << i << "):" << std::endl << X_J << std::endl;
		LOG << "v_J (" << i << "):" << std::endl << v_J << std::endl;
		LOG << "v_lambda" << i << ":" << std::endl << model.v.at(lambda) << std::endl;
		LOG << "X_base (" << i << "):" << std::endl << model.X_base[i] << std::endl;
		LOG << "X_lambda (" << i << "):" << std::endl << model.X_lambda[i] << std::endl;
		LOG << "SpatialVelocity (" << i << "): " << model.v[i] << std::endl;
		*/

		model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);
		model.I[i].setSpatialMatrix (model.IA[i]);

		model.pA[i] = crossf(model.v[i],model.I[i] * model.v[i]);

		if (f_ext != NULL && (*f_ext)[i] != SpatialVectorZero) {
			LOG << "External force (" << i << ") = " << model.X_base[i].toMatrixAdjoint() * (*f_ext)[i] << std::endl;
			model.pA[i] -= model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
		}
	}

// ClearLogOutput();

	LOG << "--- first loop ---" << std::endl;

	for (i = model.mBodies.size() - 1; i > 0; i--) {
		unsigned int q_index = model.mJoints[i].q_index;

		if (model.mJoints[i].mDoFCount == 3) {
			model.multdof3_U[i] = model.IA[i] * model.multdof3_S[i];
#ifdef EIGEN_CORE_H
			model.multdof3_Dinv[i] = (model.multdof3_S[i].transpose() * model.multdof3_U[i]).inverse().eval();
#else
			model.multdof3_Dinv[i] = (model.multdof3_S[i].transpose() * model.multdof3_U[i]).inverse();
#endif
			Vector3d tau_temp (Tau[q_index], Tau[q_index + 1], Tau[q_index + 2]);

			model.multdof3_u[i] = tau_temp - model.multdof3_S[i].transpose() * model.pA[i];

//			LOG << "multdof3_u[" << i << "] = " << model.multdof3_u[i].transpose() << std::endl;
			unsigned int lambda = model.lambda[i];
			if (lambda != 0) {
				SpatialMatrix Ia = model.IA[i] - model.multdof3_U[i] * model.multdof3_Dinv[i] * model.multdof3_U[i].transpose();
				SpatialVector pa = model.pA[i] + Ia * model.c[i] + model.multdof3_U[i] * model.multdof3_Dinv[i] * model.multdof3_u[i];
#ifdef EIGEN_CORE_H
				model.IA[lambda].noalias() += model.X_lambda[i].toMatrixTranspose() * Ia * model.X_lambda[i].toMatrix();
				model.pA[lambda].noalias() += model.X_lambda[i].applyTranspose(pa);
#else
				model.IA[lambda] += model.X_lambda[i].toMatrixTranspose() * Ia * model.X_lambda[i].toMatrix();
				model.pA[lambda] += model.X_lambda[i].applyTranspose(pa);
#endif
				LOG << "pA[" << lambda << "] = " << model.pA[lambda].transpose() << std::endl;
			}
		} else {
			model.U[i] = model.IA[i] * model.S[i];
			model.d[i] = model.S[i].dot(model.U[i]);
			model.u[i] = Tau[q_index] - model.S[i].dot(model.pA[i]);
//			LOG << "u[" << i << "] = " << model.u[i] << std::endl;

			unsigned int lambda = model.lambda[i];
			if (lambda != 0) {
				SpatialMatrix Ia = model.IA[i] - model.U[i] * (model.U[i] / model.d[i]).transpose();
				SpatialVector pa = model.pA[i] + Ia * model.c[i] + model.U[i] * model.u[i] / model.d[i];
#ifdef EIGEN_CORE_H
				model.IA[lambda].noalias() += model.X_lambda[i].toMatrixTranspose() * Ia * model.X_lambda[i].toMatrix();
				model.pA[lambda].noalias() += model.X_lambda[i].applyTranspose(pa);
#else
				model.IA[lambda] += model.X_lambda[i].toMatrixTranspose() * Ia * model.X_lambda[i].toMatrix();
				model.pA[lambda] += model.X_lambda[i].applyTranspose(pa);
#endif
				LOG << "pA[" << lambda << "] = " << model.pA[lambda].transpose() << std::endl;
			}
		}
	}

//	ClearLogOutput();

	model.a[0] = spatial_gravity * -1.;

	for (i = 1; i < model.mBodies.size(); i++) {
		unsigned int q_index = model.mJoints[i].q_index;
		unsigned int lambda = model.lambda[i];
		SpatialTransform X_lambda = model.X_lambda[i];

		model.a[i] = X_lambda.apply(model.a[lambda]) + model.c[i];
		LOG << "a'[" << i << "] = " << model.a[i].transpose() << std::endl;

		if (model.mJoints[i].mDoFCount == 3) {
			Vector3d qdd_temp = model.multdof3_Dinv[i] * (model.multdof3_u[i] - model.multdof3_U[i].transpose() * model.a[i]);
			QDDot[q_index] = qdd_temp[0];
			QDDot[q_index + 1] = qdd_temp[1];
			QDDot[q_index + 2] = qdd_temp[2];
			model.a[i] = model.a[i] + model.multdof3_S[i] * qdd_temp;
		} else {
			QDDot[q_index] = (1./model.d[i]) * (model.u[i] - model.U[i].dot(model.a[i]));
			model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
		}
	}

	LOG << "QDDot = " << QDDot.transpose() << std::endl;
}

ANY_RBDL_DLLAPI
void ForwardDynamicsLagrangian (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		VectorNd &QDDot,
		Math::LinearSolver linear_solver,
		std::vector<SpatialVector> *f_ext,
		Math::MatrixNd *H,
		Math::VectorNd *C
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	bool free_H = false;
	bool free_C = false;

	if (H == NULL) {
		H = new MatrixNd (MatrixNd::Zero(model.dof_count, model.dof_count));
		free_H = true;
	}

	if (C == NULL) {
		C = new VectorNd (VectorNd::Zero(model.dof_count));
		free_C = true;
	}

	// we set QDDot to zero to compute C properly with the InverseDynamics
	// method.
	QDDot.setZero();

	InverseDynamics (model, Q, QDot, QDDot, (*C), f_ext);
	CompositeRigidBodyAlgorithm (model, Q, *H, false);

	LOG << "A = " << std::endl << *H << std::endl;
	LOG << "b = " << std::endl << *C * -1. + Tau << std::endl;

#ifndef ANY_RBDL_USE_SIMPLE_MATH
	switch (linear_solver) {
		case (LinearSolverPartialPivLU) :
			QDDot = H->partialPivLu().solve (*C * -1. + Tau);
			break;
		case (LinearSolverColPivHouseholderQR) :
			QDDot = H->colPivHouseholderQr().solve (*C * -1. + Tau);
			break;
		case (LinearSolverHouseholderQR) :
			QDDot = H->householderQr().solve (*C * -1. + Tau);
			break;
		case (LinearSolverLLT) :
			QDDot = H->llt().solve (*C * -1. + Tau);
			break;
		default:
			LOG << "Error: Invalid linear solver: " << linear_solver << std::endl;
			assert (0);
			break;
	}
#else
	bool solve_successful = LinSolveGaussElimPivot (*H, *C * -1. + Tau, QDDot);
	assert (solve_successful);
#endif

	if (free_C) {
		delete C;
	}

	if (free_H) {
		delete H;
	}


	LOG << "x = " << QDDot << std::endl;
}

ANY_RBDL_DLLAPI
void GravityTerms (
		Model &model,
		const VectorNd &Q,
		VectorNd &Tau,
		bool updateKinematics
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	if (updateKinematics) {
		UpdateKinematicsCustom(model,&Q,nullptr,nullptr);
	}

	std::vector<Math::SpatialVector> a(model.a.size());
	SpatialVector spatial_gravity (0., 0., 0., -model.gravity[0], -model.gravity[1], -model.gravity[2]);
	a[0] = spatial_gravity;

	for (unsigned int i = 1; i < model.mBodies.size(); ++i) {
		if (model.lambda[i] == 0) {
			a[i] = model.X_lambda[i].apply(spatial_gravity);
		}	else {
			a[i] = model.X_lambda[i].apply(a[model.lambda[i]]);
		}

		if (!model.mBodies[i].mIsVirtual) {
			model.f[i] = model.I[i] * a[i];
		} else {
			model.f[i].setZero();
		}
	}

	// set of already processed mimic joints
	std::unordered_set<unsigned int> processed_mimics;

	for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
		if (model.mJoints[i].mDoFCount == 3) {
			const Eigen::Matrix<double,3,1> tau31 = model.mJoints[i].mMimicMult * model.multdof3_S[i].transpose() * model.f[i];
			RigidBodyDynamics::getMimicVector<3>(Tau.block<3,1>(model.mJoints[i].q_index, 0), tau31, model, model.mJoints[i].q_index, processed_mimics);
		} else {
			const Eigen::Matrix<double,1,1> tau11 = (Eigen::Matrix<double,1,1>() << model.mJoints[i].mMimicMult * model.S[i].dot(model.f[i])).finished();
			RigidBodyDynamics::getMimicVector<1>(Tau.block<1,1>(model.mJoints[i].q_index,0), tau11, model, model.mJoints[i].q_index, processed_mimics);
		}

		if (model.lambda[i] != 0) {
			model.f[model.lambda[i]] = model.f[model.lambda[i]] + model.X_lambda[i].applyTranspose(model.f[i]);
		}
	}
}

ANY_RBDL_DLLAPI
void NonlinearEffects (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		VectorNd &Tau,
		bool updateKinematics
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	if (updateKinematics) {
		UpdateKinematics(model,Q,QDot,Eigen::VectorXd::Zero(model.dof_count));
	}

	std::vector<Math::SpatialVector> a(model.a.size());
	SpatialVector spatial_gravity (0., 0., 0., -model.gravity[0], -model.gravity[1], -model.gravity[2]);
	a[0] = spatial_gravity;

	for (unsigned int i = 1; i < model.mBodies.size(); i++) {
		if (model.lambda[i] == 0) {
			a[i] = model.X_lambda[i].apply(spatial_gravity);
		}	else {
			a[i] = model.X_lambda[i].apply(a[model.lambda[i]]) + model.c[i];
		}

		if (!model.mBodies[i].mIsVirtual) {
			model.f[i] = model.I[i] * a[i] + crossf(model.v[i],model.I[i] * model.v[i]);
		} else {
			model.f[i].setZero();
		}
	}

	// set of already processed mimic joints
	std::unordered_set<unsigned int> processed_mimics;

	for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
		if (model.mJoints[i].mDoFCount == 3) {
			const Eigen::Matrix<double,3,1> tau31 = model.mJoints[i].mMimicMult * model.multdof3_S[i].transpose() * model.f[i];
			RigidBodyDynamics::getMimicVector<3>(Tau.block<3,1>(model.mJoints[i].q_index, 0), tau31, model, model.mJoints[i].q_index, processed_mimics);
		} else {
			const Eigen::Matrix<double,1,1> tau11 = (Eigen::Matrix<double,1,1>() << model.mJoints[i].mMimicMult * model.S[i].dot(model.f[i])).finished();
			RigidBodyDynamics::getMimicVector<1>(Tau.block<1,1>(model.mJoints[i].q_index,0), tau11, model, model.mJoints[i].q_index, processed_mimics);
		}

		if (model.lambda[i] != 0) {
			model.f[model.lambda[i]] = model.f[model.lambda[i]] + model.X_lambda[i].applyTranspose(model.f[i]);
		}
	}
}

ANY_RBDL_DLLAPI
void InverseDynamics (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &QDDot,
		VectorNd &Tau,
		std::vector<SpatialVector> *f_ext
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	// Reset the velocity of the root body
	model.v[0].setZero();
	model.a[0].set (0., 0., 0., -model.gravity[0], -model.gravity[1], -model.gravity[2]);

	for (unsigned int i = 1; i < model.mBodies.size(); i++) {
		unsigned int q_index = model.mJoints[i].q_index;
		unsigned int lambda = model.lambda[i];

		jcalc (model, i, Q, QDot);

		if (lambda != 0) {
			model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
		} else {
			model.X_base[i] = model.X_lambda[i];
		}

		model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + model.v_J[i];
		model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);

		if (model.mJoints[i].mDoFCount == 3) {
			model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i] + model.multdof3_S[i] * Vector3d (QDDot[q_index], QDDot[q_index + 1], QDDot[q_index + 2]);
		} else {
			model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i] + model.S[i] * QDDot[q_index];
		}

		if (!model.mBodies[i].mIsVirtual) {
			model.f[i] = model.I[i] * model.a[i] + crossf(model.v[i],model.I[i] * model.v[i]);
		} else {
			model.f[i].setZero();
		}

		if (f_ext != NULL && (*f_ext)[i] != SpatialVectorZero)
			model.f[i] -= model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
	}

	// set of already processed mimic joints
	std::unordered_set<unsigned int> processed_mimics;

	for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
		if (model.mJoints[i].mDoFCount == 3) {
			const Eigen::Vector3d tau31 = model.mJoints[i].mMimicMult * model.multdof3_S[i].transpose() * model.f[i];
			RigidBodyDynamics::getMimicVector<3>(Tau.block<3,1>(model.mJoints[i].q_index, 0), tau31, model, model.mJoints[i].q_index, processed_mimics);
		} else {
			const Eigen::Matrix<double,1,1> tau11 = (Eigen::Matrix<double,1,1>() << model.mJoints[i].mMimicMult * model.S[i].dot(model.f[i])).finished();
			RigidBodyDynamics::getMimicVector<1>(Tau.block<1,1>(model.mJoints[i].q_index,0), tau11, model, model.mJoints[i].q_index, processed_mimics);
		}

		if (model.lambda[i] != 0) {
			model.f[model.lambda[i]] = model.f[model.lambda[i]] + model.X_lambda[i].applyTranspose(model.f[i]);
		}
	}
}

ANY_RBDL_DLLAPI
void CompositeRigidBodyAlgorithm (Model& model, const VectorNd &Q, MatrixNd &H, bool update_kinematics) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	assert (H.rows() == model.dof_count && H.cols() == model.dof_count);

	for (unsigned int i = 1; i < model.mBodies.size(); i++) {
		if (update_kinematics) {
			jcalc_X_lambda_S (model, i, Q);
		}
		model.Ic[i] = model.I[i];
	}

	// set of already processed mimic joints
	std::set<std::pair<unsigned int, unsigned int>> processed_mimics;

	for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
		if (model.lambda[i] != 0) {
			model.Ic[model.lambda[i]] = model.Ic[model.lambda[i]] + model.X_lambda[i].applyTranspose(model.Ic[i]);
		}

		unsigned int dof_index_i = model.mJoints[i].q_index;

		if (model.mJoints[i].mDoFCount == 3) {
			Matrix63 F_63 = model.Ic[i].toMatrix() * model.multdof3_S[i];
			Matrix3d Htemp = model.mJoints[i].mMimicMult * model.mJoints[i].mMimicMult * model.multdof3_S[i].transpose() * F_63;
			RigidBodyDynamics::getMimicMatrix<3,3>(H.block<3,3>(dof_index_i, dof_index_i), Htemp, model,
												   dof_index_i, dof_index_i, processed_mimics);
			unsigned int j = i;
			unsigned int dof_index_j = dof_index_i;

			while (model.lambda[j] != 0) {
				F_63 = model.X_lambda[j].toMatrixTranspose() * (F_63);
				j = model.lambda[j];
				dof_index_j = model.mJoints[j].q_index;

				if (model.mJoints[j].mDoFCount == 3) {
					Matrix3d H_temp2 = model.mJoints[i].mMimicMult * model.mJoints[j].mMimicMult *F_63.transpose() * (model.multdof3_S[j]);
					RigidBodyDynamics::getMimicMatrix<3,3>(H.block<3,3>(dof_index_i,dof_index_j), H_temp2, model,
														   dof_index_i, dof_index_j, processed_mimics);
					RigidBodyDynamics::getMimicMatrix<3,3>(H.block<3,3>(dof_index_j,dof_index_i), H_temp2.transpose(), model,
														   dof_index_j, dof_index_i, processed_mimics);

				} else {
					Vector3d H_temp2 = model.mJoints[i].mMimicMult * model.mJoints[j].mMimicMult * F_63.transpose() * (model.S[j]);
					RigidBodyDynamics::getMimicMatrix<3,1>(H.block<3,1>(dof_index_i,dof_index_j), H_temp2, model,
														   dof_index_i, dof_index_j, processed_mimics);
					RigidBodyDynamics::getMimicMatrix<1,3>(H.block<1,3>(dof_index_j,dof_index_i), H_temp2.transpose(), model,
														   dof_index_j, dof_index_i, processed_mimics);
				}
			}
		} else {
			// This takes care of adding up the diagonal elements
			SpatialVector F = model.Ic[i] * model.S[i];
			const Eigen::Matrix<double,1,1> Hs = (Eigen::Matrix<double,1,1>() << pow(model.mJoints[i].mMimicMult, 2) * model.S[i].dot(F) ).finished();
			RigidBodyDynamics::getMimicMatrix<1,1>(H.block<1,1>(dof_index_i,dof_index_i), Hs, model,
												   dof_index_i, dof_index_i, processed_mimics);
//			std::cout << "i: " << dof_index_i << " Mii = " << Hs << std::endl;

			unsigned int j = i;
			unsigned int dof_index_j = dof_index_i;

			while (model.lambda[j] != 0) {
				F = model.X_lambda[j].applyTranspose(F);
				j = model.lambda[j];
				dof_index_j = model.mJoints[j].q_index;

				if (model.mJoints[j].mDoFCount == 3) {
					Vector3d H_temp2 = model.mJoints[i].mMimicMult * model.mJoints[j].mMimicMult * (F.transpose() * model.multdof3_S[j]).transpose();

					LOG << F.transpose() << std::endl << model.multdof3_S[j] << std::endl;
					LOG << H_temp2.transpose() << std::endl;
					RigidBodyDynamics::getMimicMatrix<1,3>(H.block<1,3>(dof_index_i,dof_index_j), H_temp2, model,
														   dof_index_i, dof_index_j, processed_mimics);
					RigidBodyDynamics::getMimicMatrix<3,1>(H.block<3,1>(dof_index_j,dof_index_i), H_temp2, model,
														   dof_index_j, dof_index_i, processed_mimics);


				} else {
					const Eigen::Matrix<double,1,1> H11 = (Eigen::Matrix<double,1,1>() << model.mJoints[i].mMimicMult * model.mJoints[j].mMimicMult * F.dot(model.S[j])).finished();
//					std::cout << "i: " << dof_index_i << " j: " << dof_index_j << " Mij = " << H11 << std::endl;
					RigidBodyDynamics::getMimicMatrix<1,1>(H.block<1,1>(dof_index_i,dof_index_j), H11, model,
														   dof_index_i, dof_index_j, processed_mimics);
					RigidBodyDynamics::getMimicMatrix<1,1>(H.block<1,1>(dof_index_j,dof_index_i), H11, model,
														   dof_index_j, dof_index_i, processed_mimics);
//					std::cout <<"H" << std::endl << H << std::endl;
				}
			}
		}
	}
}

} /* namespace RigidBodyDynamics */
