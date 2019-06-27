/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2015 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <limits>
#include <assert.h>

#include "any_rbdl/Logging.h"

#include "any_rbdl/Model.h"
#include "any_rbdl/Joint.h"

namespace RigidBodyDynamics {

	using namespace Math;
	// NOTE: Adapted for mimic
	ANY_RBDL_DLLAPI
		void jcalc (
				Model &model,
				unsigned int joint_id,
				const VectorNd &q,
				const VectorNd &qdot
				) {
			// exception if we calculate it for the root body
			assert (joint_id > 0);

			// joint vector contains also mimiced joints which are not present in the generalized coordinates
			Joint &joint = model.mJoints[joint_id];
			double q_j = joint.mMimicMult * q[joint.q_index] + joint.mMimicOffset;
			// offset is irrelevant in velocities
			double qdot_j = joint.mMimicMult * qdot[joint.q_index];

			if (joint.mJointType == JointTypeRevoluteX) {
				model.X_J[joint_id] = Xrotx (q_j);
				model.v_J[joint_id][0] = qdot_j;
			} else if (joint.mJointType == JointTypeRevoluteY) {
				model.X_J[joint_id] = Xroty (q_j);
				model.v_J[joint_id][1] = qdot_j;
			} else if (joint.mJointType == JointTypeRevoluteZ) {
				model.X_J[joint_id] = Xrotz (q_j);
				model.v_J[joint_id][2] = qdot_j;
			} else if (joint.mDoFCount == 1) {
				model.X_J[joint_id] = jcalc_XJ (model, joint_id, q);
				model.v_J[joint_id] = model.S[joint_id] * qdot_j;
			} else if (joint.mJointType == JointTypeSpherical) {
				model.X_J[joint_id] = SpatialTransform ( model.GetQuaternion (joint_id, q).toMatrix(), Vector3d (0., 0., 0.));
				model.multdof3_S[joint_id](0,0) = 1.;
				model.multdof3_S[joint_id](1,1) = 1.;
				model.multdof3_S[joint_id](2,2) = 1.;

				Vector3d omega (qdot_j,
								joint.mMimicMult * qdot[joint.q_index+1],
								joint.mMimicMult * qdot[joint.q_index+2]);

				model.v_J[joint_id] = SpatialVector (
						omega[0], omega[1], omega[2],
						0., 0., 0.);
			} else if (joint.mJointType == JointTypeEulerZYX) {
				double q0 = q_j;
				double q1 = joint.mMimicMult * q[joint.q_index+1] + joint.mMimicOffset;
				double q2 = joint.mMimicMult * q[joint.q_index+2] + joint.mMimicOffset;

				double s0 = sin (q0);
				double c0 = cos (q0);
				double s1 = sin (q1);
				double c1 = cos (q1);
				double s2 = sin (q2);
				double c2 = cos (q2);

				model.X_J[joint_id].E = Matrix3d(
						c0 * c1, s0 * c1, -s1,
						c0 * s1 * s2 - s0 * c2, s0 * s1 * s2 + c0 * c2, c1 * s2,
						c0 * s1 * c2 + s0 * s2, s0 * s1 * c2 - c0 * s2, c1 * c2
						);

				model.multdof3_S[joint_id](0,0) = -s1;
				model.multdof3_S[joint_id](0,2) = 1.;

				model.multdof3_S[joint_id](1,0) = c1 * s2;
				model.multdof3_S[joint_id](1,1) = c2;

				model.multdof3_S[joint_id](2,0) = c1 * c2;
				model.multdof3_S[joint_id](2,1) = - s2;

				double qdot0 = qdot_j;
				double qdot1 = joint.mMimicMult * qdot[joint.q_index+1];
				double qdot2 = joint.mMimicMult * qdot[joint.q_index+2];

				model.v_J[joint_id] = model.multdof3_S[joint_id] * Vector3d (qdot0, qdot1, qdot2);

				model.c_J[joint_id].set(
						- c1 * qdot0 * qdot1,
						-s1 * s2 * qdot0 * qdot1 + c1 * c2 * qdot0 * qdot2 - s2 * qdot1 * qdot2,
						-s1 * c2 * qdot0 * qdot1 - c1 * s2 * qdot0 * qdot2 - c2 * qdot1 * qdot2,
						0., 0., 0.
						);
			} else if (joint.mJointType == JointTypeEulerXYZ) {
				double q0 = q_j;
				double q1 = joint.mMimicMult * q[joint.q_index+1] + joint.mMimicOffset;
				double q2 = joint.mMimicMult * q[joint.q_index+2] + joint.mMimicOffset;

				double s0 = sin (q0);
				double c0 = cos (q0);
				double s1 = sin (q1);
				double c1 = cos (q1);
				double s2 = sin (q2);
				double c2 = cos (q2);

				model.X_J[joint_id].E = Matrix3d(
						c2 * c1, s2 * c0 + c2 * s1 * s0, s2 * s0 - c2 * s1 * c0,
						-s2 * c1, c2 * c0 - s2 * s1 * s0, c2 * s0 + s2 * s1 * c0,
						s1, -c1 * s0, c1 * c0
						);

				model.multdof3_S[joint_id](0,0) = c2 * c1;
				model.multdof3_S[joint_id](0,1) = s2;

				model.multdof3_S[joint_id](1,0) = -s2 * c1;
				model.multdof3_S[joint_id](1,1) = c2;

				model.multdof3_S[joint_id](2,0) = s1;
				model.multdof3_S[joint_id](2,2) = 1.;

				double qdot0 = qdot_j;
				double qdot1 = joint.mMimicMult * qdot[joint.q_index+1];
				double qdot2 = joint.mMimicMult * qdot[joint.q_index+2];

				model.v_J[joint_id] = model.multdof3_S[joint_id] * Vector3d (qdot0, qdot1, qdot2);

				model.c_J[joint_id].set(
						-s2 * c1 * qdot2 * qdot0 - c2 * s1 * qdot1 * qdot0 + c2 * qdot2 * qdot1,
						-c2 * c1 * qdot2 * qdot0 + s2 * s1 * qdot1 * qdot0 - s2 * qdot2 * qdot1,
						c1 * qdot1 * qdot0,
						0., 0., 0.
						);
			} else if (joint.mJointType == JointTypeEulerYXZ) {
				double q0 = q_j;
				double q1 = joint.mMimicMult * q[joint.q_index+1] + joint.mMimicOffset;
				double q2 = joint.mMimicMult * q[joint.q_index+2] + joint.mMimicOffset;

				double s0 = sin (q0);
				double c0 = cos (q0);
				double s1 = sin (q1);
				double c1 = cos (q1);
				double s2 = sin (q2);
				double c2 = cos (q2);

				model.X_J[joint_id].E = Matrix3d(
						c2 * c0 + s2 * s1 * s0, s2 * c1, -c2 * s0 + s2 * s1 * c0,
						-s2 * c0 + c2 * s1 * s0, c2 * c1, s2 * s0 + c2 * s1 * c0,
						c1 * s0, - s1, c1 * c0
						);
				model.multdof3_S[joint_id](0,0) = s2 * c1;
				model.multdof3_S[joint_id](0,1) = c2;

				model.multdof3_S[joint_id](1,0) = c2 * c1;
				model.multdof3_S[joint_id](1,1) = -s2;

				model.multdof3_S[joint_id](2,0) = -s1;
				model.multdof3_S[joint_id](2,2) = 1.;

				double qdot0 = qdot_j;
				double qdot1 = joint.mMimicMult * qdot[joint.q_index+1];
				double qdot2 = joint.mMimicMult * qdot[joint.q_index+2];

				model.v_J[joint_id] = model.multdof3_S[joint_id] * Vector3d (qdot0, qdot1, qdot2);

				model.c_J[joint_id].set(
						 c2 * c1 * qdot2 * qdot0 - s2 * s1 * qdot1 * qdot0 - s2 * qdot2 * qdot1,
						-s2 * c1 * qdot2 * qdot0 - c2 * s1 * qdot1 * qdot0 - c2 * qdot2 * qdot1,
						-c1 * qdot1 * qdot0,
						0., 0., 0.
						);
			} else if (joint.mJointType == JointTypeTranslationXYZ) {
				double q0 = q_j;
				double q1 = joint.mMimicMult * q[joint.q_index+1] + joint.mMimicOffset;
				double q2 = joint.mMimicMult * q[joint.q_index+2] + joint.mMimicOffset;

				model.X_J[joint_id].E = Matrix3d::Identity();
				model.X_J[joint_id].r = Vector3d (q0, q1, q2);

				model.multdof3_S[joint_id](3,0) = 1.;
				model.multdof3_S[joint_id](4,1) = 1.;
				model.multdof3_S[joint_id](5,2) = 1.;

				double qdot0 = qdot_j;
				double qdot1 = joint.mMimicMult * qdot[joint.q_index+1];
				double qdot2 = joint.mMimicMult * qdot[joint.q_index+2];

				model.v_J[joint_id] = model.multdof3_S[joint_id] * Vector3d (qdot0, qdot1, qdot2);

				model.c_J[joint_id].set(0., 0., 0., 0., 0., 0.);
			} else {
				std::cerr << "Error: invalid joint type " << joint.mJointType << " at id " << joint_id << std::endl;
				abort();
			}

			model.X_lambda[joint_id] = model.X_J[joint_id] * model.X_T[joint_id];
		}
	// NOTE: Adapted for mimic
	ANY_RBDL_DLLAPI
		Math::SpatialTransform jcalc_XJ (
				Model &model,
				unsigned int joint_id,
				const Math::VectorNd &q) {
			// exception if we calculate it for the root body
			assert (joint_id > 0);

			// joint vector contains also mimiced joints which are not present in the generalized coordinates
			Joint &joint = model.mJoints[joint_id];
			double q_j = joint.mMimicMult * q[joint.q_index] + joint.mMimicOffset;

			if (joint.mDoFCount == 1) {
				if (joint.mJointType == JointTypeRevolute) {
					return Xrot (q_j, Vector3d (
								joint.mJointAxes[0][0],
								joint.mJointAxes[0][1],
								joint.mJointAxes[0][2]
								));
				} else if (joint.mJointType == JointTypePrismatic) {
					return Xtrans ( Vector3d (
								joint.mJointAxes[0][3] * q_j,
								joint.mJointAxes[0][4] * q_j,
								joint.mJointAxes[0][5] * q_j
								)
							);
				}
			}
			std::cerr << "Error: invalid joint type!" << std::endl;
			abort();
			return SpatialTransform();
		}
	// NOTE: Adapted for mimic
	ANY_RBDL_DLLAPI
		void jcalc_X_lambda_S (
				Model &model,
				unsigned int joint_id,
				const VectorNd &q
				) {
			// exception if we calculate it for the root body
			assert (joint_id > 0);

			// joint vector contains also mimiced joints which are not present in the generalized coordinates
			Joint &joint = model.mJoints[joint_id];
			double q_j = joint.mMimicMult * q[joint.q_index] + joint.mMimicOffset;

			if (joint.mJointType == JointTypeRevoluteX) {
				model.X_lambda[joint_id] = Xrotx (q_j) * model.X_T[joint_id];
				model.S[joint_id] = joint.mJointAxes[0];
			} else if (joint.mJointType == JointTypeRevoluteY) {
				model.X_lambda[joint_id] = Xroty (q_j) * model.X_T[joint_id];
				model.S[joint_id] = joint.mJointAxes[0];
			} else if (joint.mJointType == JointTypeRevoluteZ) {
				model.X_lambda[joint_id] = Xrotz (q_j) * model.X_T[joint_id];
				model.S[joint_id] = joint.mJointAxes[0];
			} else if (joint.mDoFCount == 1) {
				model.X_lambda[joint_id] = jcalc_XJ (model, joint_id, q) * model.X_T[joint_id];

				// Set the joint axis
				model.S[joint_id] = joint.mJointAxes[0];
			} else if (joint.mJointType == JointTypeSpherical) {
				model.X_lambda[joint_id] = SpatialTransform (
						model.GetQuaternion (joint_id, q).toMatrix(),
						Vector3d (0., 0., 0.))
						* model.X_T[joint_id];

				model.multdof3_S[joint_id].setZero();

				model.multdof3_S[joint_id](0,0) = 1.;
				model.multdof3_S[joint_id](1,1) = 1.;
				model.multdof3_S[joint_id](2,2) = 1.;
			} else if (joint.mJointType == JointTypeEulerZYX) {
				double q0 = q_j;
				double q1 = joint.mMimicMult * q[joint.q_index+1] + joint.mMimicOffset;
				double q2 = joint.mMimicMult * q[joint.q_index+2] + joint.mMimicOffset;

				double s0 = sin (q0);
				double c0 = cos (q0);
				double s1 = sin (q1);
				double c1 = cos (q1);
				double s2 = sin (q2);
				double c2 = cos (q2);

				model.X_lambda[joint_id] = SpatialTransform (
						Matrix3d(
							c0 * c1, s0 * c1, -s1,
							c0 * s1 * s2 - s0 * c2, s0 * s1 * s2 + c0 * c2, c1 * s2,
							c0 * s1 * c2 + s0 * s2, s0 * s1 * c2 - c0 * s2, c1 * c2
							),
						Vector3d (0., 0., 0.))
					* model.X_T[joint_id];

				model.multdof3_S[joint_id].setZero();

				model.multdof3_S[joint_id](0,0) = -s1;
				model.multdof3_S[joint_id](0,2) = 1.;

				model.multdof3_S[joint_id](1,0) = c1 * s2;
				model.multdof3_S[joint_id](1,1) = c2;

				model.multdof3_S[joint_id](2,0) = c1 * c2;
				model.multdof3_S[joint_id](2,1) = - s2;
			} else if (model.mJoints[joint_id].mJointType == JointTypeEulerXYZ) {
				double q0 = q_j;
				double q1 = joint.mMimicMult * q[joint.q_index+1] + joint.mMimicOffset;
				double q2 = joint.mMimicMult * q[joint.q_index+2] + joint.mMimicOffset;

				double s0 = sin (q0);
				double c0 = cos (q0);
				double s1 = sin (q1);
				double c1 = cos (q1);
				double s2 = sin (q2);
				double c2 = cos (q2);

				model.X_lambda[joint_id] = SpatialTransform (
						Matrix3d(
							c2 * c1, s2 * c0 + c2 * s1 * s0, s2 * s0 - c2 * s1 * c0,
							-s2 * c1, c2 * c0 - s2 * s1 * s0, c2 * s0 + s2 * s1 * c0,
							s1, -c1 * s0, c1 * c0
							),
						Vector3d (0., 0., 0.))
					* model.X_T[joint_id];

				model.multdof3_S[joint_id].setZero();

				model.multdof3_S[joint_id](0,0) = c2 * c1;
				model.multdof3_S[joint_id](0,1) = s2;

				model.multdof3_S[joint_id](1,0) = -s2 * c1;
				model.multdof3_S[joint_id](1,1) = c2;

				model.multdof3_S[joint_id](2,0) = s1;
				model.multdof3_S[joint_id](2,2) = 1.;
			} else if (model.mJoints[joint_id].mJointType == JointTypeEulerYXZ ) {
				double q0 = q_j;
				double q1 = joint.mMimicMult * q[joint.q_index+1] + joint.mMimicOffset;
				double q2 = joint.mMimicMult * q[joint.q_index+2] + joint.mMimicOffset;

				double s0 = sin (q0);
				double c0 = cos (q0);
				double s1 = sin (q1);
				double c1 = cos (q1);
				double s2 = sin (q2);
				double c2 = cos (q2);

				model.X_lambda[joint_id] = SpatialTransform (
						Matrix3d(
						c2 * c0 + s2 * s1 * s0, s2 * c1, -c2 * s0 + s2 * s1 * c0,
						-s2 * c0 + c2 * s1 * s0, c2 * c1, s2 * s0 + c2 * s1 * c0,
						c1 * s0, - s1, c1 * c0
							),
						Vector3d (0., 0., 0.))
					* model.X_T[joint_id];

				model.multdof3_S[joint_id].setZero();

				model.multdof3_S[joint_id](0,0) = s2 * c1;
				model.multdof3_S[joint_id](0,1) = c2;

				model.multdof3_S[joint_id](1,0) = c2 * c1;
				model.multdof3_S[joint_id](1,1) = -s2;

				model.multdof3_S[joint_id](2,0) = -s1;
				model.multdof3_S[joint_id](2,2) = 1.;
			} else if (model.mJoints[joint_id].mJointType == JointTypeTranslationXYZ ) {
				double q0 = q_j;
				double q1 = joint.mMimicMult * q[joint.q_index+1] + joint.mMimicOffset;
				double q2 = joint.mMimicMult * q[joint.q_index+2] + joint.mMimicOffset;

				model.X_lambda[joint_id] = SpatialTransform (
						Matrix3d::Identity (3,3),
						Vector3d (q0, q1, q2))
					* model.X_T[joint_id];

				model.multdof3_S[joint_id].setZero();

				model.multdof3_S[joint_id](3,0) = 1.;
				model.multdof3_S[joint_id](4,1) = 1.;
				model.multdof3_S[joint_id](5,2) = 1.;
			} else {
				std::cerr << "Error: invalid joint type!" << std::endl;
				abort();
			}
		}
}
