/*!
* @file    rbdl_kinematics.cpp
* @author  Christian Gehring, Dario Bellicoso
* @date    Sep, 2015
*/

#include "romo_rbdl/rbdl_kinematics.hpp"
#include <unordered_set>

namespace romo {
namespace internal {

enum SpatialAlgebra {
  RowTranslation = 3,
  RowRotation = 0
};

}
}
namespace RigidBodyDynamics {

using namespace Math;

ANY_RBDL_DLLAPI
void CalcSpatialJacobianWorldToPointInWorldFrame (
    Model &model,
    const VectorNd &Q,
    unsigned int body_id,
    const Vector3d &point_position,
    MatrixNd &G,
    bool update_kinematics) {
  LOG << "-------- " << __func__ << " --------" << std::endl;

  // update the Kinematics if necessary
  if (update_kinematics) {
    UpdateKinematicsCustom (model, &Q, NULL, NULL);
  }

  SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(), CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false));

  assert (G.rows() == 6 && G.cols() == model.qdot_size );

  unsigned int reference_body_id = body_id;

  if (model.IsFixedBodyId(body_id)) {
    unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    reference_body_id = model.mFixedBodies[fbody_id]->mMovableParent;
  }

  unsigned int j = reference_body_id;

  // set of already processed mimic joints
  std::unordered_set<unsigned int> processed_mimics;

  // e[j] is set to 1 if joint j contributes to the jacobian that we are
  // computing. For all other joints the column will be zero.
  while (j != 0) {
      unsigned int q_index = model.mJoints[j].q_index;

      // multiplier should be 1.0 when no mimic
      if (model.mJoints[j].mDoFCount == 3) {
          const Eigen::Matrix<double,6,3> j63 = model.mJoints[j].mMimicMult * ((point_trans * model.X_base[j].inverse()).toMatrix() * model.multdof3_S[j]);
          RigidBodyDynamics::getMimicMatrix<6,3>(G.block<6,3>(0,q_index), j63, model, q_index, processed_mimics);
      } else {
          const Eigen::Matrix<double,6,1> j61 = model.mJoints[j].mMimicMult * point_trans.apply(model.X_base[j].inverse().apply(model.S[j]));
          RigidBodyDynamics::getMimicMatrix<6,1>(G.block<6,1>(0,q_index), j61, model, q_index, processed_mimics);
      }

    j = model.lambda[j];
  }
}


ANY_RBDL_DLLAPI
void CalcTranslationalJacobianWorldToPointInWorldFrame (
	Model &model,
	const VectorNd &Q,
	unsigned int body_id,
	const Vector3d &point_position,
	MatrixNd &G,
	bool update_kinematics
  ) {
  LOG << "-------- " << __func__ << " --------" << std::endl;

  // update the Kinematics if necessary
  if (update_kinematics) {
    UpdateKinematicsCustom (model, &Q, NULL, NULL);
  }

  SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(), CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false));

  assert (G.rows() == 3 && G.cols() == model.qdot_size );

  unsigned int reference_body_id = body_id;

  if (model.IsFixedBodyId(body_id)) {
    unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    reference_body_id = model.mFixedBodies[fbody_id]->mMovableParent;
  }

  unsigned int j = reference_body_id;

  // set of already processed mimic joints
  std::unordered_set<unsigned int> processed_mimics;

  // e[j] is set to 1 if joint j contributes to the jacobian that we are
  // computing. For all other joints the column will be zero.

  while (j != 0) {
    unsigned int q_index = model.mJoints[j].q_index;

    // multiplier should be 1.0 when no mimic
    if (model.mJoints[j].mDoFCount == 3) {
      const Eigen::Matrix3d j33 = model.mJoints[j].mMimicMult * ((point_trans * model.X_base[j].inverse()).toMatrix() * model.multdof3_S[j]).block<3,3>(romo::internal::SpatialAlgebra::RowTranslation,0);
      RigidBodyDynamics::getMimicMatrix<3,3>(G.block<3,3>(0, q_index), j33, model, q_index, processed_mimics);
    } else {
      const Eigen::Vector3d j31 = model.mJoints[j].mMimicMult * point_trans.apply(model.X_base[j].inverse().apply(model.S[j])).block<3,1>(romo::internal::SpatialAlgebra::RowTranslation,0);
      RigidBodyDynamics::getMimicMatrix<3,1>(G.block<3,1>(0, q_index), j31, model, q_index, processed_mimics);
    }

    j = model.lambda[j];
  }
}

ANY_RBDL_DLLAPI
void CalcJacobianFloatingBaseToPointOnBodyInWorldFrame(
  Model &model,
  const VectorNd &Q,
  unsigned int body_id,
  const Vector3d &point_position,
  MatrixNd &G,
  romo::internal::SpatialAlgebra type,
  bool update_kinematics
  ) {

  LOG << "-------- " << __func__ << " --------" << std::endl;
  // update the Kinematics if necessary
  if (update_kinematics) {
    UpdateKinematicsCustom (model, &Q, NULL, NULL);
  }

  SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(), CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false));

  // todo: assert number of columns
  assert (G.rows() == 3 && G.cols() == model.qdot_size);

  unsigned int reference_body_id = body_id;

  if (model.IsFixedBodyId(body_id)) {
    unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    reference_body_id = model.mFixedBodies[fbody_id]->mMovableParent;
  }

  unsigned int j = reference_body_id;

  const unsigned int floating_base_id = model.rootId;

  // Reset iterators
  unsigned int h = j;

  // Get q_index of first joint
  while (h != 0 && h!=floating_base_id) {
    h = model.lambda[h];
  }

  // set of already processed mimic joints
  std::unordered_set<unsigned int> processed_mimics;

  // e[j] is set to 1 if joint j contributes to the jacobian that we are
  // computing. For all other joints the column will be zero.
  while (j != 0 && j!=floating_base_id) {
    const unsigned int q_index = model.mJoints[j].q_index;
    if (model.mJoints[j].mDoFCount == 3) {
      const Eigen::Matrix3d j33 = model.mJoints[j].mMimicMult * ((point_trans * model.X_base[j].inverse()).toMatrix() * model.multdof3_S[j]).block<3,3>(type,0);
      RigidBodyDynamics::getMimicMatrix<3,3>(G.block<3,3>(0, q_index), j33, model, q_index, processed_mimics);
    } else {
      const Eigen::Vector3d j31 = model.mJoints[j].mMimicMult * point_trans.apply(model.X_base[j].inverse().apply(model.S[j])).block<3,1>(type,0);
      RigidBodyDynamics::getMimicMatrix<3,1>(G.block<3,1>(0, q_index), j31, model, q_index, processed_mimics);
    }
    j = model.lambda[j];
  }
}



ANY_RBDL_DLLAPI
void CalcTranslationalJacobianFloatingBaseToPointOnBodyInWorldFrame(
  Model &model,
  const VectorNd &Q,
  unsigned int body_id,
  const Vector3d &point_position,
  MatrixNd &G,
  bool update_kinematics
  ) {
  CalcJacobianFloatingBaseToPointOnBodyInWorldFrame(model, Q, body_id, point_position, G, romo::internal::SpatialAlgebra::RowTranslation, update_kinematics);
}

ANY_RBDL_DLLAPI
void CalcRotationalJacobianFloatingBaseToPointOnBodyInWorldFrame(
    Model &model,
    const Math::VectorNd &Q,
    unsigned int body_id,
    const Math::Vector3d &point_position,
    Math::MatrixNd &G,
    bool update_kinematics) {
  CalcJacobianFloatingBaseToPointOnBodyInWorldFrame(model, Q, body_id, point_position, G, romo::internal::SpatialAlgebra::RowRotation, update_kinematics);
}

ANY_RBDL_DLLAPI
void CalcRotationJacobianInWorldFrame (
    Model &model,
    const VectorNd &Q,
    unsigned int body_id,
    const Vector3d &point_position,
    MatrixNd &G,
    bool update_kinematics
  ) {
  LOG << "-------- " << __func__ << " --------" << std::endl;

  // update the Kinematics if necessary
  if (update_kinematics) {
    UpdateKinematicsCustom (model, &Q, NULL, NULL);
  }

  SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(), CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false));

  assert (G.rows() == 3 && G.cols() == model.qdot_size );

  unsigned int reference_body_id = body_id;

  if (model.IsFixedBodyId(body_id)) {
    unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    reference_body_id = model.mFixedBodies[fbody_id]->mMovableParent;
  }

  unsigned int j = reference_body_id;

  // set of already processed mimic joints
  std::unordered_set<unsigned int> processed_mimics;

  // e[j] is set to 1 if joint j contributes to the jacobian that we are
  // computing. For all other joints the column will be zero.

  while (j != 0) {
    unsigned int q_index = model.mJoints[j].q_index;

    // multiplier should be 1.0 when no mimic
    if (model.mJoints[j].mDoFCount == 3) {
      const Eigen::Matrix3d j33 = model.mJoints[j].mMimicMult * ((point_trans * model.X_base[j].inverse()).toMatrix() * model.multdof3_S[j]).block<3,3>(romo::internal::SpatialAlgebra::RowRotation,0);
      RigidBodyDynamics::getMimicMatrix<3,3>(G.block<3,3>(0, q_index), j33, model, q_index, processed_mimics);
    } else {
      const Eigen::Vector3d j31 = model.mJoints[j].mMimicMult * point_trans.apply(model.X_base[j].inverse().apply(model.S[j])).block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0);
      RigidBodyDynamics::getMimicMatrix<3,1>(G.block<3,1>(0, q_index), j31, model, q_index, processed_mimics);
    }

    j = model.lambda[j];
  }
}


ANY_RBDL_DLLAPI
bool CalcTranslationalHessianWorldToPointInWorldFrameForState(Model& model, const VectorNd &Q,
                                                              unsigned int q_j,
                                                              unsigned int body_id,
                                                              const Vector3d &point_position,
                                                              Eigen::MatrixXd& H,
                                                              bool update_kinematics)
{
  // Calculate spatial Jacobian for point on body
  Eigen::MatrixXd spatialJacobian = Eigen::MatrixXd::Zero(6, model.dof_count);
  RigidBodyDynamics::CalcSpatialJacobianWorldToPointInWorldFrame(model, Q, body_id, point_position,
                                                                 spatialJacobian,
                                                                 update_kinematics);

  return RigidBodyDynamics::CalcTranslationalHessianWorldToPointInWorldFrameForState(model, Q, q_j,
                                                                              body_id,
                                                                              point_position,
                                                                              spatialJacobian, H,
                                                                              update_kinematics);
}


ANY_RBDL_DLLAPI
bool CalcTranslationalHessianWorldToPointInWorldFrameForState(Model& model,
                               const VectorNd &Q,
                               unsigned int q_j,
                               unsigned int body_id,
                               const Vector3d &point_position,
                               const Eigen::MatrixXd& G,
                               Eigen::MatrixXd& H,
                               bool update_kinematics) {
  assert (H.rows() == 3 && H.cols() == model.qdot_size );
  assert (G.rows() == 6 && G.cols() == model.qdot_size );

  unsigned int reference_body_id = body_id;
  if (model.IsFixedBodyId(body_id)) {
   unsigned int fbody_id = body_id - model.fixed_body_discriminator;
   reference_body_id = model.mFixedBodies[fbody_id]->mMovableParent;
  }
  unsigned int i = reference_body_id;

  //-- Check if q_j is ancestor of body
  bool isAncestor = false;
  while (i != 0) {
    if (q_j >= model.mJoints[i].q_index && q_j < model.mJoints[i].q_index+model.mJoints[i].mDoFCount) {
      // found it
      isAncestor = true;
      break;
    }

    i = model.lambda[i];
  }
  if (!isAncestor) {
    return false;
  }
  //--

  // set of already processed mimic joints
  std::unordered_set<unsigned int> processed_mimics;

  i = reference_body_id;
  while (i != 0) {

    if ((model.mJoints[i].mJointType ==  JointTypeRevoluteX)
        || (model.mJoints[i].mJointType == JointTypeRevoluteY)
        || (model.mJoints[i].mJointType == JointTypeRevoluteZ)
        || (model.mJoints[i].mJointType == JointTypePrismatic)
        || (model.mJoints[i].mJointType == JointTypeRevolute))
    {
      unsigned int q_i = model.mJoints[i].q_index;
      if (q_i >= q_j) {
        // Jr_j x Jt_i
        if(model.mimic_qs.size() && q_i == q_j && model.mimic_qs.find(q_i) != model.mimic_qs.end()) {
            SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(), CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false));
            const Eigen::Matrix<double,6,1> j_sp = point_trans.apply(model.X_base[i].inverse().apply(model.S[i]));
            const Eigen::Matrix<double,3,1> h = model.mJoints[i].mMimicMult * j_sp.block<3,1>(0,0).cross(j_sp.block<3,1>(3,0));
            RigidBodyDynamics::getMimicMatrix<3,1>(H.block<3,1>(0, q_i), h, model, q_i, processed_mimics);
        }
        else {
            H.col(q_i) = G.block<3,1>(0, q_j).cross(G.block<3,1>(3, q_i));
        }
      }
      else {
        // Jr_i x Jt_j
        H.col(q_i) =
          G.block<3,1>(romo::internal::SpatialAlgebra::RowRotation, q_i).cross(
            G.block<3,1>(romo::internal::SpatialAlgebra::RowTranslation, q_j));
      }
    }
    else if (model.mJoints[i].mJointType == JointTypeTranslationXYZ
        || (model.mJoints[i].mJointType == JointTypeEulerZYX)) {
      for (unsigned int q_i = model.mJoints[i].q_index; q_i < model.mJoints[i].q_index+model.mJoints[i].mDoFCount; q_i++) {
        if (q_i >= q_j) {
          // Jr_j x Jt_i
          if(model.mimic_qs.size() && q_i == q_j && model.mimic_qs.find(q_i) != model.mimic_qs.end()) {
            SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(), CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false));
            const Eigen::Matrix<double,6,1> j_sp = point_trans.apply(model.X_base[i].inverse().apply(model.S[i]));
            const Eigen::Matrix<double,3,1> h = model.mJoints[i].mMimicMult * j_sp.block<3,1>(0,0).cross(j_sp.block<3,1>(3,0));
            RigidBodyDynamics::getMimicMatrix<3,1>(H.block<3,1>(0, q_i), h, model, q_i, processed_mimics);
          }
          else {
            H.col(q_i) =
              G.block<3,1>(romo::internal::SpatialAlgebra::RowRotation, q_j).cross(
                G.block<3,1>(romo::internal::SpatialAlgebra::RowTranslation, q_i));
          }
        }
        else {
          // Jr_i x Jt_j
          H.col(q_i) =
            G.block<3,1>(romo::internal::SpatialAlgebra::RowRotation, q_i).cross(
              G.block<3,1>(romo::internal::SpatialAlgebra::RowTranslation, q_j));
        }
      }
    }
    else if (model.mJoints[i].mJointType == JointTypeSpherical) {
      for (unsigned int q_i = model.mJoints[i].q_index; q_i < model.mJoints[i].q_index+model.mJoints[i].mDoFCount; q_i++) {
        if (q_i >= q_j || (q_j >= model.mJoints[i].q_index && q_j < model.mJoints[i].q_index+model.mJoints[i].mDoFCount))  {
          // Jr_j x Jt_i
          if(model.mimic_qs.size() && q_i == q_j && model.mimic_qs.find(q_i) != model.mimic_qs.end()) {
            SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(), CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false));
            const Eigen::Matrix<double,6,1> j_sp = point_trans.apply(model.X_base[i].inverse().apply(model.S[i]));
            const Eigen::Matrix<double,3,1> h = model.mJoints[i].mMimicMult * j_sp.block<3,1>(0,0).cross(j_sp.block<3,1>(3,0));
            RigidBodyDynamics::getMimicMatrix<3,1>(H.block<3,1>(0, q_i), h, model, q_i, processed_mimics);
          }
          else {
            H.col(q_i) =
              G.block<3,1>(romo::internal::SpatialAlgebra::RowRotation, q_j).cross(
                G.block<3,1>(romo::internal::SpatialAlgebra::RowTranslation, q_i));
          }
        }
        else {
          // Jr_i x Jt_j
          H.col(q_i) =
            G.block<3,1>(romo::internal::SpatialAlgebra::RowRotation, q_i).cross(
              G.block<3,1>(romo::internal::SpatialAlgebra::RowTranslation, q_j));
        }
      }
    }
    else {
      std::string error = "Joint of type " + std::to_string(model.mJoints[i].mJointType) + " is not supported!";
      throw std::runtime_error(error);
    }
    // get parent body
    i = model.lambda[i];
  }


  return true;

}


ANY_RBDL_DLLAPI
bool CalcRotationalHessianWorldToPointInWorldFrameForState(Model& model,
                                                           const Math::VectorNd &Q,
                                                           unsigned int q_j,
                                                           unsigned int body_id,
                                                           const Math::Vector3d &point_position,
                                                           Eigen::MatrixXd& H,
                                                           bool update_kinematics) {
  // Calculate spatial Jacobian for point on body
  Eigen::MatrixXd spatialJacobian = Eigen::MatrixXd::Zero(6, model.dof_count);
  RigidBodyDynamics::CalcSpatialJacobianWorldToPointInWorldFrame(model, Q, body_id, point_position,
                                                                 spatialJacobian,
                                                                 update_kinematics);

  return RigidBodyDynamics::CalcRotationalHessianWorldToPointInWorldFrameForState(model, Q, q_j, body_id,
                                                                           point_position,
                                                                           spatialJacobian, H,
                                                                           update_kinematics);
}


ANY_RBDL_DLLAPI
bool CalcRotationalHessianWorldToPointInWorldFrameForState(Model& model,
                                                           const Math::VectorNd &Q,
                                                           unsigned int q_j,
                                                           unsigned int body_id,
                                                           const Math::Vector3d &point_position,
                                                           const Eigen::MatrixXd& G,
                                                           Eigen::MatrixXd& H,
                                                           bool update_kinematics) {
  assert (H.rows() == 3 && H.cols() == model.qdot_size );
  assert (G.rows() == 6 && G.cols() == model.qdot_size );

  unsigned int reference_body_id = body_id;
  if (model.IsFixedBodyId(body_id)) {
   unsigned int fbody_id = body_id - model.fixed_body_discriminator;
   reference_body_id = model.mFixedBodies[fbody_id]->mMovableParent;
  }
  unsigned int i = reference_body_id;

  //-- Check if q_j is ancestor of body
  bool isAncestor = false;
  while (i != 0) {
    if (q_j >= model.mJoints[i].q_index && q_j < model.mJoints[i].q_index+model.mJoints[i].mDoFCount) {
      // found it
      isAncestor = true;
      break;
    }

    i = model.lambda[i];
  }
  if (!isAncestor) {
    return false;
  }
  //--

  // set of already processed mimic joints
  std::unordered_set<unsigned int> processed_mimics;

  i = reference_body_id;
  while (i != 0) {

    if ((model.mJoints[i].mJointType ==  JointTypeRevoluteX)
        || (model.mJoints[i].mJointType == JointTypeRevoluteY)
        || (model.mJoints[i].mJointType == JointTypeRevoluteZ)
        || (model.mJoints[i].mJointType == JointTypePrismatic)
        || (model.mJoints[i].mJointType == JointTypeRevolute))
    {
      unsigned int q_i = model.mJoints[i].q_index;
      if (q_i >= q_j) {
        // Jr_j x Jr_i
        if(model.mimic_qs.size() && q_i == q_j && model.mimic_qs.find(q_i) != model.mimic_qs.end()) {
          SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(), CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false));
          const Eigen::Matrix<double,3,1> j_r = point_trans.apply(model.X_base[i].inverse().apply(model.S[i])).block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0);
          const Eigen::Matrix<double,3,1> h = model.mJoints[i].mMimicMult * j_r.block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0).cross(j_r.block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0));
          RigidBodyDynamics::getMimicMatrix<3,1>(H.block<3,1>(0, q_i), h, model, q_i, processed_mimics);
        }
        else {
          // Jr_j x Jr_i
          H.col(q_i) =
            G.block<3,1>(romo::internal::SpatialAlgebra::RowRotation, q_j).cross(
              G.block<3,1>(romo::internal::SpatialAlgebra::RowRotation, q_i));
        }
      }
    }
    else if (model.mJoints[i].mJointType == JointTypeTranslationXYZ
            || (model.mJoints[i].mJointType == JointTypeEulerZYX)) {
      for (unsigned int q_i = model.mJoints[i].q_index; q_i < model.mJoints[i].q_index+model.mJoints[i].mDoFCount; q_i++) {
        if (q_i >= q_j) {
          // Jr_j x Jr_i
          if(model.mimic_qs.size() && q_i == q_j && model.mimic_qs.find(q_i) != model.mimic_qs.end()) {
            SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(), CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false));
            const Eigen::Matrix<double,3,1> j_r = point_trans.apply(model.X_base[i].inverse().apply(model.S[i])).block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0);
            const Eigen::Matrix<double,3,1> h = model.mJoints[i].mMimicMult * j_r.block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0).cross(j_r.block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0));
            RigidBodyDynamics::getMimicMatrix<3,1>(H.block<3,1>(0, q_i), h, model, q_i, processed_mimics);
          }
          else {
            // Jr_j x Jr_i
            H.col(q_i) =
              G.block<3,1>(romo::internal::SpatialAlgebra::RowRotation, q_j).cross(
                G.block<3,1>(romo::internal::SpatialAlgebra::RowRotation, q_i));
          }
        }
      }
    }
    else if (model.mJoints[i].mJointType == JointTypeSpherical) {
      for (unsigned int q_i = model.mJoints[i].q_index; q_i < model.mJoints[i].q_index+model.mJoints[i].mDoFCount; q_i++) {
        if (q_i >= q_j || (q_j >= model.mJoints[i].q_index && q_j < model.mJoints[i].q_index+model.mJoints[i].mDoFCount))  {
          // Jr_j x Jr_i
          if(model.mimic_qs.size() && q_i == q_j && model.mimic_qs.find(q_i) != model.mimic_qs.end()) {
            SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(), CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false));
            const Eigen::Matrix<double,3,1> j_r = point_trans.apply(model.X_base[i].inverse().apply(model.S[i])).block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0);
            const Eigen::Matrix<double,3,1> h = model.mJoints[i].mMimicMult * j_r.block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0).cross(j_r.block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0));
            RigidBodyDynamics::getMimicMatrix<3,1>(H.block<3,1>(0, q_i), h, model, q_i, processed_mimics);
          }
          else {
            // Jr_j x Jr_i
            H.col(q_i) =
              G.block<3,1>(romo::internal::SpatialAlgebra::RowRotation, q_j).cross(
                G.block<3,1>(romo::internal::SpatialAlgebra::RowRotation, q_i));
          }
        }
      }
    }
    else {
      std::string error = "Joint of type " + std::to_string(model.mJoints[i].mJointType) + " is not supported!";
      throw std::runtime_error(error);
    }
    // get parent body
    i = model.lambda[i];
  }

  return true;
}


ANY_RBDL_DLLAPI
bool CalcHessianSpatialWorldToPointInWorldFrameForState(Model& model, const VectorNd &Q,
                                                        unsigned int q_j, unsigned int body_id,
                                                        const Vector3d &point_position,
                                                        Eigen::MatrixXd& H, bool update_kinematics)
{

  // Calculate spatial Jacobian for point on body
  Eigen::MatrixXd spatialJacobian = Eigen::MatrixXd::Zero(6, model.dof_count);
  RigidBodyDynamics::CalcSpatialJacobianWorldToPointInWorldFrame(model, Q, body_id, point_position,
                                                                 spatialJacobian,
                                                                 update_kinematics);

  return RigidBodyDynamics::CalcHessianSpatialWorldToPointInWorldFrameForState(model, Q, q_j, body_id,
                                                                        point_position,
                                                                        spatialJacobian, H,
                                                                        update_kinematics);
}


ANY_RBDL_DLLAPI
bool CalcHessianSpatialWorldToPointInWorldFrameForState(Model& model, const VectorNd &Q,
                                                        unsigned int q_j, unsigned int body_id,
                                                        const Vector3d &point_position,
                                                        const Eigen::MatrixXd& spatialJacobian,
                                                        Eigen::MatrixXd& H, bool update_kinematics)
{
  assert(H.rows() == 6 && H.cols() == model.qdot_size);

  unsigned int reference_body_id = body_id;
  if (model.IsFixedBodyId(body_id)) {
    unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    reference_body_id = model.mFixedBodies[fbody_id]->mMovableParent;
  }
  unsigned int i = reference_body_id;

  //-- Check if q_j is ancestor of body
  bool isAncestor = false;
  while (i != 0) {
    if (q_j >= model.mJoints[i].q_index && q_j < model.mJoints[i].q_index+model.mJoints[i].mDoFCount) {
      // found it
      isAncestor = true;
      break;
    }

    i = model.lambda[i];
  }
  if (!isAncestor) {
    return false;
  }
  //--

  // set of already processed mimic joints
  std::unordered_set<unsigned int> processed_mimics;

  i = reference_body_id;
  while (i != 0) {

    if ((model.mJoints[i].mJointType == JointTypeRevoluteX)
        || (model.mJoints[i].mJointType == JointTypeRevoluteY)
        || (model.mJoints[i].mJointType == JointTypeRevoluteZ)
        || (model.mJoints[i].mJointType == JointTypePrismatic)
        || (model.mJoints[i].mJointType == JointTypeRevolute)) {
      unsigned int q_i = model.mJoints[i].q_index;
      if (q_i >= q_j) {
        if(model.mimic_qs.size() && q_i == q_j && model.mimic_qs.find(q_i) != model.mimic_qs.end()) {
          SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(), CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false));
          const Eigen::Matrix<double,6,1> j_sp = point_trans.apply(model.X_base[i].inverse().apply(model.S[i]));
          Eigen::Matrix<double,6,1> h;
          // Translational component: Jr_j x Jt_i
          h.block<3,1>(romo::internal::SpatialAlgebra::RowTranslation,0) = model.mJoints[i].mMimicMult * j_sp.block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0).cross(j_sp.block<3,1>(romo::internal::SpatialAlgebra::RowTranslation,0));
          // Rotational component: Jr_j x Jr_i
          h.block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0) = model.mJoints[i].mMimicMult * j_sp.block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0).cross(j_sp.block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0));
          RigidBodyDynamics::getMimicMatrix<6,1>(H.block<6,1>(0, q_i), h, model, q_i, processed_mimics);
        }
        else {
          // Translational component: Jr_j x Jt_i
          H.block<3, 1>(romo::internal::SpatialAlgebra::RowTranslation, q_i) =
            spatialJacobian.block<3, 1>(romo::internal::SpatialAlgebra::RowRotation, q_j).cross(
              spatialJacobian.block<3, 1>(romo::internal::SpatialAlgebra::RowTranslation, q_i));
          // Rotational component: Jr_j x Jr_i
          H.block<3, 1>(romo::internal::SpatialAlgebra::RowRotation, q_i) =
            spatialJacobian.block<3, 1>(romo::internal::SpatialAlgebra::RowRotation, q_j).cross(
              spatialJacobian.block<3, 1>(romo::internal::SpatialAlgebra::RowRotation, q_i));
        }
      } else {
        // Translational component: Jr_i x Jt_j
        H.block<3, 1>(romo::internal::SpatialAlgebra::RowTranslation, q_i) =
          spatialJacobian.block<3, 1>(romo::internal::SpatialAlgebra::RowRotation, q_i).cross(
            spatialJacobian.block<3, 1>(romo::internal::SpatialAlgebra::RowTranslation, q_j));
        // Rotational component is null
      }
    }

    else if (model.mJoints[i].mJointType == JointTypeTranslationXYZ
          || (model.mJoints[i].mJointType == JointTypeEulerZYX)) {
      for (unsigned int q_i = model.mJoints[i].q_index;
          q_i < model.mJoints[i].q_index + model.mJoints[i].mDoFCount; q_i++) {
        if (q_i >= q_j) {
          if(model.mimic_qs.size() && q_i == q_j && model.mimic_qs.find(q_i) != model.mimic_qs.end()) {
            SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(), CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false));
            const Eigen::Matrix<double,6,1> j_sp = point_trans.apply(model.X_base[i].inverse().apply(model.S[i]));
            Eigen::Matrix<double,6,1> h;
            // Translational component: Jr_j x Jt_i
            h.block<3,1>(romo::internal::SpatialAlgebra::RowTranslation,0) = model.mJoints[i].mMimicMult * j_sp.block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0).cross(j_sp.block<3,1>(romo::internal::SpatialAlgebra::RowTranslation,0));
            // Rotational component: Jr_j x Jr_i
            h.block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0) = model.mJoints[i].mMimicMult * j_sp.block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0).cross(j_sp.block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0));
            RigidBodyDynamics::getMimicMatrix<6,1>(H.block<6,1>(0, q_i), h, model, q_i, processed_mimics);
          }
          else {
            // Translational component: Jr_j x Jt_i
            H.block<3, 1>(romo::internal::SpatialAlgebra::RowTranslation, q_i) =
              spatialJacobian.block<3, 1>(romo::internal::SpatialAlgebra::RowRotation, q_j).cross(
                spatialJacobian.block<3, 1>(romo::internal::SpatialAlgebra::RowTranslation, q_i));
            // Rotational component: Jr_j x Jr_i
            H.block<3, 1>(romo::internal::SpatialAlgebra::RowRotation, q_i) =
              spatialJacobian.block<3, 1>(romo::internal::SpatialAlgebra::RowRotation, q_j).cross(
                spatialJacobian.block<3, 1>(romo::internal::SpatialAlgebra::RowRotation, q_i));
          }
        } else {
          // Translational component: Jr_i x Jt_j
          H.block<3, 1>(romo::internal::SpatialAlgebra::RowTranslation, q_i) =
            spatialJacobian.block<3, 1>(romo::internal::SpatialAlgebra::RowRotation, q_i).cross(
              spatialJacobian.block<3, 1>(romo::internal::SpatialAlgebra::RowTranslation, q_j));
          // Rotational component is null
        }
      }
    }

    else if (model.mJoints[i].mJointType == JointTypeSpherical) {
      for (unsigned int q_i = model.mJoints[i].q_index;
          q_i < model.mJoints[i].q_index + model.mJoints[i].mDoFCount; q_i++) {
        if (q_i >= q_j
            || (q_j >= model.mJoints[i].q_index
                && q_j < model.mJoints[i].q_index + model.mJoints[i].mDoFCount)) {
          if(model.mimic_qs.size() && q_i == q_j && model.mimic_qs.find(q_i) != model.mimic_qs.end()) {
            SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(), CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false));
            const Eigen::Matrix<double,6,1> j_sp = point_trans.apply(model.X_base[i].inverse().apply(model.S[i]));
            Eigen::Matrix<double,6,1> h;
            // Translational component: Jr_j x Jt_i
            h.block<3,1>(romo::internal::SpatialAlgebra::RowTranslation,0) = model.mJoints[i].mMimicMult * j_sp.block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0).cross(j_sp.block<3,1>(romo::internal::SpatialAlgebra::RowTranslation,0));
            // Rotational component: Jr_j x Jr_i
            h.block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0) = model.mJoints[i].mMimicMult * j_sp.block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0).cross(j_sp.block<3,1>(romo::internal::SpatialAlgebra::RowRotation,0));
            RigidBodyDynamics::getMimicMatrix<6,1>(H.block<6,1>(0, q_i), h, model, q_i, processed_mimics);
          }
          else {
            // Translational component: Jr_j x Jt_i
            H.block<3, 1>(romo::internal::SpatialAlgebra::RowTranslation, q_i) =
              spatialJacobian.block<3, 1>(romo::internal::SpatialAlgebra::RowRotation, q_j).cross(
                spatialJacobian.block<3, 1>(romo::internal::SpatialAlgebra::RowTranslation, q_i));
            // Rotational component: Jr_j x Jr_i
            H.block<3, 1>(romo::internal::SpatialAlgebra::RowRotation, q_i) =
              spatialJacobian.block<3, 1>(romo::internal::SpatialAlgebra::RowRotation, q_j).cross(
                spatialJacobian.block<3, 1>(romo::internal::SpatialAlgebra::RowRotation, q_i));
          }
        } else {
          // Jr_i x Jt_j
          H.block<3, 1>(romo::internal::SpatialAlgebra::RowTranslation, q_i) =
            spatialJacobian.block<3, 1>(romo::internal::SpatialAlgebra::RowRotation, q_i).cross(
              spatialJacobian.block<3, 1>(romo::internal::SpatialAlgebra::RowTranslation, q_j));
          // Rotational component is null
        }
      }
    }

    else {
      std::string error =
          "[CalcHessian6dWorldToPointInWorldFrameForState] Joint of type "
              + std::to_string(model.mJoints[i].mJointType) + " is not supported!";
      throw std::runtime_error(error);
    }
    // get parent body
    i = model.lambda[i];
  }

  return true;
}


ANY_RBDL_DLLAPI
Math::Vector3d CalcPositionWorldToCoMInWorldFrame(Model& model,
                                    const VectorNd &Q,
                                    bool update_kinematics)
{
  Math::Vector3d position = Math::Vector3d::Zero();
  double totalMass = 0.0;
  for (unsigned int i=0; i<model.mBodies.size(); i++) {
    position += RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q, i, model.mBodies[i]->GetCenterOfMass(), update_kinematics)*model.mBodies[i]->GetMass();
    totalMass += model.mBodies[i]->GetMass();
  }
  position /= totalMass;
  return position;
}


ANY_RBDL_DLLAPI
void CalcTranslationalJacobianWorldToCoMInWorldFrame(Model& model,
                                          const VectorNd &Q,
                                          MatrixNd &G,
                                          bool update_kinematics)
{
  assert (G.rows() == 3 && G.cols() == model.qdot_size );

  MatrixNd bodyJacobian(3, model.qdot_size );

  double totalMass = 0.0;
  for (unsigned int i=0; i<model.mBodies.size(); i++) {
    totalMass += model.mBodies[i]->GetMass();
  }

  for (unsigned int i=0; i<model.mBodies.size(); i++) {
    bodyJacobian.setZero();
    RigidBodyDynamics::CalcPointJacobian(model, Q, i, model.mBodies[i]->GetCenterOfMass(), bodyJacobian, update_kinematics);
    G += bodyJacobian*(model.mBodies[i]->GetMass()/totalMass);
  }
}


ANY_RBDL_DLLAPI
bool CalcTranslationalHessianWorldToCoMInWorldFrameForState(
    Model& model, const VectorNd &Q,
    unsigned int q_j, const Eigen::MatrixXd& G,
    Eigen::MatrixXd& H, bool update_kinematics)
{
  assert (H.rows() == 3 && H.cols() == model.qdot_size );
  assert (G.rows() == 6 && G.cols() == model.qdot_size );

  // r = Sum_i r_i * m_i/mTotal
  // dr/dq = Sum_i dr_i/dq * m_i/mTotal
  // d^2r/dq^2 = Sum_i d^2r_i/dq^2 * m_i/mTotal
  MatrixNd bodyHessian(3, model.qdot_size);

  double totalMass = 0.0;
  for (unsigned int i=0; i<model.mBodies.size(); i++) {
    totalMass += model.mBodies[i]->GetMass();
  }

  for (unsigned int i=0; i<model.mBodies.size(); i++) {
    bodyHessian.setZero();
    if (!RigidBodyDynamics::CalcTranslationalHessianWorldToPointInWorldFrameForState(
        model, Q, q_j, i, model.mBodies[i]->GetCenterOfMass(), G, bodyHessian,
        update_kinematics)) {
      // hessian is zero
      continue;
    }
    H += bodyHessian*(model.mBodies[i]->GetMass()/totalMass);
  }
  return true;
}


ANY_RBDL_DLLAPI
bool CalcTranslationalHessianWorldToCoMInWorldFrameForState(
    Model& model, const VectorNd &Q,
    unsigned int q_j, Eigen::MatrixXd& H,
    bool update_kinematics)
{
  assert (H.rows() == 3 && H.cols() == model.qdot_size );
  // r = Sum_i r_i * m_i/mTotal
  // dr/dq = Sum_i dr_i/dq * m_i/mTotal
  // d^2r/dq^2 = Sum_i d^2r_i/dq^2 * m_i/mTotal
  MatrixNd bodyHessian(3, model.qdot_size);

  double totalMass = 0.0;
  for (unsigned int i=0; i<model.mBodies.size(); i++) {
    totalMass += model.mBodies[i]->GetMass();
  }

  for (unsigned int i=0; i<model.mBodies.size(); i++) {
    bodyHessian.setZero();
    if (!RigidBodyDynamics::CalcTranslationalHessianWorldToPointInWorldFrameForState(model, Q, q_j, i, model.mBodies[i]->GetCenterOfMass(), bodyHessian, update_kinematics)) {
      // hessian is zero
      continue;
    }
    H += bodyHessian*(model.mBodies[i]->GetMass()/totalMass);
  }
  return true;
}

ANY_RBDL_DLLAPI
Vector3d CalcBodyAngularVelocityInWorldFrame(
    Model &model,
    const VectorNd &Q,
    const VectorNd &QDot,
    unsigned int body_id,
    bool update_kinematics) {
  LOG << "-------- " << __func__ << " --------" << std::endl;
  assert (model.IsBodyId(body_id));
  assert (model.q_size == Q.size());
  assert (model.qdot_size == QDot.size());

  // Reset the velocity of the root body
  model.v[0].setZero();

  // update the Kinematics with zero acceleration
  if (update_kinematics) {
    UpdateKinematicsCustom (model, &Q, &QDot, NULL);
  }

  unsigned int reference_body_id = body_id;
  Vector3d reference_point = Vector3d::Zero();

  if (model.IsFixedBodyId(body_id)) {
    unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    reference_body_id = model.mFixedBodies[fbody_id]->mMovableParent;
    Vector3d base_coords = CalcBodyToBaseCoordinates (model, Q, body_id, Vector3d::Zero(), false);
    reference_point = CalcBaseToBodyCoordinates (model, Q, reference_body_id, base_coords, false);
  }

  SpatialVector point_spatial_velocity = SpatialTransform(
      CalcBodyWorldOrientation(model, Q, reference_body_id, false).transpose(),
      reference_point).apply(model.v[reference_body_id]);

  return Vector3d(point_spatial_velocity[0],
                  point_spatial_velocity[1],
                  point_spatial_velocity[2]);
}

} // namespace
