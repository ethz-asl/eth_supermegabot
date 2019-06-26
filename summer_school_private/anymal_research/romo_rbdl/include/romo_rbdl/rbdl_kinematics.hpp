/*!
* @file    rbdl_kinematics.hpp
* @author  Christian Gehring, Dario Bellicoso
* @date    Sep, 2015
*/

#pragma once
#include "any_rbdl/rbdl.h"
#include "any_rbdl/rbdl_mathutils.h"
#include "any_rbdl/Logging.h"

#include "any_rbdl/Model.h"
#include "any_rbdl/Kinematics.h"

namespace RigidBodyDynamics {

/** \brief Computes the spatial Jacobian for a point on a body.
 *
 * If a position of a point is computed by a function \f$g(q(t))\f$ for which its
 * time derivative is \f$\frac{d}{dt} g(q(t)) = G_t(q)\dot{q}\f$ then this
 * function computes the translational Jacobian matrix \f$G_t(q)\f$.
 *
 * This method returns \f$G(q) = \begin{bmatrix} G_t(q) \\ G_r(q)\end{bmatrix}\f$,
 * where \f$G_r(q)\f$ is the rotational Jacobian
 *
 * The spatial velocity of a point on a body at the origin of the base coordinate
 * system can be expressed as \f${}^0 \hat{v}_i = G(q) * \dot{q}\f$. The
 * matrix \f$G(q)\f$ is called the spatial body jacobian of the body and
 * can be computed using this function.
 *
 * \param model   rigid body model
 * \param Q       state vector of the internal joints
 * \param body_id the id of the body
 * \param point_position the position of the point in body-local data
 * \param G       a matrix of size 6 x \#qdot_size where the result will be stored in
 * \param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 * The result will be returned via the G argument.
 *
 * \note This function only evaluates the entries of G that are non-zero. One
 * Before calling this function one has to ensure that all other values
 * have been set to zero, e.g. by calling G.setZero().
 */
ANY_RBDL_DLLAPI
void CalcSpatialJacobianWorldToPointInWorldFrame (
    Model &model,
    const Math::VectorNd &Q,
    unsigned int body_id,
    const Math::Vector3d &point_position,
    Math::MatrixNd &G,
    bool update_kinematics = true);

ANY_RBDL_DLLAPI
void CalcTranslationalJacobianWorldToPointInWorldFrame (
    Model &model,
    const Math::VectorNd &Q,
    unsigned int body_id,
    const Math::Vector3d &point_position,
    Math::MatrixNd &G,
    bool update_kinematics = true);

ANY_RBDL_DLLAPI
void CalcTranslationalJacobianFloatingBaseToPointOnBodyInWorldFrame(
    Model &model,
    const Math::VectorNd &Q,
    unsigned int body_id,
    const Math::Vector3d &point_position,
    Math::MatrixNd &G,
    bool update_kinematics = true);

ANY_RBDL_DLLAPI
void CalcRotationalJacobianFloatingBaseToPointOnBodyInWorldFrame(
    Model &model,
    const Math::VectorNd &Q,
    unsigned int body_id,
    const Math::Vector3d &point_position,
    Math::MatrixNd &G,
    bool update_kinematics = true);

ANY_RBDL_DLLAPI
void CalcRotationJacobianInWorldFrame (
    Model &model,
    const Math::VectorNd &Q,
    unsigned int body_id,
    const Math::Vector3d &point_position,
    Math::MatrixNd &G,
    bool update_kinematics = true);


/** \brief Computes the part of the Hessian tensor of a point on a body with respect to the given generalized coordinate.
 *
 * The result will be returned via the H argument and represents the matrix that tells you
 * how the Jacobian dr/dq changes with respect to q_j.
 * \returns false only if it contains only zero elements. If it returns true it can still contain only zero elements.
 *
 * \param model   rigid body model
 * \param Q       state vector of the internal joints
 * \param q_j     index of the generalized
 * \param body_id the id of the body
 * \param point_position the position of the point in body-local data
 * \param G       a matrix of size 6 x \#qdot_size where the result will be stored in
 * \param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 *
 * \note This function only evaluates the entries of G that are non-zero. One
 * Before calling this function one has to ensure that all other values
 * have been set to zero, e.g. by calling G.setZero().
 */
ANY_RBDL_DLLAPI
bool CalcTranslationalHessianWorldToPointInWorldFrameForState(Model& model,
                                                 const Math::VectorNd &Q,
                                                 unsigned int q_j,
                                                 unsigned int body_id,
                                                 const Math::Vector3d &point_position,
                                                 Eigen::MatrixXd& H,
                                                 bool update_kinematics=true);

ANY_RBDL_DLLAPI
bool CalcTranslationalHessianWorldToPointInWorldFrameForState(Model& model,
                               const Math::VectorNd &Q,
                               unsigned int q_j,
                               unsigned int body_id,
                               const Math::Vector3d &point_position,
                               const Eigen::MatrixXd& G,
                               Eigen::MatrixXd& H,
                               bool update_kinematics=true);



ANY_RBDL_DLLAPI
bool CalcRotationalHessianWorldToPointInWorldFrameForState(Model& model,
                                                           const Math::VectorNd &Q,
                                                           unsigned int q_j,
                                                           unsigned int body_id,
                                                           const Math::Vector3d &point_position,
                                                           const Eigen::MatrixXd& G,
                                                           Eigen::MatrixXd& H,
                                                           bool update_kinematics=true);

ANY_RBDL_DLLAPI
bool CalcRotationalHessianWorldToPointInWorldFrameForState(Model& model,
                                                           const Math::VectorNd &Q,
                                                           unsigned int q_j,
                                                           unsigned int body_id,
                                                           const Math::Vector3d &point_position,
                                                           Eigen::MatrixXd& H,
                                                           bool update_kinematics=true);


ANY_RBDL_DLLAPI
bool CalcHessianSpatialWorldToPointInWorldFrameForState(Model& model,
                                                        const Math::VectorNd &Q,
                                                        unsigned int q_j,
                                                        unsigned int body_id,
                                                        const Math::Vector3d &point_position,
                                                        Eigen::MatrixXd& H,
                                                        bool update_kinematics=true);

ANY_RBDL_DLLAPI
bool CalcHessianSpatialWorldToPointInWorldFrameForState(Model& model,
                                                     const Math::VectorNd &Q,
                                                     unsigned int q_j,
                                                     unsigned int body_id,
                                                     const Math::Vector3d &point_position,
                                                     const Eigen::MatrixXd& spatialJacobian,
                                                     Eigen::MatrixXd& H,
                                                     bool update_kinematics=true);

ANY_RBDL_DLLAPI
Math::Vector3d CalcPositionWorldToCoMInWorldFrame(Model& model,
                                    const Math::VectorNd &Q,
                                    bool update_kinematics=true);

ANY_RBDL_DLLAPI
Math::Vector3d CalcLinearVelocityCoMInWorldFrame(Model& model,
                                                 const Math::VectorNd &Q,
                                                 const Math::VectorNd &Qdot,
                                                 bool update_kinematics = true);

// assumes G is zero
ANY_RBDL_DLLAPI
void CalcTranslationalJacobianWorldToCoMInWorldFrame(Model& model,
                                          const Math::VectorNd &Q,
                                          Math::MatrixNd &G,
                                          bool update_kinematics=true);

ANY_RBDL_DLLAPI
bool CalcTranslationalHessianWorldToCoMInWorldFrameForState(Model& model,
                                                 const Math::VectorNd &Q,
                                                 unsigned int q_j,
                                                 Eigen::MatrixXd& H,
                                                 bool update_kinematics=true);

ANY_RBDL_DLLAPI
bool CalcTranslationalHessianWorldToCoMInWorldFrameForState(Model& model,
                                                 const Math::VectorNd &Q,
                                                 unsigned int q_j,
                                                 const Eigen::MatrixXd& G,
                                                 Eigen::MatrixXd& H,
                                                 bool update_kinematics = true);

ANY_RBDL_DLLAPI
Math::Vector3d CalcBodyAngularVelocityInWorldFrame(
    Model &model,
    const Math::VectorNd &Q,
    const Math::VectorNd &QDot,
    unsigned int body_id,
    bool update_kinematics);

} // namespace RigidBodyDynamics
