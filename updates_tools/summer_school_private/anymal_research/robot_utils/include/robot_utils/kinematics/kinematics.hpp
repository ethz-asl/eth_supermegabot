/*
 * kinematics.hpp
 *
 *  Created on: February 01, 2018
 *      Author: Yvain de Viragh
 */
#pragma once

#include <kindr/Core>
#include <Eigen/Core>
#include <robot_utils/math/math.hpp>

namespace robot_utils {

/*! Computes the final pose of a frame moving at constant frame-fixed twist.
 *
 * Computes the final pose of a frame moving at constant frame-fixed twist
 * given its start pose. The pose is exact in the sense that the analytic
 * solution is used for its computation.
 *
 * Note: A frame moving at constant frame-fixed twist follows a helical path. By contrast,
 * if moving at constant reference twist, it would travel along a straight line.
 *
 * @param poseStartToReference               pose start frame to reference frame
 * @param twistReferenceToStartInStartFrame  twist (constant) of start frame w.r.t. reference frame in start frame
 * @param timeHorizonInSeconds               end time minus start time
 * @return                                   pose end frame to reference frame
 */
kindr::HomTransformQuatD getPoseEndToReferenceFromTwistReferenceToStartInStartFrame(const kindr::HomTransformQuatD& poseStartToReference,
                                                                                    const kindr::TwistLocalD& twistReferenceToStartInStartFrame,
                                                                                    double timeHorizonInSeconds);

/*! Computes the approximated final pose of a frame moving at constant frame-fixed twist.
 *
 * Computes the approximated final pose of a frame moving at constant frame-fixed
 * twist given its start pose. The solution is based on an approximation valid for
 * small angles of rotation, i.e. given the product of the time horizon and the
 * magnitude of the angular velocity is small. Namely, we use the following approximation:
 *
 * finalPose = initialPose + initialOrientation * eulerVectorToRotationMatrix(t*angularVelocityInMovingFrame) * linearVelocityInMovingFrame * t;
 *
 * Note: A frame moving at constant frame-fixed twist follows a helical path. By contrast,
 * if moving at constant reference twist, it would travel along a straight line. However,
 * for large rotations this function predicts a spiral path.
 *
 * Note: The function getPoseEndToReferenceFromTwistReferenceToStartInStartFrame should be preferred,
 * as it has comparable computational complexity, but provides the exact/analytic solution.
 *
 * @param poseStartToReference               pose start frame to reference frame
 * @param twistReferenceToStartInStartFrame  twist (constant) of start frame w.r.t. reference frame in start frame
 * @param timeHorizonInSeconds               end time minus start time
 * @return                                   pose end frame to reference frame
 */
kindr::HomTransformQuatD getApproximatedPoseEndToReferenceFromTwistReferenceToStartInStartFrame(const kindr::HomTransformQuatD& poseStartToReference,
                                                                                                const kindr::TwistLocalD& TwistReferenceToStartInStartFrame,
                                                                                                double timeHorizonInSeconds);

} /* namespace robot_utils */
