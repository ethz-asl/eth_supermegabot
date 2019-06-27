/*
 * phys_typedefs.hpp
 *
 *  Created on: Nov 10, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

#include "kindr/Core"

namespace romo {

typedef kindr::HomTransformQuatD      Pose;
typedef kindr::TwistLocalD            Twist;

typedef kindr::RotationQuaternionPD      RotationQuaternion;
typedef kindr::RotationQuaternionDiffPD  RotationQuaternionDiff;
typedef kindr::AngleAxisPD               AngleAxis;
typedef kindr::RotationMatrixPD          RotationMatrix;
typedef kindr::EulerAnglesZyxPD          EulerAnglesZyx;
typedef kindr::RotationVectorPD          RotationVector;

typedef kindr::EulerAnglesXyzPD          EulerAnglesXyz;
typedef kindr::EulerAnglesXyzDiffPD      EulerAnglesXyzDiff;

typedef kindr::Position3D               Position;
typedef kindr::Velocity3D               LinearVelocity;

typedef kindr::LocalAngularVelocityPD    LocalAngularVelocity;
typedef kindr::EulerAnglesZyxDiffPD      EulerAnglesZyxDiff;

typedef kindr::Acceleration3D           LinearAcceleration;
typedef kindr::AngularAcceleration3D    AngularAcceleration;

typedef kindr::Force3D                  Force;
typedef kindr::Torque3D                 Torque;

typedef kindr::VectorTypeless3D         Vector;

} // namespace romo
