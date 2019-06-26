/*
 * rbdl_utils.h
 *
 *  Created on: Oct 1, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

#include <any_rbdl/rbdl.h>
#include <vector>
#include <string>
#include <urdf_parser/urdf_parser.h>

namespace romo {

bool URDFReadFromFile(urdf::ModelInterfaceSharedPtr& urdf_model,
                      const char* filename, RigidBodyDynamics::Model* model,
                      const std::vector<std::string>& movable_joints_in_order,
                      bool verbose = false,
                      bool useQuaternion = true,
                      double absoluteGravityAcceleration = 9.81);
bool URDFReadFromFile(const char* filename, RigidBodyDynamics::Model* model,
                      const std::vector<std::string>& movable_joints_in_order,
                      bool verbose = false,
                      bool useQuaternion = true,
                      double absoluteGravityAcceleration = 9.81);

bool URDFReadFromString(urdf::ModelInterfaceSharedPtr& urdf_model,
                        const char* model_xml_string,
                        RigidBodyDynamics::Model* model,
                        const std::vector<std::string>& movable_joints_in_order,
                        bool verbose = false,
                        bool useQuaternion = true,
                        double absoluteGravityAcceleration = 9.81);
bool URDFReadFromString(const char* model_xml_string,
                        RigidBodyDynamics::Model* model,
                        const std::vector<std::string>& movable_joints_in_order,
                        bool verbose = false,
                        bool useQuaternion = true,
                        double absoluteGravityAcceleration = 9.81);

/* Given generalized velocities u = [v^T w^T qj^T ...]^T, special care should be taken when mapping these to the operational space
 */
//! Convert the mapping of angular velocities from the active convention to the passive one.
inline static bool convertGeneralizedVelocitiesActiveToPassive(Eigen::MatrixXd& passive, const Eigen::MatrixXd& active) {
  passive.block(0, 3, active.rows(), 3) *= -1.0;
  return true;
}

//! Convert the mapping of angular velocities from the passive convention to the active one.
inline static bool convertGeneralizedVelocitiesPassiveToActive(Eigen::MatrixXd& active, const Eigen::MatrixXd& passive) {
  active.block(0, 3, passive.rows(), 3) *= -1.0;
  return true;
}

} // namespace
