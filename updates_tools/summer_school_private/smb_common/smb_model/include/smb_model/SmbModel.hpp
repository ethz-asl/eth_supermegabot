/*
 * SmbModel.hpp
 *
 *  Created on: Aug 1, 2018
 *      Author: Koen Kraemer
 */

#pragma once

// eigen
#include <Eigen/Core>

// stl
#include <memory>

// super mega bot description
#include "smb_description/SmbDescription.hpp"

// super mega bot model
#include "smb_model/common/containers.hpp"
#include "smb_model/SmbState.hpp"

// romo
#include "romo_rbdl/RobotModelRbdl.hpp"

namespace smb_model {

class SmbModel : public romo_rbdl::RobotModelRbdl<ConcreteDescription, SmbState> {
 private:
  using Base = romo_rbdl::RobotModelRbdl< ConcreteDescription, SmbState>;

 public:
  using SmbDescription = RD;

// TODO do I need limb/branch/bodynode?
  using typename Base::BodyEnum;
  using typename Base::BodyNodeEnum;
  using typename Base::CoordinateFrameEnum;
  using typename Base::BranchEnum;
  using typename Base::LimbEnum;
  using typename Base::RobotState;

 public:
  explicit SmbModel(double dt = 0.0025); //TODO check default control frequency
  virtual ~SmbModel() override = default;

  /*! Initialize the super mega bot model by parsing a URDF file located in pathToUrdfFile
   * @returns true if the initialization is successful
   */
  bool initModelFromUrdfFile(const std::string& pathToUrdfFile, bool useQuaternion = true, bool verbose = false);

  /*! Initialize the super mega bot model by parsing the string urdfDescription
   * containing the URDF of the model
   * @returns true if the initialization is successful
   */
  bool initModelFromUrdfString(const std::string& urdfDescription, bool useQuaternion = true, bool verbose = false);

  void setIsRealRobot(const bool isRealRobot);
  bool getIsRealRobot() const;

  /*******************
   * State accessors *
   *******************/
  virtual void setState(const SmbState& state, bool updatePosition = true, bool updateVelocity = false, bool updateAcceleration = false);
  /*******************/

// TODO leave in? necessary?
  // const JointTorques& getJointTorques() const;
  // void setJointTorques(const JointTorques& jointTorques);

  const Eigen::MatrixXd& getActuatorSelectionMatrix() const;

  double getTimeStep() const;

  void addVariablesToLog(bool update, const std::string& nsPostfix = std::string{"/model/"});
  const std::string& getUrdfDescription() const;

  bool useQuaternion() const  {
    return useQuaternion_;
  }
protected:

  /*! Initialize the local variables after successfully loading the model from the URDF
   * @returns true if successful
   */
  bool initializeModel(bool useQuaternion);

 protected:
  bool isRealRobot_;
  double timeStep_;

  Eigen::MatrixXd actuatorSelectionMatrix_;

  bool useQuaternion_;
};


} /* namespace smb_model */
