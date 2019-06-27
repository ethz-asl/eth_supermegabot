/*!
* @file    TestQuadrupedModel.hpp
* @author  Christian Gehring, Dario Bellicoso
* @date    Sep, 2015
*/

#pragma once

// gtest
#include <gtest/gtest.h>

// boost
#include <boost/filesystem.hpp>

// eigen
#include <Eigen/Core>

// alma model
#include "smb_model/SmbModel.hpp"
#include "smb_model/common/rbdl_utils.hpp"

// romo tests
#include "romo_test/TestRobotModel.hpp"

class TestSmbModel : virtual public romo_test::TestRobotModel<smb_model::SmbModel> {
 private:
  using BaseTest = romo_test::TestRobotModel<smb_model::SmbModel>;

 public:
  TestSmbModel()
      : BaseTest()
  {
    model_.reset(new smb_model::SmbModel());
    state_.reset(new smb_model::SmbState());
  }

  virtual ~TestSmbModel() {

  }

  void init() {
    initModel("smb");
  }

  void initModel(const std::string& urdfName, bool useQuat = true, bool verbose = false) {
    useQuaternion = useQuat;

    // Reset the state.
    state_->setZero();

    //-- Load model for testing
    boost::filesystem::path filePath(__FILE__);
    std::string path = filePath.parent_path().parent_path().generic_string() + std::string{"/resources/" + urdfName + ".urdf"};
    ASSERT_TRUE(getModelPtr()->initModelFromUrdfFile(path, useQuaternion, verbose));

    // Set the state to the model.
    model_->setState(*state_);
  }

  void getRandomGeneralizedPositionsRbdl(Eigen::VectorXd& q) {
    smb_model::SmbState state;
    state.setRandom();
    smb_model::setRbdlQFromState(q, state);
  }

  void setRbdlQFromState(Eigen::VectorXd& rbdlQ, const RobotStateType& state) {
    smb_model::setRbdlQFromState(rbdlQ, state);
  }

  void setStateFromRbdlQ(RobotStateType& state, const Eigen::VectorXd& rbdlQ) {
    smb_model::setStateFromRbdlQ(state, rbdlQ);
  }

 public:
  bool useQuaternion = true;
};

