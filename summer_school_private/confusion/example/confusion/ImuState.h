/*
 * Copyright 2018 Timothy Sandy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef INCLUDE_CONFUSION_EXAMPLES_IMUSTATE_H_
#define INCLUDE_CONFUSION_EXAMPLES_IMUSTATE_H_

//#define OPT_GRAVITY

#include <confusion/models/ImuChain.h>
#include <confusion/models/TagMeas.h>
#include <confusion/models/PoseMeas.h>
#include <confusion/utilities/Pose.h>
#include <confusion/State.h>
#include <Eigen/Core>
#include <deque>
#include <vector>
#include <memory>

class ImuState : public confusion::State {
 public:
  ImuState(double t = 0.0) : confusion::State(t, NUM_PROCESS_SENSORS, NUM_UPDATE_SENSORS) {
#ifdef OPT_GRAVITY
    processChains_[IMU] = std::make_shared<confusion::ImuChain>();
#else
    processChains_[IMU] = std::make_shared<confusion::ImuChain>(false);
#endif

    parameters_.emplace_back(confusion::Parameter(T_w_i_.trans.data(), 3, "t_w_i"));
    parameters_.emplace_back(confusion::Parameter(T_w_i_.rot.coeffs().data(),
                                                  4,
                                                  "q_w_i",
                                                  std::make_shared<confusion::QuatParam>()));
    parameters_.emplace_back(confusion::Parameter(linVel_.data(), 3, "linVel"));
    parameters_.emplace_back(confusion::Parameter(accelBias_.data(), 3, "b_a"));
    parameters_.emplace_back(confusion::Parameter(gyroBias_.data(), 3, "b_g"));

    //Set the IMU biases to zero because they are not set in the initialize function
    //so that previously found bises can be re-used when tracking is restarted
    accelBias_.setZero();
    gyroBias_.setZero();
  }

  ImuState(const ImuState *stateIn) : confusion::State(stateIn->t(), NUM_PROCESS_SENSORS, NUM_UPDATE_SENSORS),
                                      T_w_i_(stateIn->T_w_i_),
                                      angVel_(stateIn->angVel_),
                                      linVel_(stateIn->linVel_),
                                      accelBias_(stateIn->accelBias_),
                                      gyroBias_(stateIn->gyroBias_),
                                      referenceFrameOffsets_(stateIn->referenceFrameOffsets_) {

#ifdef OPT_GRAVITY
    processChains_[IMU] = std::make_shared<confusion::ImuChain>(true);
#else
    processChains_[IMU] = std::make_shared<confusion::ImuChain>(false);
#endif

    //todo Clone the passed state parameters to seed the biases and linVel
    parameters_.emplace_back(confusion::Parameter(T_w_i_.trans.data(), 3, "t_w_i"));
    parameters_.emplace_back(confusion::Parameter(T_w_i_.rot.coeffs().data(),
                                                  4,
                                                  "q_w_i",
                                                  std::make_shared<confusion::QuatParam>()));
    parameters_.emplace_back(confusion::Parameter(linVel_.data(), 3, "linVel"));
    parameters_.emplace_back(confusion::Parameter(accelBias_.data(), 3, "b_a"));
    parameters_.emplace_back(confusion::Parameter(gyroBias_.data(), 3, "b_g"));
  }

  void setTagReferenceFrameOffsets(std::map<std::string, std::shared_ptr<confusion::Pose<double>>> *tagReferenceFrameOffsets) {
    referenceFrameOffsets_ = tagReferenceFrameOffsets;
  }

  //Initialize the IMU pose and the global reference frame from the acceleration and tag measurement assuming no sensor acceleration
  bool initFirstState(
      const std::vector<std::deque<std::shared_ptr<confusion::ProcessMeasurement>>> &processMeasBuffer,
      const std::vector<std::deque<std::shared_ptr<confusion::UpdateMeasurement>>> &updateMeasBuffer,
      confusion::StaticParameterVector &staticParameters) {
    // Make sure the external reference frame pointer has been set once the AprilTagModule has been constructed
    if (!referenceFrameOffsets_) {
      std::cout << "ERROR: tagReferenceFrameOffsets not yet set when trying to initialize the first state. This should be set in the estimator constructor!" << std::endl;
      abort();
    }

    //Make sure that there are process measurements preceeding the first state
    if (processMeasBuffer[IMU].empty())
      return false;

    //Initialize from the tag observation
    if (!updateMeasBuffer[TAG].empty() && processMeasBuffer[IMU].back()->t() >= updateMeasBuffer[TAG].front()->t()) {
      //If tags already exist, use those to initialize
      if (!referenceFrameOffsets_->empty()) {
        //We need a tag and an odometry measurement at the same time
        //A tag that already exists has to have been seen.
        bool ready = false;
        std::shared_ptr<confusion::TagMeas> tagMeasPtr;
        std::shared_ptr<confusion::Pose<double>> T_w_ref;
        for (int tagMeasIndex = 0; tagMeasIndex < updateMeasBuffer[TAG].size(); ++tagMeasIndex) {
          //Check if a known tag was observed
          tagMeasPtr = std::dynamic_pointer_cast<confusion::TagMeas>(updateMeasBuffer[TAG][tagMeasIndex]);
          try {
            T_w_ref = referenceFrameOffsets_->at(tagMeasPtr->referenceFrameName_);
            ready = true;
            break;
          }
          catch (const std::out_of_range &oor) {
            continue;
          }
        }

        if (ready) {
          t_ = tagMeasPtr->t();
          confusion::Pose<double> T_c_t_init = tagMeasPtr->getTct();
          T_c_t_init.print("T_c_t_init");

          //Initialize the state
          T_w_i_ = *T_w_ref * T_c_t_init.inverse() * tagMeasPtr->T_c_i();
          T_w_i_.rot.normalize();
          T_w_i_.print("First pose T_w_i");
          angVel_.setZero();
          linVel_.setZero();
          return true;
        }
      } else {
        //Get the IMU measurement closest to the first state instance
        int index = 0;
        while (index < processMeasBuffer[IMU].size() - 1
            && processMeasBuffer[IMU][index + 1]->t() <= updateMeasBuffer[TAG].front()->t())
          ++index;
        std::cout << "Initializing the first state at t=" << updateMeasBuffer[TAG].front()->t() <<
                  " with IMU measurement at t=" << processMeasBuffer[IMU][index]->t() << std::endl;

        //Align the first pose with the accelerometer measurement
        auto imuMeasPtr = std::dynamic_pointer_cast<confusion::ImuMeas>(processMeasBuffer[IMU][index]);
        Eigen::Quaterniond q_w_i_init = Eigen::Quaterniond::FromTwoVectors(imuMeasPtr->a_, Eigen::Vector3d(0, 0, 1));
        std::cout << "first accel: " << imuMeasPtr->a_.transpose() << ", q_w_i_init: "
                  << q_w_i_init.coeffs().transpose() << std::endl;

        //Initialize the first tag pose. It is coincident with the world origin, but rotated.
        auto tagMeasPtr = std::dynamic_pointer_cast<confusion::TagMeas>(updateMeasBuffer[TAG].front());
        t_ = tagMeasPtr->t();
        confusion::Pose<double> T_c_t_init = tagMeasPtr->getTct();

        //		Eigen::Matrix4d T_c_t_mat = getRelativeTransform(tagMeasPtr->tagDetection_, tagMeasPtr->tagMeasCalibration_.tagSize_,
        //					tagMeasPtr->tagMeasCalibration_.projMat_(0,0), tagMeasPtr->tagMeasCalibration_.projMat_(1,1),
        //					tagMeasPtr->tagMeasCalibration_.projMat_(0,2), tagMeasPtr->tagMeasCalibration_.projMat_(1,2));
        //		confusion::Pose<double>T_c_t_init(T_c_t_mat);
        T_c_t_init.print("T_c_t_init");
        confusion::Pose<double> T_w_t_init;
        T_w_t_init.rot = q_w_i_init * tagMeasPtr->T_c_i().rot.conjugate() * T_c_t_init.rot;
        T_w_t_init.print("First tag pose IC");

        std::shared_ptr<confusion::Pose<double>> T_w_ref =
            std::make_shared<confusion::Pose<double>>(T_w_t_init);
        (*referenceFrameOffsets_)[tagMeasPtr->referenceFrameName_] = T_w_ref;

        //Add the tag pose to the static parameters
#ifdef OPT_GRAVITY
        staticParameters.addParameter(confusion::Parameter(
                T_w_ref->trans.data(), 3, "t_w_t", true));
        staticParameters.addParameter(confusion::Parameter(
                T_w_ref->rot.coeffs().data(), 4, "q_w_t", true,
                std::make_shared<confusion::QuatParam>()));
#else
        staticParameters.addParameter(confusion::Parameter(
            T_w_ref->trans.data(), 3, "t_w_t", true));
        Eigen::MatrixXd ftrInitialWeighting(2, 2);
        ftrInitialWeighting.setIdentity();
        ftrInitialWeighting /= tagMeasPtr->tagMeasCalibration_.ftr_init_stddev_;
        confusion::Parameter ftrParam(T_w_ref->rot.coeffs().data(), 4, "q_w_t",
                                      std::make_shared<confusion::FixedYawParameterization>());
        ftrParam.setInitialConstraintWeighting(ftrInitialWeighting);
        staticParameters.addParameter(ftrParam);
#endif
        std::cout << "Added external reference frame with name " << tagMeasPtr->referenceFrameName_ << std::endl;

        //		tagMeasPtr->tag_ = tagVector_.back();

        //Initialize the imu pose
        T_w_i_ = T_w_t_init * T_c_t_init.inverse() * tagMeasPtr->T_c_i();
        T_w_i_.print("First pose IC");
        angVel_.setZero();
        linVel_.setZero();
      }

      return true;
    }

    //Initialize from the posemeas
    //Note that we assume that reference frame offsets from pose measurements are not known a priori!
    if (!updateMeasBuffer[POSEMEAS].empty() && processMeasBuffer[IMU].back()->t() >= updateMeasBuffer[POSEMEAS].front()->t()) {
      if (!referenceFrameOffsets_->empty()) {
        bool ready = false;
        auto poseMeasPtr = std::dynamic_pointer_cast<confusion::PoseMeas>(updateMeasBuffer[POSEMEAS].front());
        std::shared_ptr<confusion::Pose<double>> T_w_ref;
        try {
          T_w_ref = referenceFrameOffsets_->at(poseMeasPtr->referenceFrameName());
          ready = true;
        }
        catch (const std::out_of_range &oor) { }

        if (ready) {
          t_ = poseMeasPtr->t();
          T_w_i_ = *T_w_ref * poseMeasPtr->T_wa_ba() * poseMeasPtr->T_imu_sensor().inverse();
          T_w_i_.rot.normalize();
          T_w_i_.print("First pose T_w_i");
          angVel_.setZero();
          linVel_.setZero();
          return true;
        }
      }
      else {
        //Get the IMU mesetInitialConstraintWeightingasurement closest to the first state instance
        int index = 0;
        while (index < processMeasBuffer[IMU].size() - 1
            && processMeasBuffer[IMU][index + 1]->t() <= updateMeasBuffer[POSEMEAS].front()->t())
          ++index;
        std::cout << "Initializing the first state at t=" << updateMeasBuffer[POSEMEAS].front()->t() <<
                  " with IMU measurement at t=" << processMeasBuffer[IMU][index]->t() << std::endl;

        //Align the first pose with the accelerometer measurement
        auto imuMeasPtr = std::dynamic_pointer_cast<confusion::ImuMeas>(processMeasBuffer[IMU][index]);
        Eigen::Quaterniond q_w_i_init = Eigen::Quaterniond::FromTwoVectors(imuMeasPtr->a_, Eigen::Vector3d(0, 0, 1));
        std::cout << "first accel: " << imuMeasPtr->a_.transpose() << ", q_w_i_init: "
                  << q_w_i_init.coeffs().transpose() << std::endl;

        //Initialize the pose measurement reference frame. It is coincident with the world origin, but rotated to align with gravity
        auto poseMeasPtr = std::dynamic_pointer_cast<confusion::PoseMeas>(updateMeasBuffer[POSEMEAS].front());
        t_ = poseMeasPtr->t();
        confusion::Pose<double> T_wa_ba = poseMeasPtr->T_wa_ba();
        confusion::Pose<double> T_w_wa;
        T_w_wa.trans.setZero();
        T_w_wa.rot = q_w_i_init * poseMeasPtr->T_imu_sensor().rot * T_wa_ba.rot.conjugate();
        T_w_wa.print("T_w_wa initialized to");

        std::shared_ptr<confusion::Pose<double>> T_w_ref = std::make_shared<confusion::Pose<double>>(T_w_wa);
        (*referenceFrameOffsets_)[poseMeasPtr->referenceFrameName()] = T_w_ref;

        //Add the pose measurement reference frame to the static parameters
#ifdef OPT_GRAVITY
        staticParameters.addParameter(confusion::Parameter(
                  T_w_ref->trans.data(), 3, "t_w_wa", true));
          staticParameters.addParameter(confusion::Parameter(
                  T_w_ref->rot.coeffs().data(), 4, "q_w_wa", true,
                  std::make_shared<confusion::QuatParam>()));
#else
        staticParameters.addParameter(confusion::Parameter(
            T_w_ref->trans.data(), 3, "t_w_wa", true));
        staticParameters.addParameter(confusion::Parameter(
            T_w_ref->rot.coeffs().data(), 4, "q_w_wa", std::make_shared<confusion::FixedYawParameterization>()));
#endif

        //Initialize the imu state
        T_w_i_ = T_w_wa * T_wa_ba * poseMeasPtr->T_imu_sensor().inverse();
        T_w_i_.print("First pose IC");
        angVel_.setZero();
        linVel_.setZero();

        return true;
      }
    }

    return false;
  }




  //Step through the process measurements to get the predicted new state. Can also add the appropriate prediction measurements here as well.
  //There is risk that this will process the update measurements in a different order than the StateVector. Is that a problem?
  //todo TEMP Made this virtual to support creating the shared_ptr of the derived class
  virtual std::shared_ptr<State> createNextState(
      const std::vector<std::deque<std::shared_ptr<confusion::ProcessMeasurement>>> &processMeasBuffer,
      const std::vector<std::deque<std::shared_ptr<confusion::UpdateMeasurement>>> &updateMeasBuffer,
      confusion::StaticParameterVector &staticParameters) {
    bool foundGoodUpdate = false;
    double t_des;
    size_t measType = 0;
    while (measType < NUM_UPDATE_SENSORS) {
      if (!updateMeasBuffer[measType].empty() && processMeasBuffer[IMU].back()->t() >= updateMeasBuffer[measType].front()->t()) {
        t_des = updateMeasBuffer[measType].front()->t();
        foundGoodUpdate = true;
        break;
      }
      ++measType;
    }

    if (!foundGoodUpdate) {
      std::cout << "[ImuState::createNextState] Didn't receive a process measurement to initialize the next state. Will wait and try again." << std::endl;
      return nullptr;
    }

    //Forward propagate through the IMU measurements
    auto state = std::make_shared<ImuState>(this);

#ifdef OPT_GRAVITY
    auto firstMeasPtr = std::dynamic_pointer_cast<confusion::ImuMeas>(processMeasBuffer[IMU].front());
    Eigen::Vector3d g_w = confusion::gravityVec(firstMeasPtr->gravity_rot_->data(),
            firstMeasPtr->imuCalibration_->gravityMagnitude_);
#else
    auto firstMeasPtr = std::dynamic_pointer_cast<confusion::ImuMeas>(processMeasBuffer[IMU].front());
    Eigen::Vector3d g_w = firstMeasPtr->imuCalibration_->g_w_;
#endif

    forwardPropagateImuMeas(processMeasBuffer[IMU], t_, t_des, g_w,
                            state->T_w_i_, state->linVel_, state->accelBias_, state->gyroBias_);

    state->t_ = t_des;
//		state->print();

    return std::dynamic_pointer_cast<State>(state);
  }

  bool addUpdateMeasDerived(std::shared_ptr<confusion::UpdateMeasurement> measPtr,
                            confusion::StaticParameterVector &staticParameters) {
    switch (measPtr->measType()) {
      case TAG : {
        //Initialize the tag pose if a new tag was observed
        auto tagMeasPtr = std::dynamic_pointer_cast<confusion::TagMeas>(measPtr);
        if (tagMeasPtr->T_w_ref_ptr_)
          std::cout << "addUpdateMeas: tagMeas already has a reference frame assigned!?" << std::endl;

        std::shared_ptr<confusion::Pose<double>> T_w_ref;
        try {
          T_w_ref = referenceFrameOffsets_->at(tagMeasPtr->referenceFrameName_);
        }
        catch (const std::out_of_range &oor) {
          confusion::Pose<double> T_c_t_init = tagMeasPtr->getTct();
          T_w_ref = std::make_shared<confusion::Pose<double>>(T_w_i_ * tagMeasPtr->T_c_i().inverse() * T_c_t_init);
          (*referenceFrameOffsets_)[tagMeasPtr->referenceFrameName_] = T_w_ref;

          //Add the tag pose to the static parameters
          staticParameters.addParameter(confusion::Parameter(T_w_ref->trans.data(), 3, "T_w_t"));
          staticParameters.addParameter(confusion::Parameter(T_w_ref->rot.coeffs().data(),
                                                             4,
                                                             "q_w_t",
                                                             std::make_shared<confusion::QuatParam>()));
          std::cout << "Added external reference frame with name " << tagMeasPtr->referenceFrameName_ << std::endl;
          T_w_ref->print("tag pose init");
        }

        tagMeasPtr->assignExternalReferenceFrame(T_w_ref);

        break;
      }
      case POSEMEAS : {
        //Initialize the pose meas reference frame offset if it hasn't been initialized yet
        auto poseMeasPtr = std::dynamic_pointer_cast<confusion::PoseMeas>(measPtr);

        std::shared_ptr<confusion::Pose<double>> T_w_ref;
        try {
          T_w_ref = referenceFrameOffsets_->at(poseMeasPtr->referenceFrameName());
        }
        catch (const std::out_of_range &oor) {
          T_w_ref = std::make_shared<confusion::Pose<double>>(T_w_i_ * poseMeasPtr->T_imu_sensor() * poseMeasPtr->T_wa_ba().inverse());
          (*referenceFrameOffsets_)[poseMeasPtr->referenceFrameName()] = T_w_ref;

          //Add the tag pose to the static parameters
          staticParameters.addParameter(confusion::Parameter(T_w_ref->trans.data(), 3, "T_w_ref"));
          staticParameters.addParameter(confusion::Parameter(T_w_ref->rot.coeffs().data(),
                                                             4,
                                                             "q_w_ref",
                                                             std::make_shared<confusion::QuatParam>()));
          std::cout << "Added external reference frame with name " << poseMeasPtr->referenceFrameName() << std::endl;
          T_w_ref->print("ref pose init");
        }

        poseMeasPtr->assignExternalReferenceFrame(T_w_ref);

        break;
      }
      default :
        std::cout << "[ImuState::createNextState] Unknown update measurement type received!?" << std::endl;
    }

    //Assign the measurement to the state
    updateMeasurements_[measPtr->measType()].push_back(measPtr);

    return true;
  }

  Eigen::Vector3d getAngularVelocity() {
    if (processChains_[IMU]->measurements_.empty())
      return Eigen::Vector3d::Zero();

    auto firstMeasPtr = std::dynamic_pointer_cast<confusion::ImuMeas>(processChains_[IMU]->measurements_.front());
    angVel_ = T_w_i_.rot * (firstMeasPtr->w_ - gyroBias_);
    return angVel_;
  }

  confusion::ImuStateParameters getParameterStruct() {
    confusion::ImuStateParameters params;
    params.t_ = t();
    params.T_w_i_ = T_w_i_;
    params.angVel_ = angVel_;
    params.linVel_ = linVel_;
    params.accelBias_ = accelBias_;
    params.gyroBias_ = gyroBias_;

    return params;
  }

  void print() {
    std::cout << "t=" << t_ << "; t_w_i: " << T_w_i_.trans.transpose() <<
              "; q_w_i: " << T_w_i_.rot.coeffs().transpose() <<
              "; v_w_i: " << linVel_.transpose() <<
              "; accelBias_: " << accelBias_.transpose() <<
              "; gyroBias_: " << gyroBias_.transpose() << std::endl;
  }

  confusion::Pose<double> T_w_i_;
  Eigen::Vector3d angVel_;
  Eigen::Vector3d linVel_;
  Eigen::Vector3d accelBias_;
  Eigen::Vector3d gyroBias_;

  std::map<std::string, std::shared_ptr<confusion::Pose<double>>> *referenceFrameOffsets_ = nullptr;
};

#endif /* INCLUDE_CONFUSION_EXAMPLES_IMUSTATE_H_ */
