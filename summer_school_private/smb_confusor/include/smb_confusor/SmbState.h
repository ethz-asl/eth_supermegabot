//
// Created by tim on 17.11.18.
//

#ifndef SMB_SMBSTATE_H
#define SMB_SMBSTATE_H

//#define OPT_GRAVITY ; Instead of optimizing the roll/pitch of the world reference frame, optimize the direction of gravity

#include <confusion/State.h>
#include "smb_confusor/SensorEnumDefinition.h"
#include <confusion/models/ImuChain.h>
#include <confusion/models/TagMeas.h>
#include <confusion/models/PoseMeas.h>
#include "smb_confusor/BaseMotionModel.h"
#include <confusion/utilities/Pose.h>

class SmbState : public confusion::State {
 public:
  SmbState(int &stateInitializationSensor, double t = 0.0) :
            confusion::State(t, NUM_PROCESS_SENSORS, NUM_UPDATE_SENSORS),
            stateInitializationSensor_(stateInitializationSensor) {
#ifdef OPT_GRAVITY
    processChains_[IMU] = std::make_shared<confusion::ImuChain>();
#else
    processChains_[IMU] = std::make_shared<confusion::ImuChain>(false);
#endif

    processChains_[BMM] = std::make_shared<BaseMotionModel>();

    parameters_.emplace_back(confusion::Parameter(T_w_i_.trans.data(), 3, "t_w_i"));
    parameters_.emplace_back(confusion::Parameter(T_w_i_.rot.coeffs().data(), 4, "q_w_i",
    std::make_shared<confusion::QuatParam>()));
    parameters_.emplace_back(confusion::Parameter(linVel_.data(), 3, "linVel"));
    parameters_.emplace_back(confusion::Parameter(accelBias_.data(), 3, "b_a"));
    parameters_.emplace_back(confusion::Parameter(gyroBias_.data(), 3, "b_g"));

    //Set the IMU biases to zero because they are not set in the initialize function
    //so that previously found bises can be re-used when tracking is restarted
    accelBias_.setZero();
    gyroBias_.setZero();
  }

  SmbState(const SmbState *stateIn) : confusion::State(stateIn->t(), NUM_PROCESS_SENSORS, NUM_UPDATE_SENSORS),
                                      T_w_i_(stateIn->T_w_i_),
                                      angVel_(stateIn->angVel_),
                                      linVel_(stateIn->linVel_),
                                      accelBias_(stateIn->accelBias_),
                                      gyroBias_(stateIn->gyroBias_),
                                      tagReferenceFrameOffsets_(stateIn->tagReferenceFrameOffsets_),
                                      externalReferenceFrameOffsets_(stateIn->externalReferenceFrameOffsets_),
                                      stateInitializationSensor_(stateIn->stateInitializationSensor_){

#ifdef OPT_GRAVITY
    processChains_[IMU] = std::make_shared<confusion::ImuChain>(true);
#else
    processChains_[IMU] = std::make_shared<confusion::ImuChain>(false);
#endif

    processChains_[BMM] = std::make_shared<BaseMotionModel>();

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
    tagReferenceFrameOffsets_ = tagReferenceFrameOffsets;
  }

  void setExternalReferenceFrameOffsets(std::map<std::string, std::shared_ptr<confusion::Pose<double>>> *externalReferenceFrameOffsets) {
    externalReferenceFrameOffsets_ = externalReferenceFrameOffsets;
  }

  //Initialize the IMU pose and the global reference frame from the acceleration and tag measurement assuming no sensor acceleration
  bool initFirstState(
      const std::vector<std::deque<std::shared_ptr<confusion::ProcessMeasurement>>> &processMeasBuffer,
      const std::vector<std::deque<std::shared_ptr<confusion::UpdateMeasurement>>> &updateMeasBuffer,
      confusion::StaticParameterVector &staticParameters) override {
    std::cout << "[SmbState::initFirstState] Trying to initialize first state with process sensor at index " << stateInitializationSensor_ << "..." << std::endl;

    // Make sure the external reference frame pointer has been set once the ExternalReferenceTrackingModule has been constructed
    if (!tagReferenceFrameOffsets_ && !externalReferenceFrameOffsets_) {
      std::cout << "ERROR: Neither tag nor external reference frame offsets set when trying to initialize the first state. This should be set in the estimator constructor!" << std::endl;
      abort();
    }

    //Make sure that there are process measurements preceeding the first state
    if (processMeasBuffer[stateInitializationSensor_].empty())
      return false;
std::cout << "here0" << std::endl;
    //Initialize from the tag observation
    //todo There could be problem if one type of reference frame (tag or external) is pre-specified but the other type of measurement is the first one received
    if (tagReferenceFrameOffsets_) {
      if (!updateMeasBuffer[TAG].empty() && processMeasBuffer[stateInitializationSensor_].back()->t() >= updateMeasBuffer[TAG].front()->t()) {
        //If tags already exist, use those to initialize
        if (!tagReferenceFrameOffsets_->empty()) {
          //We need a tag and an odometry measurement at the same time
          //A tag that already exists has to have been seen.
          bool ready = false;
          std::shared_ptr<confusion::TagMeas> tagMeasPtr;
          std::shared_ptr<confusion::Pose<double>> T_w_ref;
          for (int tagMeasIndex = 0; tagMeasIndex < updateMeasBuffer[TAG].size(); ++tagMeasIndex) {
            //Check if a known tag was observed
            tagMeasPtr = std::dynamic_pointer_cast<confusion::TagMeas>(updateMeasBuffer[TAG][tagMeasIndex]);
            try {
              T_w_ref = tagReferenceFrameOffsets_->at(tagMeasPtr->referenceFrameName_);
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
          confusion::Pose<double> T_w_t_init;
          confusion::Pose<double> T_c_t_init;
          std::shared_ptr<confusion::TagMeas> tagMeasPtr;
          switch(stateInitializationSensor_) {
            case IMU : {
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
              tagMeasPtr = std::dynamic_pointer_cast<confusion::TagMeas>(updateMeasBuffer[TAG].front());
              t_ = tagMeasPtr->t();
              T_c_t_init = tagMeasPtr->getTct();

              T_c_t_init.print("T_c_t_init");

              T_w_t_init.rot = q_w_i_init * tagMeasPtr->T_c_i().rot.conjugate() * T_c_t_init.rot;
              T_w_t_init.print("First tag pose IC");
              break;
            }
            case BMM : {
              // Make sure that IMU measurements are not also being used, since the world will not be gravity-aligned in this case
              if (!processMeasBuffer[IMU].empty()) {
                std::cout << "\n\n[SmbState::initFirstState] WARNING: BMM is being used for state creation even "
                             "though IMU measurements are also being used! This will not work because the world "
                             "will not be gravity aligned!\n\n" << std::endl;
              }

              // Make sure a BMM measurements has been received before the first state time
              int index = 0;
              while (processMeasBuffer[BMM].back()->t() < updateMeasBuffer[TAG][index]->t())
                ++index;
              std::cout << "Initializing the first state at t=" << updateMeasBuffer[TAG][index]->t() << std::endl;

              // Here we just set the world frame coincident with the first tag observed, so T_w_t_init is identity
              tagMeasPtr = std::dynamic_pointer_cast<confusion::TagMeas>(updateMeasBuffer[TAG][index]);
              t_ = tagMeasPtr->t();
              T_c_t_init = tagMeasPtr->getTct();

              break;
            }
            default :
              std::cout << "[SmbState::createNextState] ERROR: Unexpected stateInitializationSensor set!" << std::endl;
          }

          std::shared_ptr<confusion::Pose<double>> T_w_ref =
              std::make_shared<confusion::Pose<double>>(T_w_t_init);
          (*tagReferenceFrameOffsets_)[tagMeasPtr->referenceFrameName_] = T_w_ref;

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
    }

    //Initialize from the posemeas
    //Note that we assume that reference frame offsets from pose measurements are not known a priori!
    if (externalReferenceFrameOffsets_) {
      std::cout << "here1" << std::endl;
      if (!updateMeasBuffer[POSEMEAS].empty() &&
          processMeasBuffer[stateInitializationSensor_].back()->t() >= updateMeasBuffer[POSEMEAS].front()->t()) {
        if (!externalReferenceFrameOffsets_->empty()) {
          bool ready = false;
          auto poseMeasPtr = std::dynamic_pointer_cast<confusion::PoseMeas>(updateMeasBuffer[POSEMEAS].front());
          std::shared_ptr<confusion::Pose<double>> T_w_ref;
          try {
            T_w_ref = externalReferenceFrameOffsets_->at(poseMeasPtr->referenceFrameName());
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
          std::cout << "here2" << std::endl;
          confusion::Pose<double> T_w_wa;
          confusion::Pose<double> T_wa_ba;
          std::shared_ptr<confusion::PoseMeas> poseMeasPtr;
          switch(stateInitializationSensor_) {
            case IMU : {
              //Get the IMU measurement closest to the first state instance
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
              poseMeasPtr = std::dynamic_pointer_cast<confusion::PoseMeas>(updateMeasBuffer[POSEMEAS].front());
              t_ = poseMeasPtr->t();
              T_wa_ba = poseMeasPtr->T_wa_ba();
              T_w_wa.trans.setZero();
              T_w_wa.rot = q_w_i_init * poseMeasPtr->T_imu_sensor().rot * T_wa_ba.rot.conjugate();
              T_w_wa.print("T_w_wa initialized to");
            }
            case BMM : {
              std::cout << "here3" << std::endl;
              // Make sure that IMU measurements are not also being used, since the world will not be gravity-aligned in this case
              if (!processMeasBuffer[IMU].empty()) {
                std::cout << "\n\n[SmbState::initFirstState] WARNING: BMM is being used for state creation even "
                             "though IMU measurements are also being used! This will not work because the world "
                             "will not be gravity aligned!\n\n" << std::endl;
              }

              // Make sure a BMM measurements has been received before the first state time
              int index = 0;
              while (processMeasBuffer[BMM].back()->t() < updateMeasBuffer[POSEMEAS][index]->t())
                ++index;
              std::cout << "Initializing the first state at t=" << updateMeasBuffer[POSEMEAS][index]->t() << std::endl;

              // Here we just set the world frame coincident with the start robot pose, so T_w_wa stays identity
              poseMeasPtr = std::dynamic_pointer_cast<confusion::PoseMeas>(updateMeasBuffer[POSEMEAS][index]);
              t_ = poseMeasPtr->t();
              T_wa_ba = poseMeasPtr->T_wa_ba();
              break;
            }
            default:
              std::cout << "[SmbState::createNextState] ERROR: Unexpected stateInitializationSensor set!" << std::endl;
              abort();
          }

          std::shared_ptr<confusion::Pose<double>> T_w_ref = std::make_shared<confusion::Pose<double>>(T_w_wa);
          (*externalReferenceFrameOffsets_)[poseMeasPtr->referenceFrameName()] = T_w_ref;

          //Add the pose measurement reference frame to the static parameters
  #ifdef OPT_GRAVITY
          staticParameters.addParameter(confusion::Parameter(
                    T_w_ref->trans.data(), 3, "t_world_"+poseMeasPtr->referenceFrameName(), true));
            staticParameters.addParameter(confusion::Parameter(
                    T_w_ref->rot.coeffs().data(), 4, "q_world_"+poseMeasPtr->referenceFrameName(), true,
                    std::make_shared<confusion::QuatParam>()));
  #else
          staticParameters.addParameter(confusion::Parameter(
              T_w_ref->trans.data(), 3, "t_world_"+poseMeasPtr->referenceFrameName(), true));
          staticParameters.addParameter(confusion::Parameter(
              T_w_ref->rot.coeffs().data(), 4, "q_world_"+poseMeasPtr->referenceFrameName(),
              std::make_shared<confusion::FixedYawParameterization>()));
  #endif

          //Initialize the imu state
          T_w_i_ = T_w_wa * T_wa_ba * poseMeasPtr->T_imu_sensor().inverse();
          T_w_i_.print("First pose IC");
          angVel_.setZero();
          linVel_.setZero();

          return true;
        }
      }
    }

    return false;
  }

  std::shared_ptr<State> createNextState(
      const std::vector<std::deque<std::shared_ptr<confusion::ProcessMeasurement>>> &processMeasBuffer,
      const std::vector<std::deque<std::shared_ptr<confusion::UpdateMeasurement>>> &updateMeasBuffer,
      confusion::StaticParameterVector &staticParameters) override {
    bool foundGoodUpdate = false;
    double t_des;
    size_t measType = 0;
    while (measType < NUM_UPDATE_SENSORS) {
      if (!updateMeasBuffer[measType].empty() &&
          processMeasBuffer[stateInitializationSensor_].back()->t() >= updateMeasBuffer[measType].front()->t()) {
        t_des = updateMeasBuffer[measType].front()->t();
        foundGoodUpdate = true;
        break;
      }
      ++measType;
    }

    if (!foundGoodUpdate) {
      std::cout << "[SmbState::createNextState] Didn't receive a process measurement to initialize the next state. Will wait and try again." << std::endl;
      return nullptr;
    }

    //Forward propagate through the IMU measurements
    auto state = std::make_shared<SmbState>(this);

    switch (stateInitializationSensor_) {
      case IMU: {
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

        break;
      }
      case BMM: {
        //todo Remember that this doesn't update the linear velocity/IMU biases
        auto wheelSpeeds = std::dynamic_pointer_cast<WheelSpeeds>(processMeasBuffer[BMM].front());
        confusion::Pose<double> T_i_b = wheelSpeeds->T_imu_base_;
        confusion::Pose<double> T_w_b = state->T_w_i_ * T_i_b;
        forwardPropagateBmm(processMeasBuffer[BMM], t_, t_des, T_w_b);
        state->T_w_i_ = T_w_b * T_i_b.inverse();

        break;
      }
      default:
        std::cout << "[SmbState::createNextState] ERROR: Unexpected stateInitializationSensor set!" << std::endl;
        abort();
    }

    state->t_ = t_des;
//		state->print();

    return std::dynamic_pointer_cast<State>(state);
  }

  bool addUpdateMeasDerived(std::shared_ptr<confusion::UpdateMeasurement> measPtr,
                            confusion::StaticParameterVector &staticParameters) override {
    switch (measPtr->measType()) {
      case TAG : {
        if (!tagReferenceFrameOffsets_)
          return false;

        //Initialize the tag pose if a new tag was observed
        auto tagMeasPtr = std::dynamic_pointer_cast<confusion::TagMeas>(measPtr);
        if (tagMeasPtr->T_w_ref_ptr_)
          std::cout << "addUpdateMeas: tagMeas already has a reference frame assigned!?" << std::endl;

        std::shared_ptr<confusion::Pose<double>> T_w_ref;
        try {
          T_w_ref = tagReferenceFrameOffsets_->at(tagMeasPtr->referenceFrameName_);
        }
        catch (const std::out_of_range &oor) {
          confusion::Pose<double> T_c_t_init = tagMeasPtr->getTct();
          T_w_ref = std::make_shared<confusion::Pose<double>>(T_w_i_ * tagMeasPtr->T_c_i().inverse() * T_c_t_init);
          (*tagReferenceFrameOffsets_)[tagMeasPtr->referenceFrameName_] = T_w_ref;

          //Add the tag pose to the static parameters
          staticParameters.addParameter(confusion::Parameter(T_w_ref->trans.data(), 3, "t_world_" + tagMeasPtr->referenceFrameName_));
          staticParameters.addParameter(confusion::Parameter(T_w_ref->rot.coeffs().data(),
                                                             4,
                                                             "q_world_" + tagMeasPtr->referenceFrameName_,
                                                             std::make_shared<confusion::QuatParam>()));
          std::cout << "Added external reference frame with name " << tagMeasPtr->referenceFrameName_ << std::endl;
          T_w_ref->print("tag pose init");
        }

        tagMeasPtr->assignExternalReferenceFrame(T_w_ref);

        break;
      }
      case POSEMEAS : {
        if (!externalReferenceFrameOffsets_)
          return false;

        //Initialize the pose meas reference frame offset if it hasn't been initialized yet
        auto poseMeasPtr = std::dynamic_pointer_cast<confusion::PoseMeas>(measPtr);

        std::shared_ptr<confusion::Pose<double>> T_w_ref;
        try {
          T_w_ref = externalReferenceFrameOffsets_->at(poseMeasPtr->referenceFrameName());
        }
        catch (...) {
          T_w_ref = std::make_shared<confusion::Pose<double>>(T_w_i_ * poseMeasPtr->T_imu_sensor() * poseMeasPtr->T_wa_ba().inverse());
          (*externalReferenceFrameOffsets_)[poseMeasPtr->referenceFrameName()] = T_w_ref;

          //Add the tag pose to the static parameters
          staticParameters.addParameter(confusion::Parameter(T_w_ref->trans.data(), 3, "t_world_" + poseMeasPtr->referenceFrameName()));
          staticParameters.addParameter(confusion::Parameter(T_w_ref->rot.coeffs().data(),
                                                             4,
                                                             "q_world_" + poseMeasPtr->referenceFrameName(),
                                                             std::make_shared<confusion::QuatParam>()));
          std::cout << "Added external reference frame with name " << poseMeasPtr->referenceFrameName() << std::endl;
          T_w_ref->print("ref pose init");
        }

        poseMeasPtr->assignExternalReferenceFrame(T_w_ref);

        break;
      }
      default :
        std::cout << "[SmbState::createNextState] Unknown update measurement type received!?" << std::endl;
        return false;
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

  void print() override {
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

  std::map<std::string, std::shared_ptr<confusion::Pose<double>>> *tagReferenceFrameOffsets_ = nullptr;
  std::map<std::string, std::shared_ptr<confusion::Pose<double>>> *externalReferenceFrameOffsets_ = nullptr;

  int &stateInitializationSensor_;
};
#endif //SMB_SMBSTATE_H
