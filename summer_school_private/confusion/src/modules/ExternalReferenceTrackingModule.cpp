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

#include "confusion/modules/external_reference_tracking/ExternalReferenceTrackingModule.h"

namespace confusion {

ExternalReferenceTrackingModule::ExternalReferenceTrackingModule(ros::NodeHandle &node,
                                                                 ConFusor &conFusor,
                                                                 std::string configFileName,
                                                                 int poseMeasIndex,
                                                                 bool* newMeasReceivedFlag)
    : node_(node),
      conFusor_(conFusor),
      poseMeasIndex_(poseMeasIndex) {
  std::cout << "[ExternalReferenceTrackingModule] PoseMeas index is " << poseMeasIndex_ << std::endl;
  if (newMeasReceivedFlag)
    newMeasReceivedFlag_ = newMeasReceivedFlag;

  confusionPath_ = ros::package::getPath("confusion");
  boost::property_tree::read_info(configFileName, pt);
#ifdef OPT_GRAVITY
  //	Eigen::MatrixXd gravityRotInitialWeighting(2,2);
  //	gravityRotInitialWeighting.setIdentity();
  //	gravityRotInitialWeighting /=
  // tagTrackerParameters_.tagMeasCalibration_.ftr_init_stddev;
  conFusor_.addStaticParameter(
      confusion::Parameter(gravity_rot_.data(), 2, "g_r"));  //, gravityRotInitialWeighting);
#endif

  // Initialize external posemeas sources and subscribers.
  for (auto &child : pt.get_child("external_pose_meas")) {
    std::string sensor_name = child.first.data();
    initializeSensorframeOffset(sensor_name);

    // Check for msg types and initialize subscribers.
    std::string msg_type = child.second.get<std::string>("msg_type");
    if (msg_type == "geometry_msgs::TransformStamped") {
      subsExternalPoseMeas_.push_back(
          node_.subscribe(child.second.get<std::string>("ros_topic"), 10,
                          &ExternalReferenceTrackingModule::externalPoseMeasTFStampedCallback, this,
                          ros::TransportHints().tcpNoDelay()));
    } else if (msg_type == "nav_msgs::Odometry") {
      subsExternalPoseMeas_.push_back(
          node_.subscribe(child.second.get<std::string>("ros_topic"), 10,
                          &ExternalReferenceTrackingModule::externalPoseMeasOdomCallback, this,
                          ros::TransportHints().tcpNoDelay()));
    } else if (msg_type == "OdomWithState") {
      subsExternalPoseMeas_.push_back(
          node_.subscribe(child.second.get<std::string>("ros_topic"), 10,
                          &ExternalReferenceTrackingModule::externalPoseMeasOdomWithStateCallback, this,
                          ros::TransportHints().tcpNoDelay()));
    } else {
      std::cerr << " unknown message type: " << msg_type;
    }
  }

  std::cout << "[ExternalReferenceTrackingModule] Initialization complete." << std::endl;
}

void ExternalReferenceTrackingModule::initializeSensorframeOffset(std::string sensor_name) {
  // get the configuration subtree of this sensorframe
  boost::property_tree::ptree config = pt.get_child("external_pose_meas." + sensor_name);
  std::string frame = config.get<std::string>("frame");

  // create pose for initial offset. The user can specify T_body_sensor or T_sensor_body
  auto T_body_sensor = std::make_shared<confusion::Pose<double>>();
  std::string settings_key = "T_body_" + frame;
  boost::optional<double> tx_body_sensor = config.get_optional<double>(settings_key + ".px");
  if (tx_body_sensor) {
    std::cout << "Found " << settings_key << std::endl;
    T_body_sensor->trans(0) = config.get<double>(settings_key + ".px");
    T_body_sensor->trans(1) = config.get<double>(settings_key + ".py");
    T_body_sensor->trans(2) = config.get<double>(settings_key + ".pz");
    T_body_sensor->rot.w() = config.get<double>(settings_key + ".qw");
    T_body_sensor->rot.x() = config.get<double>(settings_key + ".qx");
    T_body_sensor->rot.y() = config.get<double>(settings_key + ".qy");
    T_body_sensor->rot.z() = config.get<double>(settings_key + ".qz");
    T_body_sensor->rot.normalize();
  }
  else {
    std::string settings_key_inv = "T_" + frame + "_body";
    boost::optional<double> tx_sensor_body = config.get_optional<double>(settings_key_inv + ".px");
    if (tx_sensor_body) {
      std::cout << "Found " << settings_key_inv << std::endl;
      confusion::Pose<double> T_sensor_body;
      T_sensor_body.trans(0) = config.get<double>(settings_key_inv + ".px");
      T_sensor_body.trans(1) = config.get<double>(settings_key_inv + ".py");
      T_sensor_body.trans(2) = config.get<double>(settings_key_inv + ".pz");
      T_sensor_body.rot.w() = config.get<double>(settings_key_inv + ".qw");
      T_sensor_body.rot.x() = config.get<double>(settings_key_inv + ".qx");
      T_sensor_body.rot.y() = config.get<double>(settings_key_inv + ".qy");
      T_sensor_body.rot.z() = config.get<double>(settings_key_inv + ".qz");
      T_sensor_body.rot.normalize();

      *T_body_sensor = T_sensor_body.inverse();
    }
    else {
      std::cout << "ERROR: Neither " << settings_key << " nor " << settings_key_inv <<
          " are specified in the config file for sensor " << frame << std::endl;
      abort();
    }
  }
  T_body_sensor->print(settings_key);
  sensorFrameOffsets_[frame] = T_body_sensor;
  std::cout << "Sensor frame offset for " << sensor_name << " at addresses " << T_body_sensor->trans.data() << " and " << T_body_sensor->rot.coeffs().data() << std::endl;

  // scale defaults to 1
  auto scale = std::make_shared<double>(config.get<double>("scale", 1.0));
  sensorPoseMeasScales_[frame] = scale;

  auto pose_meas_config = std::make_shared<confusion::PoseMeasConfig>();
  pose_meas_config->w_trans = 1.0 / config.get<double>("pose_meas_trans_stddev");
  pose_meas_config->w_rot = 1.0 / config.get<double>("pose_meas_rot_stddev");
  pose_meas_config->sensorOffset_trans_init_stddev = config.get<double>("t_init_stddev");
  pose_meas_config->sensorOffset_rot_init_stddev = config.get<double>("q_init_stddev");
  pose_meas_config->useLossFunction = config.get<bool>("use_loss_function");
  pose_meas_config->lossCoefficient = config.get<double>("loss_coefficient");
  pose_meas_config->optTranslationalExtrinsic = config.get<bool>("optimize_t_body");
  pose_meas_config->optRotationalExtrinsic = config.get<bool>("optimize_q_body");
  pose_meas_config->optScale = config.get<bool>("optimize_scale", false);
  sensorPoseMeasConfigs_[frame] = pose_meas_config;

  // Optimisation of Translation
  if (pose_meas_config->optTranslationalExtrinsic) {
    Eigen::MatrixXd t_body_sensor_initial_weighting(3, 3);
    t_body_sensor_initial_weighting.setIdentity();
    t_body_sensor_initial_weighting /= pose_meas_config->sensorOffset_trans_init_stddev;
    confusion::Parameter tParam(T_body_sensor->trans.data(), 3, "t_body_" + frame);
    tParam.setInitialConstraintWeighting(t_body_sensor_initial_weighting);
    conFusor_.addStaticParameter(tParam);
  } else {
    conFusor_.addStaticParameter(
        confusion::Parameter(T_body_sensor->trans.data(), 3, "t_body_" + frame, true));
  }

  // Optimisation of Rotation
  if (pose_meas_config->optRotationalExtrinsic) {
    Eigen::MatrixXd q_body_sensor_initial_weighting(3, 3);
    q_body_sensor_initial_weighting.setIdentity();
    q_body_sensor_initial_weighting /= pose_meas_config->sensorOffset_rot_init_stddev;

    confusion::Parameter tParam(T_body_sensor->rot.coeffs().data(), 4, "q_body_" + frame, false,
                                std::make_shared<confusion::QuatParam>());
    tParam.setInitialConstraintWeighting(q_body_sensor_initial_weighting);
    conFusor_.addStaticParameter(tParam);
  } else {
    conFusor_.addStaticParameter(confusion::Parameter(T_body_sensor->rot.coeffs().data(), 4,
                                                      "q_body_" + frame, true,
                                                      std::make_shared<confusion::QuatParam>()));
  }

  // Optimisation of Scale
  if (pose_meas_config->optScale) {
    conFusor_.addStaticParameter(
        confusion::Parameter(scale.get(), 1, "scale_" + sensor_name, false));
  } else {
    conFusor_.addStaticParameter(
        confusion::Parameter(scale.get(), 1, "scale_" + sensor_name, true));
  }
};

void ExternalReferenceTrackingModule::externalPoseMeasTFStampedCallback(
    const geometry_msgs::TransformStampedPtr &msg) {
  externalPoseMeasCallback(msg);
}

void ExternalReferenceTrackingModule::externalPoseMeasOdomCallback(const nav_msgs::OdometryPtr &msg) {
  externalPoseMeasCallback(msg);
}

void ExternalReferenceTrackingModule::externalPoseMeasOdomWithStateCallback(const confusion::OdomWithStatePtr &msg) {
  externalPoseMeasCallback(msg);
}

//todo Can you set the callbacks using the templated function directly?
template <typename MsgType>
void ExternalReferenceTrackingModule::externalPoseMeasCallback(const MsgType &msg) {
  double t = msg->header.stamp.toSec();

  // Get the sensor configuration
  std::shared_ptr<confusion::Pose<double>> sensorOffsetPtr;
  std::shared_ptr<confusion::PoseMeasConfig> poseMeasConfigPtr;
  std::shared_ptr<double> scalePtr;
  try {
    sensorOffsetPtr = sensorFrameOffsets_.at(msg->child_frame_id);
    poseMeasConfigPtr = sensorPoseMeasConfigs_.at(msg->child_frame_id);
    scalePtr = sensorPoseMeasScales_.at(msg->child_frame_id);
  } catch (...) {
    std::cerr << "ERROR: PoseMeas received message with uninitialized child_frame_id '"
              << msg->child_frame_id << "'. Throwing out the measurement." << std::endl;
    return;
  }

  confusion::Pose<double> T_wa_ba_meas = confusion::getPoseFromMsg(msg);

  auto poseMeasPtr = std::make_shared<confusion::PoseMeas>(t, msg->header.frame_id, *sensorOffsetPtr, T_wa_ba_meas,
                                                           *poseMeasConfigPtr, poseMeasIndex_); //scalePtr.get(), poseMeasIndex_);
  std::cout << "Created pose meas at t=" << poseMeasPtr->t() <<
        ", type: " << poseMeasPtr->measType() <<
        ", refFrame: " << poseMeasPtr->referenceFrameName() <<
        ", delay vs now time: " << ros::Time::now().toSec() - t << std::endl;
  conFusor_.addUpdateMeasurement(poseMeasPtr);
  *newMeasReceivedFlag_ = true;

#ifdef SYNC_DEBUG
  std::cout << "PoesMeas received with stamp " << t << " from frame " << msg->header.frame_id
            << " to frame " << msg->child_frame_id << std::endl;
#endif
}

//todo Is there a nice way to have this called automatically after solving?
void ExternalReferenceTrackingModule::copyOutParametersAfterOptimization() {
  std::lock_guard<std::mutex> lg(parameterQueryMtx_);
  for (auto &externalReferenceFrame : referenceFrameOffsets_)
    referenceFrameOffsetsCopy_[externalReferenceFrame.first] = *externalReferenceFrame.second;
  for (auto &sensorFrame : sensorFrameOffsets_)
    sensorFrameOffsetsCopy_[sensorFrame.first] = *sensorFrame.second;
  for (auto &scaleParam : sensorPoseMeasScales_)
    sensorPoseMeasScalesCopy_[scaleParam.first] = *scaleParam.second;
}

void ExternalReferenceTrackingModule::publishFrameOffsetTfs() {
  ros::Time t_tf = ros::Time::now();
  std::lock_guard<std::mutex> lg(parameterQueryMtx_);
  for (auto &externalReferenceFrame : referenceFrameOffsetsCopy_) {
    tfBroadcaster_.sendTransform(tf::StampedTransform(getTfMsg(externalReferenceFrame.second),
                                                      t_tf, "world", externalReferenceFrame.first));
    //externalReferenceFrame.second.print(externalReferenceFrame.first);
  }
  for (auto &sensorFrame : sensorFrameOffsetsCopy_) {
    tfBroadcaster_.sendTransform(tf::StampedTransform(getTfMsg(sensorFrame.second),
                                                      t_tf, "body", sensorFrame.first));
    //sensorFrame.second.print(sensorFrame.first);
  }
}

bool ExternalReferenceTrackingModule::getReferenceFrameOffset(const std::string &frameName, confusion::Pose<double> &T_world_ref) const {
  std::lock_guard<std::mutex> lg(parameterQueryMtx_);
  try {
    T_world_ref = referenceFrameOffsetsCopy_.at(frameName);
  }
  catch(...) {
    return false;
  }

  return true;
}

bool ExternalReferenceTrackingModule::getSensorFrameOffset(const std::string &frameName, confusion::Pose<double> &T_body_sensor) const {
  std::lock_guard<std::mutex> lg(parameterQueryMtx_);
  try {
    T_body_sensor = sensorFrameOffsetsCopy_.at(frameName);
  }
  catch(...) {
    return false;
  }

  return true;
}

bool ExternalReferenceTrackingModule::getSensorScale(const std::string &frameName, double &scale) const {
  std::lock_guard<std::mutex> lg(parameterQueryMtx_);
  try {
    scale = sensorPoseMeasScalesCopy_.at(frameName);
  }
  catch(...) {
    return false;
  }

  return true;
}

void ExternalReferenceTrackingModule::setExtrinsicsUnconstForBatchSolve() {
  for (auto &sensorFrame : sensorFrameOffsets_) {
    sensorFrame.second->print(sensorFrame.first);
    conFusor_.staticParameters_.getParameter(sensorFrame.second->trans.data())->unsetConstant();
    conFusor_.staticParameters_.getParameter(sensorFrame.second->rot.coeffs().data())->unsetConstant();
  }
}
void ExternalReferenceTrackingModule::resetExtrinsicsConstAfterBatchSolve() {
  for (auto &sensorFrame : sensorFrameOffsets_) {
    std::shared_ptr<confusion::PoseMeasConfig> poseMeasConfigPtr = sensorPoseMeasConfigs_.at(sensorFrame.first);
    if (!poseMeasConfigPtr->optTranslationalExtrinsic)
      conFusor_.staticParameters_.getParameter(sensorFrame.second->trans.data())->setConstant();
    if (!poseMeasConfigPtr->optRotationalExtrinsic)
      conFusor_.staticParameters_.getParameter(sensorFrame.second->rot.coeffs().data())->setConstant();
  }
}

}  // namespace confusion
