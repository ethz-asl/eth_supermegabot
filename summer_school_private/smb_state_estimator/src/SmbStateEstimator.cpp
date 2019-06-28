//
// Created by tim on 07.10.18.
//

#include <smb_state_estimator/SmbStateEstimator.h>


namespace smb_state_estimator {

SmbStateEstimator::SmbStateEstimator(any_node::Node::NodeHandlePtr nh) :
    any_state_estimator::AnyStateEstimator<smb_description_ros::SmbContainersRos>(nh) {
  stateEstimateSubscriber_ = nh->subscribe("/pose_est_out", 10, &SmbStateEstimator::stateEstimateCallback, this);
  imuSubscriber_ = nh->subscribe("/imu0", 10, &SmbStateEstimator::imuCallback, this);
  basePosePublisher_ = nh->advertise<geometry_msgs::PoseStamped>("/base_pose_measured", 10);

  //todo Subscribe to IMU measurements from VI sensor

  smb_description_ros::initializeSmbState(smbMsgRos_);
}

void SmbStateEstimator::readParameters() {
  //Read in the IMU to base offset
//  T_i_b_.trans(0) = param<double>("T_i_b/x", 0.0);
//  T_i_b_.trans(1) = param<double>("T_i_b/y", 0.0);
//  T_i_b_.trans(2) = param<double>("T_i_b/z", 0.0);
//  T_i_b_.rot.w() = param<double>("T_i_b/qw", 1.0);
//  T_i_b_.rot.x() = param<double>("T_i_b/qx", 0.0);
//  T_i_b_.rot.y() = param<double>("T_i_b/qy", 0.0);
//  T_i_b_.rot.z() = param<double>("T_i_b/qz", 0.0);
//
//  stateEstimatorDropoutTime_ = param<double>("stateEstimatorDropoutTime", 2.0);

  MELO_INFO("[SmbStateEstimator] Ready to start...\n");

  // Read parameters from the smb_confusor config file
  std::string packagePath = ros::package::getPath("smb_confusor");
  configFile_ = packagePath + configFile_;
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(configFile_, pt);

  T_i_b_.trans(0) = pt.get<double>("T_imu_base.px");
  T_i_b_.trans(1) = pt.get<double>("T_imu_base.py");
  T_i_b_.trans(2) = pt.get<double>("T_imu_base.pz");
  T_i_b_.rot.w() = pt.get<double>("T_imu_base.qw");
  T_i_b_.rot.x() = pt.get<double>("T_imu_base.qx");
  T_i_b_.rot.y() = pt.get<double>("T_imu_base.qy");
  T_i_b_.rot.z() = pt.get<double>("T_imu_base.qz");
  T_i_b_.rot.normalize();

  stateEstimatorDropoutTime_ = pt.get<double>("stateEstimatorDropoutTime");
  int forwardPropagationSensor = pt.get<int>("forwardPropagationSensor");
}

void SmbStateEstimator::receiveMeasurements() {
  //Get the current time
  currentTime_ = any_measurements_ros::fromRos(ros::Time::now());
//MELO_INFO("[SmbStateEstimator] t=%10.10f", currentTime_.toSeconds());

//  //Get new IMU measurements
//  if (imuConnected_) {
//    //Get and process IMU measurements
//    if (imu_->getMeasurements(imuMutexTimeout, maxImuMeas_, imuMeasurements_)) {
////MELO_INFO("[SmbStateEstimator] received %lu IMU measurements between %f and %f", imuMeasurements_.size(), imuMeasurements_.front().t, imuMeasurements_.back().t);
//      //Copy the measurements over to the Propagator
//      while (!imuMeasurements_.empty()) {
//        //todo Use Confusion ImuMeas type in xsens_driver_adrl so I don't have to make a copy?
//        confusion::ImuMeas imuMeas(imuMeasurements_.front().t, imuMeasurements_.front().a, imuMeasurements_.front().w);
//        imuPropagator_.addImuMeasurement(imuMeas);
//        imuMeasurements_.pop_front();
//      }
//    }
//
//    if (!imuMeasurements_.empty())
//      MELO_INFO("[SmbStateEstimator] IMU measurement buffer isn't empty after processing???-----------");
//  }

  //Get new actuator measurements
  if (!this->actuatorReadingsSubscriber_->receive(this->receiveMaxLockTime_)) {
    //todo Count and report how many are dropped per X seconds
    //MELO_WARN("[SmbStateEstimator] receive actuator readings returned false??");
  }
}

void SmbStateEstimator::preprocessMeasurements() {
  //todo For Anymal, the actuator readings are copied over into another bin here
}

void SmbStateEstimator::advanceEstimator() {
  if (stateEstimateReceived_) {
    //Forward propagate latest estimate up to the current time
    imuState_now_ = imuPropagator_.propagate(currentTime_.toSeconds());
  }
}

void SmbStateEstimator::setOutput() {
  estimatedState_.time_ = currentTime_;

  //Identify state estimator dropouts
  if (currentTime_.toSeconds() - lastStateEstimatorTime_ > stateEstimatorDropoutTime_) {
    stateEstimateValid_ = false;
  }

  //Indicate if the state estimator is valid
  if (stateEstimateValid_)
    estimatedState_.status_ = smb_description::StateStatus::STATUS_OK;
  else
    estimatedState_.status_ = smb_description::StateStatus::STATUS_ERROR_ESTIMATOR;

  auto smbActuatorReadings = actuatorReadings_.wheelReadings_;
  constexpr int numOfWheels = SmbDescription::ConcreteDefinitions::getNumWheels();

  //Fill in robot state
  Eigen::Matrix<double, 6, 1> q;
  for (const auto actuatorKey : SmbDescription::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    const auto actuatorId = actuatorKey.getId();

    if (smb_description::SmbTopology::mapSmbActuatorEnumToActuatorEnum::containsValue(actuatorEnum)) {
      const auto smbActuatorEnum = smb_description::SmbTopology::mapSmbActuatorEnumToActuatorEnum::find(actuatorEnum);

      estimatedState_.smbState_.getWheelVelocities()(actuatorId) = smbActuatorReadings[smbActuatorEnum].getWheelVelocity();
      estimatedState_.smbState_.getWheelTorques()(actuatorId) = smbActuatorReadings[smbActuatorEnum].getWheelTorque();
    }
    else {
      MELO_WARN("[SmbStateEstimator] Unknown/invalid actuatorEnum ");
    }
  }

  //Add the wheel speed measurement and forward-propagate using the BMM
  //todo This is being done at a higher rate than the wheel speeds are actually being read from the motor controller
  bmmPropagator_.addMeasurement(WheelSpeeds(currentTime_.toSeconds(),
                                            estimatedState_.smbState_.getWheelVelocities()(0),
                                            estimatedState_.smbState_.getWheelVelocities()(1),
                                            bmmCalibration_,
                                            T_i_b_, 0));
  if (stateEstimateReceived_) {
    //Forward propagate latest estimate up to the current time
    baseState_now_ = bmmPropagator_.propagate(currentTime_.toSeconds());
  }

  //Base pose
  confusion::Pose<double> T_w_b = imuState_now_.T_w_i_ * T_i_b_;
  kindr::HomTransformQuatD kindrBasePose(smb_model::Position(T_w_b.trans), smb_model::RotationQuaternion(T_w_b.rot));
  estimatedState_.smbState_.setPoseBaseToWorld(kindrBasePose);

  //Base velocity
  Eigen::Vector3d w_trans_b_i = T_w_b.trans - imuState_now_.T_w_i_.trans;
  Eigen::Matrix<double,6,6> VT_b_i = confusion::rigidBodyVelocityTransform(w_trans_b_i);
  Eigen::Matrix<double,6,1> w_vel_w_i;
  w_vel_w_i.head<3>() = imuState_now_.angVel_;
  w_vel_w_i.tail<3>() = imuState_now_.linVel_;
  Eigen::Matrix<double,6,1> w_vel_w_b = VT_b_i * w_vel_w_i;

  estimatedState_.smbState_.setAngularVelocityBaseInBaseFrame(T_w_b.rot.conjugate() * w_vel_w_b.head<3>());
  estimatedState_.smbState_.setLinearVelocityBaseInWorldFrame(smb_model::LinearVelocity(w_vel_w_b.tail<3>()));

//  const kindr::Velocity3D B_v_B(robotBaseLinkLinearVelocity_[0],
//                                robotBaseLinkLinearVelocity_[1],
//                                robotBaseLinkLinearVelocity_[2]);
//
//  const kindr::LocalAngularVelocityPD B_w_IB(robotBaseLinkAngularVelocity_[0],
//                                             robotBaseLinkAngularVelocity_[1],
//                                             robotBaseLinkAngularVelocity_[2]);
//
//
//  //-- Generalized positions
//  estimatedState_.smbState_.setPositionWorldToBaseInWorldFrame(
//      smb_model::Position(robotBaseLinkPose_.pos[0] - frameOdometryOffsetX_,
//                           robotBaseLinkPose_.pos[1] - frameOdometryOffsetY_,
//                           robotBaseLinkPose_.pos[2] - frameOdometryOffsetZ_));
//  estimatedState_.smbState_.setOrientationBaseToWorld(orientationBaseToWorld);
//  //--
//
//  //-- Generalized velocities
//  estimatedState_.smbState_.setLinearVelocityBaseInWorldFrame(orientationBaseToWorld.rotate(B_v_B));
//  estimatedState_.smbState_.setAngularVelocityBaseInBaseFrame(B_w_IB);

  //Store the base pose for publishing to the navigation planning/controller
  {
    std::lock_guard<std::mutex> lock(publishRosMtx_);
    _T_w_i_ = imuState_now_.T_w_i_;
    _T_w_b_ = T_w_b;
    _T_w_b_odom_ = baseState_now_.T_w_b;
  }
}

void SmbStateEstimator::publish() {
  //todo Publish IMU messages here?
//  if (!this->imuPublisher_->publish(this->imu_, this->sendMaxLockTime_)) {
//    ++imuPublisherMissCount_;
//  }
//
//  if (!imuPublisherThrottled_->publish(this->imu_, any_measurements_ros::toRos(this->imu_), this->sendMaxLockTime_)) {
//    ++imuThrottledMissCount_;
//  }

//  if (!actuatorReadingsPublisher_->publish(this->actuatorReadings_, this->sendMaxLockTime_)) {
//    ++actuatorReadingsPublisherMissCount_;
//  }
//

  //Copy the state to rosmsg
  smb_description_ros::ConversionTraits<smb_description::SmbState, smb_msgs::SmbState>::convert(estimatedState_, smbMsgRos_);

  robotStatePublisher_->publish(estimatedState_, smbMsgRos_, sendMaxLockTime_);
}

void SmbStateEstimator::stateEstimateCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  //todo Make this general to easily support different state estimators?
  if (msg->data.size() != 17) {
    MELO_WARN("[SmbStateEstimator] State estimate message received with the wrong size. Expected size=%d, but got size=%lud", 21, msg->data.size());
    return;
  }

  confusion::ImuStateParameters imuStateUpdate;
  imuStateUpdate.t_ = msg->data[0];
  imuStateUpdate.T_w_i_ = confusion::Pose<double>(msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5],msg->data[6],msg->data[7]);
  imuStateUpdate.T_w_i_.rot.normalize();
  imuStateUpdate.linVel_ << msg->data[8],msg->data[9],msg->data[10];
  imuStateUpdate.accelBias_ << msg->data[11],msg->data[12],msg->data[13];
  imuStateUpdate.gyroBias_ << msg->data[14],msg->data[15],msg->data[16];

  stateEstimateValid_ = true;
  lastStateEstimatorTime_ = imuStateUpdate.t_;

//#ifdef IMU_PROP_DEBUG
  MELO_INFO("Update received for t=%10.10f; delay=%f\n", imuStateUpdate.t_, ros::Time::now().toSec()-imuStateUpdate.t_);
//#endif

  //Do some sanity checks
  if (imuPropagator_.isBufferEmpty())
    MELO_WARN("[SmbStateEstimator] imuPropagator is empty when applying a state update!?");

  if (imuStateUpdate.t_ < imuPropagator_.getFrontMeasurement().t() || imuStateUpdate.t_ > imuPropagator_.getBackMeasurement().t())
    MELO_WARN("[SmbStateEstimator] Time mismatch in state update! t_state=%10.10f, t_imu_back=%10.10f, t_imu_front=%10.10f\n",
              imuStateUpdate.t_, imuPropagator_.getBackMeasurement().t(), imuPropagator_.getFrontMeasurement().t());

  imuPropagator_.update(imuStateUpdate);

  //Also update the estimate in the BMM propagator
  BaseMotionModelParameters bmmParameters;
  bmmParameters.t = msg->data[0];
  bmmParameters.T_w_b = imuStateUpdate.T_w_i_ * T_i_b_;
  bmmPropagator_.update(bmmParameters);

  stateEstimateReceived_ = true;
}


void SmbStateEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg) {
  confusion::ImuMeas imuMeas = confusion::rosmsgToConfusionImuMeas(*imuMsg);
  imuPropagator_.addImuMeasurement(imuMeas);

#ifdef SYNC_DEBUG
  //std::cout << "Imu meas received with stamp " << t_imu_source << std::endl;
#endif
}


void SmbStateEstimator::publishRos() {
  confusion::Pose<double> T_w_b;
  confusion::Pose<double> T_w_i;
  confusion::Pose<double> T_w_b_odom;
  {
    std::lock_guard<std::mutex> lock(publishRosMtx_);
    T_w_b = _T_w_b_;
    T_w_i = _T_w_i_;
    T_w_b_odom = _T_w_b_odom_;
  }

  if (stateEstimateReceived_) {
    basePosePublisher_.publish(getMsg(T_w_b,"/world"));

    //Send imu/world tf for ICP
    tfBroadcaster_.sendTransform(tf::StampedTransform(getTfMsg(T_w_i), ros::Time::now(), "world", "imu"));
    tfBroadcaster_.sendTransform(tf::StampedTransform(getTfMsg(T_w_b_odom), ros::Time::now(), "world", "base_odom"));
  }

  //todo Quick hack to throttle the state estimator publishing for Rhino
  static int statePubThrottleCounter = 0;
  if (statePubThrottleCounter > 40) {
    robotStatePublisher_->sendRos();
    statePubThrottleCounter = 0;
  }
  else
    ++statePubThrottleCounter;
}

} // namespace smb_state_estimator
