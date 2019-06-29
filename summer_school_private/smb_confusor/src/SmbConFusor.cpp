//
// Created by tim on 16.11.18.
//

#include "smb_confusor/SmbConFusor.h"


SmbConFusor::SmbConFusor(ros::NodeHandle &node):node_(node),
                                                  conFusor_(std::make_shared<SmbState>(stateInitializationSensor_)),
                                                  statesBatch_(NUM_PROCESS_SENSORS, NUM_UPDATE_SENSORS),
                                                  imuPropagator_(5000) {
  // Load the configuration from text file
  std::string packagePath = ros::package::getPath("smb_confusor");
  configFile = packagePath + configFile;
  boost::property_tree::read_info(configFile, pt);

  //This allows a launch file to override the value in the config file from a launch file, so remove the need to always swtich the config files depending on which mode the estimator is run in
  forwardPropagateState_ = pt.get<bool>("forwardPropagateState");
  node.param<bool>("forward_propagate_state_override", forwardPropagateState_, forwardPropagateState_);
  std::cout << "[SmbConfusor] forwardPropagateState = " << forwardPropagateState_ << std::endl;


  useImu_ = pt.get<bool>("useImu");
  useBmm_ = pt.get<bool>("useBmm");
  useTags_ = pt.get<bool>("useTags");
  useLidar_ = pt.get<bool>("useLidar");

  // If the IMU is not used, but the BMM is, use the BMM for state initialiation
  if (!useImu_ && useBmm_) {
    stateInitializationSensor_ = BMM;
  }

  forwardPropagationSensor_ = pt.get<int>("forwardPropagationSensor");
  if ((forwardPropagationSensor_ == IMU && !useImu_) ||
      (forwardPropagationSensor_ == BMM && !useBmm_) ) {
    std::cout << "ERROR: forwardPropagationSensor is set to a sensor that is not being used!" << std::endl;
    abort();
  }

  auto firstSmbState = std::dynamic_pointer_cast<SmbState>(conFusor_.stateVector_.front());

  // Set up module for tracking relative to stationary fiducials
  if (useTags_) {
    aprilTagInterface_ = std::unique_ptr<confusion::AprilTagModule>(new confusion::AprilTagModule(
        node, &conFusor_, configFile, TAG)); //&newUpdateMeasReady_)); Not stopping optimizations for new tag measurements for now!
#ifdef SYNC_DEBUG
    aprilTagInterface_->setVerbose(true);
#endif

    firstSmbState->setTagReferenceFrameOffsets(aprilTagInterface_->getTagReferenceFrameOffsetsMapPointer());
  }

  // Set up external reference tracking for incorporating ICP updates
  if (useLidar_) {
    ertModule_ = std::unique_ptr<confusion::ExternalReferenceTrackingModule>(
        new confusion::ExternalReferenceTrackingModule(
            node, conFusor_, configFile, POSEMEAS, &newUpdateMeasReady_));
    firstSmbState->setExternalReferenceFrameOffsets(ertModule_->getReferenceFrameOffsetsMapPointer());
  }

  // Set up the logger
  std::string logFileName = packagePath + "/data/smb_confusor_log.txt";
  logger_ = std::unique_ptr<confusion::Logger>(new confusion::Logger(logFileName, *conFusor_.stateVector_.front()));
  logData_ = pt.get<bool>("logData");

  conFusor_.setSolverOptions(pt.get<int>("numThreads"), pt.get<int>("maxNumIterations"));
  batchSize_ = pt.get<int>("batchSize");
  runBatch_ = pt.get<bool>("runBatch");

  //Set the initial constraint on the state
  std::vector<Eigen::MatrixXd> stateParamInitialWeightings(conFusor_.stateVector_.front()->parameters_.size());
  Eigen::MatrixXd param_weighting(3, 3);
  param_weighting.setIdentity();
  param_weighting /= pt.get<double>("twi_init_stddev");
  stateParamInitialWeightings[0] = param_weighting;

  param_weighting.setIdentity();
  param_weighting /= pt.get<double>("qwi_init_stddev");
  stateParamInitialWeightings[1] = param_weighting;

  param_weighting.setIdentity();
  param_weighting /= pt.get<double>("vwi_init_stddev");
  stateParamInitialWeightings[2] = param_weighting;

  param_weighting.setIdentity();
  param_weighting /= pt.get<double>("ba_init_stddev");
  stateParamInitialWeightings[3] = param_weighting;

  param_weighting.setIdentity();
  param_weighting /= pt.get<double>("bg_init_stddev");
  stateParamInitialWeightings[4] = param_weighting;

  conFusor_.stateVector_.front()->setInitialStateWeighting(stateParamInitialWeightings);

  // Setup the IMU
  imuCalibration_.gravityMagnitude_ = pt.get<double>("gravityMagnitude");
  double wi_stddev_ = pt.get<double>("wi_stddev");
  double ai_stddev_ = pt.get<double>("ai_stddev");
  double bg_stddev_ = pt.get<double>("bg_stddev");
  double ba_stddev_ = pt.get<double>("ba_stddev");

  imuCalibration_.cov_imu_nominal_.setIdentity();
  imuCalibration_.cov_imu_nominal_.block<3, 3>(0, 0) *= wi_stddev_ * wi_stddev_;
  imuCalibration_.cov_imu_nominal_.block<3, 3>(3, 3) *= ai_stddev_ * ai_stddev_;
  imuCalibration_.cov_imu_nominal_.block<3, 3>(6, 6) *= bg_stddev_ * bg_stddev_;
  imuCalibration_.cov_imu_nominal_.block<3, 3>(9, 9) *= ba_stddev_ * ba_stddev_;

  imuCalibration_.g_w_ << 0.0, 0.0, imuCalibration_.gravityMagnitude_;

  gravity_rot_.setZero();
#ifdef OPT_GRAVITY
  //	Eigen::MatrixXd gravityRotInitialWeighting(2,2);
  //	gravityRotInitialWeighting.setIdentity();
  //	gravityRotInitialWeighting /= pt.get<double>("ftr_init_stddev");
      conFusor_.addStaticParameter(confusion::Parameter(gravity_rot_.data(), 2, "g_r")); //, gravityRotInitialWeighting);
#endif

  if (useImu_)
    subImu_ = node_.subscribe(pt.get<std::string>("imu_topic"), 1000, &SmbConFusor::imuCallback,
                            this, ros::TransportHints().tcpNoDelay());

  //Set the config parameters for BMM
  T_imu_base_.trans(0) = pt.get<double>("T_imu_base.px");
  T_imu_base_.trans(1) = pt.get<double>("T_imu_base.py");
  T_imu_base_.trans(2) = pt.get<double>("T_imu_base.pz");
  T_imu_base_.rot.w() = pt.get<double>("T_imu_base.qw");
  T_imu_base_.rot.x() = pt.get<double>("T_imu_base.qx");
  T_imu_base_.rot.y() = pt.get<double>("T_imu_base.qy");
  T_imu_base_.rot.z() = pt.get<double>("T_imu_base.qz");
  T_imu_base_.rot.normalize();

  baseMotionModelCalibration_.useLossFunction_ = pt.get<bool>("bmmUseLossFunction");
  baseMotionModelCalibration_.lossCoefficient_ = pt.get<double>("bmmLossCoefficient");
  double bmm_cov_stationary_actuated = pt.get<double>("bmm_cov_stationary_actuated");
  double bmm_cov_stationary_unactuated = pt.get<double>("bmm_cov_stationary_unactuated");
  double bmm_cov_moving_actuated = pt.get<double>("bmm_cov_moving_actuated");
  double bmm_cov_moving_unactuated = pt.get<double>("bmm_cov_moving_unactuated");

  baseMotionModelCalibration_.cov_Twb_stationary.setIdentity();
  baseMotionModelCalibration_.cov_Twb_stationary *= bmm_cov_stationary_unactuated;
  baseMotionModelCalibration_.cov_Twb_stationary(2,2) = bmm_cov_stationary_actuated;
  baseMotionModelCalibration_.cov_Twb_stationary(3,3) = bmm_cov_stationary_actuated;

  baseMotionModelCalibration_.cov_Twb_moving.setIdentity();
  baseMotionModelCalibration_.cov_Twb_moving *= bmm_cov_moving_unactuated;
  baseMotionModelCalibration_.cov_Twb_moving(2,2) = bmm_cov_moving_actuated;
  baseMotionModelCalibration_.cov_Twb_moving(3,3) = bmm_cov_moving_actuated;
//std::cout << "cov_stationary:\n" << baseMotionModelCalibration_.cov_Twb_stationary << "\n moving:\n" << baseMotionModelCalibration_.cov_Twb_moving << std::endl;

  baseMotionModelCalibration_.movingWheelSpeedThld_ = pt.get<double>("movingWheelSpeedThld");
  baseMotionModelCalibration_.wheelRadius_ = pt.get<double>("wheelRadius");
  baseMotionModelCalibration_.wheelbase_ = pt.get<double>("wheelbase");

  if (useBmm_)
    baseOdometrySub_ = node.subscribe("/wheelSpeeds", 10,
                                    &SmbConFusor::baseOdometryCallback, this);

  bmmPub_ = node.advertise<geometry_msgs::PoseStamped>("/bmm_pose", 10);

#ifdef OPT_GRAVITY
  //Force the world to map offset to identity
  auto T_world_map = std::make_shared<confusion::Pose<double>>(); //Set to identity
  tagReferenceFrameOffsets_["/map"] = T_world_map;

  conFusor_.addStaticParameter(confusion::Parameter(
                  T_world_map->trans.data(), 3, "t_world_map", true));
  conFusor_.addStaticParameter(confusion::Parameter(
                  T_world_map->rot.coeffs().data(), 4, "q_world_map", true,
                  std::make_shared<confusion::QuatParam>()));
#endif

  //Set up publishers and subscribers
  auto callbackPtr = std::make_shared<confusion::SolverOverrunCallback>(newUpdateMeasReady_);
  conFusor_.setIterationCallback(callbackPtr);

  pubStates_ = node_.advertise<geometry_msgs::PoseArray>("/states", 10);
  pubState_ = node_.advertise<std_msgs::Float64MultiArray>("/pose_est_out", 10);
  pubPose_ = node_.advertise<geometry_msgs::PoseStamped>("/T_w_i_opt", 10);
  pubRtPose_ = node_.advertise<geometry_msgs::PoseStamped>("/T_w_i", 10);

  //Listen for other user triggers
  subTriggerBatch_ = node_.subscribe("/trigger_batch", 1, &SmbConFusor::triggerBatchCalCallback, this);
  subDrawDiagram_ = node_.subscribe("/draw_confusion_diagram", 1, &SmbConFusor::drawDiagramCallback, this);

  //todo Temp for debugging
  pubLidarPose_ = node_.advertise<geometry_msgs::PoseStamped>("/lidarPoseRaw", 10);
  pubRawImuPose_ = node_.advertise<geometry_msgs::PoseStamped>("/imuPoseRaw", 10);

  // Start the estimator on a separate thread
  estimatorThread_ = std::thread(&SmbConFusor::runEstimator, this);
  estimatorThread_.detach();

  std::cout << "[SmbConFusor] Initialization complete." << std::endl;
}

void SmbConFusor::imuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg) {
  if (!run_)
    return;

//  if (firstMeasurement_) {
//    t_epoch_ = imuMsg->header.stamp.toSec();
//    firstMeasurement_ = false;
//  }

  double t_imu_source = imuMsg->header.stamp.toSec();
  if (t_imu_source <= t_imu_latest_)
    std::cout << "IMU timestamps out of sequence!!! t_meas=" << t_imu_source << ", t_last=" << t_imu_latest_ << std::endl;
  t_imu_latest_ = t_imu_source;

  Eigen::Vector3d a(imuMsg->linear_acceleration.x,
                    imuMsg->linear_acceleration.y,
                    imuMsg->linear_acceleration.z);
  Eigen::Vector3d w(imuMsg->angular_velocity.x,
                    imuMsg->angular_velocity.y,
                    imuMsg->angular_velocity.z);

  conFusor_.addProcessMeasurement(std::make_shared<confusion::ImuMeas>(
      t_imu_source, a, w, &imuCalibration_, &gravity_rot_));

  //Optionally publish the forward-propagated state at IMU rate
  if (forwardPropagateState_ && forwardPropagationSensor_ == IMU) {
    confusion::ImuMeas meas(t_imu_source, a, w, &imuCalibration_, &gravity_rot_);
    imuPropagator_.addImuMeasurement(meas);

    if (tracking_) {
      confusion::ImuStateParameters state = imuPropagator_.propagate(t_imu_source);
      pubRtPose_.publish(getMsg(state.T_w_i_, "/world", ros::Time(state.t())));
      tfBroadcaster_.sendTransform(tf::StampedTransform(getTfMsg(state.T_w_i_), ros::Time::now(), "world", "imu"));
    }
  }

  // Publish tfs at high rate for Lidar SLAM
  ros::Time t_now = ros::Time::now();
  tfBroadcaster_.sendTransform(tf::StampedTransform(getTfMsg(T_imu_base_), t_now, "imu", "base"));

  if (ertModule_)
    ertModule_->publishFrameOffsetTfs();

  // Also publish a static offset from body to imu
  tfBroadcaster_.sendTransform(tf::StampedTransform(getTfMsg(confusion::Pose<double>()), t_now, "imu", "body"));

#ifdef SYNC_DEBUG
  std::cout << "Imu meas received with stamp " << t_imu_source << ". Delayed by: " << ros::Time::now().toSec() - t_imu_source << std::endl;
#endif
}

//Message data is [t, leftSpeed, rightSpeed] with speeds in rad/sec
void SmbConFusor::baseOdometryCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
//  if (firstMeasurement_) {
//    std::cout << "First meas is bmm at t=" << msg->data[0] << std::endl;
//    t_epoch_ = msg->data[0];
//    firstMeasurement_ = false;
//  }

  double t_meas = msg->data[0];
  if (t_meas <= t_bmm_latest_)
    std::cout << "BMM timestamps out of sequence!!! t_meas=" << t_meas << ", t_last=" << t_bmm_latest_ << std::endl;
  t_bmm_latest_ = t_meas;

  auto wheelSpeedMeasurement = std::make_shared<WheelSpeeds>(
      t_meas,
      msg->data[1],
      msg->data[2],
      baseMotionModelCalibration_,
//      *T_armbase_imu_ptr_,
      T_imu_base_,
      BMM);

  if (useBmm_) {
    conFusor_.addProcessMeasurement(wheelSpeedMeasurement);
  }

//  baseMotionModel_.measurements_.push_back(wheelSpeedMeasurement);
  if (!bmmMeasReceived_) {
    bmmMeasReceived_ = true;
  }

  if (forwardPropagateState_ && forwardPropagationSensor_ == BMM) {
    bmmPropagator_.addMeasurement(*wheelSpeedMeasurement);

    if (tracking_) {
      BaseMotionModelParameters bmmParams = bmmPropagator_.propagate(t_meas);
      confusion::Pose<double> T_w_i = bmmParams.T_w_b * T_imu_base_.inverse();
      tfBroadcaster_.sendTransform(tf::StampedTransform(getTfMsg(T_w_i), ros::Time::now(), "world", "imu"));
      pubRtPose_.publish(getMsg(T_w_i, "/world"));
    }
  }

  // Publish tfs at high rate for Lidar SLAM
  ros::Time t_now = ros::Time::now();
  tfBroadcaster_.sendTransform(tf::StampedTransform(getTfMsg(T_imu_base_), t_now, "imu", "base"));

  if (ertModule_)
    ertModule_->publishFrameOffsetTfs();

  // Also publish a static offset from body to imu
  tfBroadcaster_.sendTransform(tf::StampedTransform(getTfMsg(confusion::Pose<double>()), t_now, "imu", "body"));

#ifdef SYNC_DEBUG
  std::cout << "Added wheel speed measurement for t=" << t_meas <<
       ". leftWheelSpd=" << msg->data[1] << ", rightWheelSpd=" << msg->data[2] << std::endl;
#endif
}

void SmbConFusor::runEstimator() {
  ros::Rate loop_rate(loop_freq);

  while (run_) {
    if (!ros::ok()) {
      run_ = false;
      tracking_ = false;
      break;
    }

    ros::Time t_start = ros::Time::now();

    // Only optimize if a state linked to a lidar measurement is received. This
    // prevents that we try to optimize for each image AND lidar measurement that
    // arrives
    bool newLidarStateCreated = false;
    if (conFusor_.assignMeasurements()) {
      auto stateIter = conFusor_.stateVector_.begin();
      while (stateIter != conFusor_.stateVector_.end() && (*stateIter)->t() <= t_latest_state_)
        ++stateIter;

      if (useLidar_) { //todo Better handle case when lidar isn't active
        while (stateIter != conFusor_.stateVector_.end()) {
          if (!(*stateIter)->updateMeasurements_[POSEMEAS].empty()) {
            newLidarStateCreated = true;
            break;
          }
          ++stateIter;
        }
      }
      else
        newLidarStateCreated = true;
    }

    if (!newLidarStateCreated) {
      loop_rate.sleep();
      continue; //No new states or measurements added, so wait for more measurements
    }
    newUpdateMeasReady_ = false;

    t_latest_state_ = conFusor_.stateVector_.back()->t();

    // Manage the batch size. If there are too many states currently active, remove
    // the oldest state(s).
    if (conFusor_.numStates() > batchSize_) {
      if (conFusor_.numStates() - batchSize_ > 1)
        std::cout << "WARNING: Had to marginalize " << conFusor_.numStates() - batchSize_ <<
                  " states in one iteration. The estimator is not running fast enough." << std::endl;

      stateToDropIndex_ -= conFusor_.numStates() - batchSize_;

      //Optionally store the marginalized states to run a batch prboblem later
      if (runBatch_)
        conFusor_.marginalizeFrontStates(conFusor_.numStates() - batchSize_, &statesBatch_);
      else
        conFusor_.marginalizeFrontStates(conFusor_.numStates() - batchSize_);
    }

    std::cout << "States at: ";
    for (auto &state: conFusor_.stateVector_)
      std::cout << state->t() << ", ";
    std::cout << std::endl;

    // Solve the MHE problem for the active batch of states
    conFusor_.optimize();
    conFusor_.briefPrint();

    tracking_ = true;

    // You can optionally draw a diagram of the current MHE problem structure
    if (drawDiagramRequest_) {
      confusion::Diagram diagram(conFusor_);
      drawDiagramRequest_ = false;
    }

    ros::Time t_end = ros::Time::now();
    double t_iter = (t_end - t_start).toSec();

    std::cout << "Optimized state delayed by " << ros::Time::now().toSec() - conFusor_.stateVector_.back()->t() <<
        " sec. Solver computation time: " << t_iter << " sec" << std::endl;
//    std::cout << "SmbConFusor optimization took " << t_iter << " sec\n" << std::endl;

    publish(conFusor_.stateVector_.back(), &(conFusor_.stateVector_));

    if (logData_) {
      Eigen::VectorXd priorResidual;
      std::vector<Eigen::VectorXd> processResiduals;
      std::vector<Eigen::VectorXd> updateResiduals;
      conFusor_.getResiduals(priorResidual, processResiduals, updateResiduals);

      logger_->writeBatch(conFusor_.stateVector_);
      logger_->writeStaticParameters(conFusor_.staticParameters_, conFusor_.stateVector_.back()->t_);
      logger_->writeResiduals(priorResidual, processResiduals, updateResiduals);
    }

    loop_rate.sleep();
  }

  runEstimatorLoopDone_ = true;
}


void SmbConFusor::stopTracking() {
  std::cout << "Stopping tracking" << std::endl;

  //Stop the estimator and wait for it to exit
  run_ = false;
  tracking_ = false;
  while (ros::ok() && !runEstimatorLoopDone_)
    ros::Rate(10).sleep();

  //Stop tracking so that it can be started with the optimized parameters
  conFusor_.stopTracking(&statesBatch_);

  runEstimatorLoopDone_ = false;
}


void SmbConFusor::startTracking() {
  if (run_) {
    std::cout << "TagTracker is already running." << std::endl;
    return;
  }

  stateToDropIndex_ = -1;

  std::cout << "Starting tracking" << std::endl;
  run_ = true;
  estimatorThread_ = std::thread(&SmbConFusor::runEstimator, this);
  estimatorThread_.detach();
}


void SmbConFusor::stopTracking(const std_msgs::Empty &msg) {
  stopTracking();
}


void SmbConFusor::startTracking(const std_msgs::Empty &msg) {
  startTracking();
}


void SmbConFusor::drawDiagramCallback(const std_msgs::Empty &msg) {
  std::cout << "User requested for a MHE diagram to be drawn." << std::endl;
  drawDiagramRequest_ = true;
}


void SmbConFusor::triggerBatchCalCallback(const std_msgs::Empty &msg) {
  if (!runBatch_) {
    std::cout << "WARNING: User requested a batch calibration run even runBatch is not set in "
                 "the config file. Cannot perform the batch calibration." << std::endl;
    return;
  }

  if (run_)
    stopTracking();

  std::cout << "Starting a batch calibration run for " << statesBatch_.size() << " states" << std::endl;

  //Optimize the extrinsic calibrations
  std::shared_ptr<confusion::Pose<double>> T_c_i_ptr;
  if (aprilTagInterface_) {
    T_c_i_ptr = aprilTagInterface_->sensorFrameOffsets_["cam"];
    conFusor_.staticParameters_.getParameter(T_c_i_ptr->trans.data())->unsetConstant();
    conFusor_.staticParameters_.getParameter(T_c_i_ptr->rot.coeffs().data())->unsetConstant();
  }
  if (ertModule_)
    ertModule_->setExtrinsicsUnconstForBatchSolve();

  publish(statesBatch_.back(), &statesBatch_);

  confusion::BatchFusor batchFusor(statesBatch_, conFusor_.staticParameters_);
  batchFusor.buildProblem();

  Eigen::VectorXd priorResidual;
  priorResidual.resize(0);
  std::vector<Eigen::VectorXd> processResiduals;
  std::vector<Eigen::VectorXd> updateResiduals;
  batchFusor.getResiduals(processResiduals, updateResiduals);

  batchFusor.optimize();

  publish(statesBatch_.back(), &statesBatch_);

  processResiduals.clear();
  updateResiduals.clear();
  batchFusor.getResiduals(processResiduals, updateResiduals);

  if (logData_) {
    logger_->writeBatch(batchFusor.stateVector_);
    logger_->writeStaticParameters(conFusor_.staticParameters_, statesBatch_.back()->t_);
    logger_->writeResiduals(priorResidual, processResiduals, updateResiduals);
  }

  //Reset the status of the extrinsic calibration parameters
  if (aprilTagInterface_) {
    batchFusor.printParameterCovariance(T_c_i_ptr->trans.data(), "t_c_i");
    batchFusor.printParameterCovariance(T_c_i_ptr->rot.coeffs().data(), "q_c_i");

    T_c_i_ptr->print("T_c_i");
    if (!pt.get<bool>("optimizeTci")) {
      conFusor_.staticParameters_.getParameter(T_c_i_ptr->trans.data())->setConstant();
      conFusor_.staticParameters_.getParameter(T_c_i_ptr->rot.coeffs().data())->setConstant();
    }
  }
  if (ertModule_)
    ertModule_->resetExtrinsicsConstAfterBatchSolve();

  //Empty the batch states to start logging for another batch problem
  statesBatch_.reset();

  //Now restart the estimator
  startTracking();
}


void SmbConFusor::publish(std::shared_ptr<confusion::State> statePtr, const confusion::StateVector *stateVector) {
  //Publish the most recent state for use in SL
  auto state = std::dynamic_pointer_cast<SmbState>(statePtr);

  if (forwardPropagateState_ && forwardPropagationSensor_ == IMU) {
    imuPropagator_.update(state->getParameterStruct());
  }

  std_msgs::Float64MultiArray msg;
  msg.layout.dim.resize(1);
  msg.layout.dim[0].label = "state";
  msg.layout.dim[0].size = 1;
  msg.layout.dim[0].stride = 17;
  msg.data.resize(17);
  msg.data[0] = state->t_;
  msg.data[1] = state->T_w_i_.trans.x();
  msg.data[2] = state->T_w_i_.trans.y();
  msg.data[3] = state->T_w_i_.trans.z();
  msg.data[4] = state->T_w_i_.rot.w();
  msg.data[5] = state->T_w_i_.rot.x();
  msg.data[6] = state->T_w_i_.rot.y();
  msg.data[7] = state->T_w_i_.rot.z();
  msg.data[8] = state->linVel_.x();
  msg.data[9] = state->linVel_.y();
  msg.data[10] = state->linVel_.z();
  msg.data[11] = state->accelBias_.x();
  msg.data[12] = state->accelBias_.y();
  msg.data[13] = state->accelBias_.z();
  msg.data[14] = state->gyroBias_.x();
  msg.data[15] = state->gyroBias_.y();
  msg.data[16] = state->gyroBias_.z();
  pubState_.publish(msg);

  //Publish the latest T_w_i
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time(state->t_);
  pose_msg.header.frame_id = "world";
  pose_msg.pose.position.x = state->T_w_i_.trans.x();
  pose_msg.pose.position.y = state->T_w_i_.trans.y();
  pose_msg.pose.position.z = state->T_w_i_.trans.z();
  pose_msg.pose.orientation.w = state->T_w_i_.rot.w();
  pose_msg.pose.orientation.x = state->T_w_i_.rot.x();
  pose_msg.pose.orientation.y = state->T_w_i_.rot.y();
  pose_msg.pose.orientation.z = state->T_w_i_.rot.z();
  pubPose_.publish(pose_msg);

  //Publish a marker for each tag
  if (aprilTagInterface_)
    aprilTagInterface_->publishTagMarkers();

  //Publish an arrow for each state in the batch
  geometry_msgs::PoseArray statesMsg;
  statesMsg.header.frame_id = "/world";
  for (int i = 0; i < stateVector->size(); ++i) {
    auto state = std::dynamic_pointer_cast<const SmbState>((*stateVector)[i]);
    statesMsg.poses.push_back(getMsg(state->T_w_i_));
  }
  pubStates_.publish(statesMsg);

  if (forwardPropagateState_ && forwardPropagationSensor_ == BMM) {
    auto state = std::dynamic_pointer_cast<SmbState>(statePtr);

    BaseMotionModelParameters bmmParams;
    bmmParams.t = state->t();
    bmmParams.T_w_b = state->T_w_i_ * T_imu_base_;

    bmmPropagator_.update(bmmParams);
  }

  //todo Put this somewhere nicer or make it get called automatically after optimization somehow?
  if (ertModule_)
    ertModule_->copyOutParametersAfterOptimization();
}


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  std::cout << std::setprecision(15) << std::endl;

  ros::init(argc, argv, "SmbConFusor");
  ros::NodeHandle nh("~");

  SmbConFusor smbConFusor(nh);

  ros::spin();

  return 0;
}
