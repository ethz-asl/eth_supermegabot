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

namespace confusion {

template <typename StateType>
TagTracker<StateType>::TagTracker(ros::NodeHandle &node) : node_(node),
                                                conFusor_(std::make_shared<StateType>()),
                                                statesBatch_(NUM_PROCESS_SENSORS, NUM_UPDATE_SENSORS),
                                                imuPropagator_(5000) {
  // Load config parameters from the text file
  confusionPath_ = ros::package::getPath("confusion");
  configFile = confusionPath_ + configFile;
  boost::property_tree::read_info(configFile, pt);

  aprilTagInterface_ = std::unique_ptr<AprilTagModule>(new AprilTagModule(node, &conFusor_, configFile, TAG, &newTagMeasReady_));

  // Link the states to the reference frame maps for adding new frames on the fly
  auto firstState = std::dynamic_pointer_cast<StateType>(conFusor_.stateVector_.front());
  firstState->setTagReferenceFrameOffsets(aprilTagInterface_->getTagReferenceFrameOffsetsMapPointer());

  // Set up the logger
  std::string logFileName = confusionPath_ + "/data/tagtracker_log.txt";
  logger_ = std::unique_ptr<confusion::Logger>(new confusion::Logger(logFileName,
                                                                     *conFusor_.stateVector_.front()));
  logData_ = pt.get<bool>("logData");

  conFusor_.setSolverOptions(pt.get<int>("numThreads"), pt.get<int>("maxNumIterations"));
  batchSize_ = pt.get<int>("batchSize");
  runBatch_ = pt.get<bool>("runBatch");
  forwardPropagateState_ = pt.get<bool>("forwardPropagateState");

  //Set the initial constraint on the state
  tagTrackerParameters_.twi_init_stddev = pt.get<double>("twi_init_stddev");
  tagTrackerParameters_.qwi_init_stddev = pt.get<double>("qwi_init_stddev");
  tagTrackerParameters_.vwi_init_stddev = pt.get<double>("vwi_init_stddev");
  tagTrackerParameters_.ba_init_stddev = pt.get<double>("ba_init_stddev");
  tagTrackerParameters_.bg_init_stddev = pt.get<double>("bg_init_stddev");

  std::vector<Eigen::MatrixXd> stateParamInitialWeightings(conFusor_.stateVector_.front()->parameters_.size());
  Eigen::MatrixXd param_weighting(3, 3);
  param_weighting.setIdentity();
  param_weighting /= tagTrackerParameters_.twi_init_stddev;
  stateParamInitialWeightings[0] = param_weighting;

  param_weighting.setIdentity();
  param_weighting /= tagTrackerParameters_.qwi_init_stddev;
  stateParamInitialWeightings[1] = param_weighting;

  param_weighting.setIdentity();
  param_weighting /= tagTrackerParameters_.vwi_init_stddev;
  stateParamInitialWeightings[2] = param_weighting;

  param_weighting.setIdentity();
  param_weighting /= tagTrackerParameters_.ba_init_stddev;
  stateParamInitialWeightings[3] = param_weighting;

  param_weighting.setIdentity();
  param_weighting /= tagTrackerParameters_.bg_init_stddev;
  stateParamInitialWeightings[4] = param_weighting;

  conFusor_.stateVector_.front()->setInitialStateWeighting(stateParamInitialWeightings);

  tagTrackerParameters_.imuCalibration_.gravityMagnitude_ = pt.get<double>("gravityMagnitude");
  tagTrackerParameters_.wi_stddev_ = pt.get<double>("wi_stddev");
  tagTrackerParameters_.ai_stddev_ = pt.get<double>("ai_stddev");
  tagTrackerParameters_.bg_stddev_ = pt.get<double>("bg_stddev");
  tagTrackerParameters_.ba_stddev_ = pt.get<double>("ba_stddev");
  tagTrackerParameters_.initialize();

  gravity_rot_.setZero();
#ifdef OPT_GRAVITY
  //	Eigen::MatrixXd gravityRotInitialWeighting(2,2);
  //	gravityRotInitialWeighting.setIdentity();
  //	gravityRotInitialWeighting /= tagTrackerParameters_.tagMeasCalibration_.ftr_init_stddev;
      conFusor_.addStaticParameter(confusion::Parameter(gravity_rot_.data(), 2, "g_r")); //, gravityRotInitialWeighting);
#endif

  //Set up publishers and subscribers
  auto callbackPtr = std::make_shared<confusion::SolverOverrunCallback>(newTagMeasReady_);
  conFusor_.setIterationCallback(callbackPtr);

  pubStates_ = node_.advertise<geometry_msgs::PoseArray>("/states", 10);
  pubState_ = node_.advertise<std_msgs::Float64MultiArray>("/pose_est_out", 10);
  pubRtState_ = node_.advertise<std_msgs::Float64MultiArray>("/rt_est_out", 10);
  pubPose_ = node_.advertise<geometry_msgs::PoseStamped>("/imu_pose", 10);
  pubRtPose_ = node_.advertise<geometry_msgs::PoseStamped>("/imu_pose_prop", 10);

  //Listen for other user triggers
  subTriggerBatch_ = node_.subscribe("/trigger_batch", 1, &TagTracker<StateType>::triggerBatchCalCallback, this);
  subDrawDiagram_ = node_.subscribe("/draw_confusion_diagram", 1, &TagTracker<StateType>::drawDiagramCallback, this);

  //Start listening for measurements
  subImu_ = node_.subscribe(pt.get<std::string>("imu_topic"), 1000, &TagTracker<StateType>::imuCallback,
                            this, ros::TransportHints().tcpNoDelay());

  //Start the estimator on a separate thread
  estimatorThread_ = std::thread(&TagTracker<StateType>::runEstimator, this);
  estimatorThread_.detach();

  std::cout << "[TagTracker] Initialization complete." << std::endl;
}

template <typename StateType>
void TagTracker<StateType>::imuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg) {
  if (!run_)
    return;

//  if (firstMeasurement_) {
//    t_epoch_ = imuMsg->header.stamp.toSec();
//    firstMeasurement_ = false;
//  }

  double t_imu_source = imuMsg->header.stamp.toSec();
  t_imu_latest_ = t_imu_source;

  Eigen::Vector3d a(imuMsg->linear_acceleration.x,
                    imuMsg->linear_acceleration.y,
                    imuMsg->linear_acceleration.z);
  Eigen::Vector3d w(imuMsg->angular_velocity.x,
                    imuMsg->angular_velocity.y,
                    imuMsg->angular_velocity.z);

  conFusor_.addProcessMeasurement(std::make_shared<confusion::ImuMeas>(
      t_imu_source, a, w, &tagTrackerParameters_.imuCalibration_, &gravity_rot_));

  //Optionally publish the forward-propagated state at IMU rate
  if (forwardPropagateState_) {
    confusion::ImuMeas meas(t_imu_source, a, w, &tagTrackerParameters_.imuCalibration_, &gravity_rot_);
    imuPropagator_.addImuMeasurement(meas);

    if (tracking_) {
      confusion::ImuStateParameters state = imuPropagator_.propagate(t_imu_source);

      geometry_msgs::PoseStamped pose_msg = getMsg(state.T_w_i_, "/world", ros::Time(state.t()));
      pubRtPose_.publish(pose_msg);

      std_msgs::Float64MultiArray msg;
      msg.layout.dim.resize(1);
      msg.layout.dim[0].label = "state";
      msg.layout.dim[0].size = 1;
      msg.layout.dim[0].stride = 21;
      msg.data.resize(21);
      msg.data[0] = state.t_;
      msg.data[1] = 0; //State index no longer used??
      msg.data[2] = state.T_w_i_.trans.x();
      msg.data[3] = state.T_w_i_.trans.y();
      msg.data[4] = state.T_w_i_.trans.z();
      msg.data[5] = state.T_w_i_.rot.w();
      msg.data[6] = state.T_w_i_.rot.x();
      msg.data[7] = state.T_w_i_.rot.y();
      msg.data[8] = state.T_w_i_.rot.z();
      msg.data[9] = state.angVel_.x();
      msg.data[10] = state.angVel_.y();
      msg.data[11] = state.angVel_.z();
      msg.data[12] = state.linVel_.x();
      msg.data[13] = state.linVel_.y();
      msg.data[14] = state.linVel_.z();
      msg.data[15] = state.accelBias_.x();
      msg.data[16] = state.accelBias_.y();
      msg.data[17] = state.accelBias_.z();
      msg.data[18] = state.gyroBias_.x();
      msg.data[19] = state.gyroBias_.y();
      msg.data[20] = state.gyroBias_.z();
      pubRtState_.publish(msg);

      //Publish tf message
      tfBroadcaster_.sendTransform(tf::StampedTransform(getTfMsg(state.T_w_i_), ros::Time::now(), "world", "imu"));
    }
  }

#ifdef SYNC_DEBUG
	std::cout << "Imu meas received with stamp " << t_imu_source << std::endl;
#endif
}

template <typename StateType>
void TagTracker<StateType>::runEstimator() {
  ros::Rate loop_rate(loop_freq);

  while (run_) {
    if (!ros::ok()) {
      run_ = false;
      tracking_ = false;
      break;
    }

    ros::Time t_start = ros::Time::now();

    // This processes any new measurements that have been added to the ConFusor in the callbacks
    // and either assigns them to existing states or spawns new states if the measurements are
    // newer than the most recent state.
    if (!conFusor_.assignMeasurements()) {
      loop_rate.sleep();
      continue; //No new states or measurements added, so wait for more measurements
    }
    newTagMeasReady_ = false;

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

//    std::cout << "Estimate delayed from latest IMU measurement by " << t_imu_latest_-conFusor_.stateVector_.back()->t() << " sec. Solver time: " << t_iter << " sec" << std::endl;
//    std::cout << "TagTracker run took " << t_iter << " sec\n" << std::endl;

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

template <typename StateType>
void TagTracker<StateType>::stopTracking() {
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

template <typename StateType>
void TagTracker<StateType>::startTracking() {
  if (run_) {
    std::cout << "TagTracker is already running." << std::endl;
    return;
  }

  stateToDropIndex_ = -1;

  std::cout << "Starting tracking" << std::endl;
  run_ = true;
  estimatorThread_ = std::thread(&TagTracker<StateType>::runEstimator, this);
  estimatorThread_.detach();
}

template <typename StateType>
void TagTracker<StateType>::stopTracking(const std_msgs::Empty &msg) {
  stopTracking();
}

template <typename StateType>
void TagTracker<StateType>::startTracking(const std_msgs::Empty &msg) {
  startTracking();
}

template <typename StateType>
void TagTracker<StateType>::drawDiagramCallback(const std_msgs::Empty &msg) {
  std::cout << "User requested for a MHE diagram to be drawn." << std::endl;
  drawDiagramRequest_ = true;
}

template <typename StateType>
void TagTracker<StateType>::triggerBatchCalCallback(const std_msgs::Empty &msg) {
  if (!runBatch_) {
    std::cout << "WARNING: User requested a batch calibration run even runBatch is not set in "
                 "the config file. Cannot perform the batch calibration." << std::endl;
    return;
  }

  if (run_)
    stopTracking();

  std::cout << "Starting a batch calibration run for " << statesBatch_.size() << " states" << std::endl;

  //Activate the extrinsic calibrations
  auto &T_c_i_ptr = aprilTagInterface_->sensorFrameOffsets_["cam"];
  conFusor_.staticParameters_.getParameter(T_c_i_ptr->trans.data())->unsetConstant();
  conFusor_.staticParameters_.getParameter(T_c_i_ptr->rot.coeffs().data())->unsetConstant();

  publish(statesBatch_.back(), &statesBatch_);

  confusion::BatchFusor batchFusor(statesBatch_, conFusor_.staticParameters_);
  batchFusor.buildProblem();

  Eigen::VectorXd priorResidual;
  priorResidual.resize(0);
  std::vector<Eigen::VectorXd> processResiduals;
  std::vector<Eigen::VectorXd> updateResiduals;
  batchFusor.getResiduals(processResiduals, updateResiduals);

  batchFusor.optimize();

  batchFusor.printParameterCovariance(T_c_i_ptr->trans.data(), "t_c_i");
  batchFusor.printParameterCovariance(T_c_i_ptr->rot.coeffs().data(), "q_c_i");

  publish(statesBatch_.back(), &statesBatch_);

  T_c_i_ptr->print("T_c_i");

  processResiduals.clear();
  updateResiduals.clear();
  batchFusor.getResiduals(processResiduals, updateResiduals);

  if (logData_) {
    logger_->writeBatch(batchFusor.stateVector_);
    logger_->writeStaticParameters(conFusor_.staticParameters_, statesBatch_.back()->t_);
    logger_->writeResiduals(priorResidual, processResiduals, updateResiduals);
  }

  //Reset the status of the extrinsic calibration parameters
  if (!pt.get<bool>("optimizeTci")) {
    conFusor_.staticParameters_.getParameter(T_c_i_ptr->trans.data())->setConstant();
    conFusor_.staticParameters_.getParameter(T_c_i_ptr->rot.coeffs().data())->setConstant();
  }

  //Empty the batch states to start logging for another batch problem
  statesBatch_.reset();

  //Now restart the estimator
  startTracking();
}

template <typename StateType>
void TagTracker<StateType>::publish(std::shared_ptr<confusion::State> statePtr, const confusion::StateVector *stateVector) {
  //Publish the most recent state for use in SL
  auto state = std::dynamic_pointer_cast<StateType>(statePtr);

  if (forwardPropagateState_) {
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
  aprilTagInterface_->publishTagMarkers();

  //Publish an arrow for each state in the batch
  geometry_msgs::PoseArray statesMsg;
  statesMsg.header.frame_id = "/world";
  for (int i = 0; i < stateVector->size(); ++i) {
    auto state = std::dynamic_pointer_cast<const StateType>((*stateVector)[i]);
    statesMsg.poses.push_back(getMsg(state->T_w_i_));
  }
  pubStates_.publish(statesMsg);

  publishDerived(statePtr, stateVector);
}

} // namespace confusion
