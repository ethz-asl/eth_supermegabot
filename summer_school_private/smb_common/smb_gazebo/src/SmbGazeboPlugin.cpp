/*
 *  SmbGazeboPlugin.cpp
 *
 *  Created on: Aug 21, 2018
 *      Author: Koen Kraemer
 */

#include <smb_gazebo/SmbGazeboPlugin.hpp>


namespace gazebo {

  using namespace param_io;

  SmbGazeboPlugin::SmbGazeboPlugin() :
    SmbGazeboPluginBase("SuperMegaBot"),
    receiveMaxLockTime_{std::chrono::microseconds{200}}
	{

	}

  void SmbGazeboPlugin::initJointStructures()
     {
       // Init the joint structures.
       for (const auto actuatorKey : ConcreteDescription::getActuatorKeys()) {
         const auto actuatorEnum = actuatorKey.getEnum();
         const auto actuatorName = actuatorKey.getName();
         auto joint = this->robotDescriptionUrdfModel_.getJoint(actuatorName);
         if (!joint) {
           ROS_ERROR_STREAM_NAMED("gazebo_ros_control", "Joint named '" << actuatorName << "' does not exist in the URDF model.");
           return;
         }
         auto simJoint = this->model_->GetJoint(actuatorName);
         if (!simJoint) {
           ROS_ERROR_STREAM_NAMED("gazebo_ros_control", "Joint named '" << actuatorName << "' does not exist in Gazebo.");
           return;
         }

         // Initialize first order low-pass filters for joint velocity estimation
         this->jointVelocityFilter_[actuatorEnum] = robot_utils::FirstOrderFilterD(1.0, this->publishingTimeStep_/100.0, 1.0);
         // Set joint position to default initial position
 #if GAZEBO_MAJOR_VERSION >= 7
         simJoint->SetPosition(0, this->jointPositionsDefault_[actuatorEnum]);
 #else
         simJoint->SetAngle(0, this->jointPositionsDefault_[actuatorEnum]);
 #endif
         this->simJoints_[actuatorEnum] = simJoint;
         this->jointTypes_[actuatorEnum] = joint->type;
         this->jointPositionsReset_[actuatorEnum] = this->jointPositionsDefault_[actuatorEnum];
         this->jointPositionLimitsLow_[actuatorEnum] = joint->limits->lower;
         this->jointPositionLimitsHigh_[actuatorEnum] = joint->limits->upper;
         this->jointTorqueLimits_[actuatorEnum] = joint->limits->effort;
         this->jointVelocityLimits_[actuatorEnum] = joint->limits->velocity;
       }

       /* Initialize actuators */
       romo_std::fillContainer<ConcreteDescription>(smbCommands_.wheelCommands_);
       romo_std::fillContainer<ConcreteDescription>(smbDefaultPositionCommands_.wheelCommands_); //TODO what to do here? There are no positions
       romo_std::fillContainer<ConcreteDescription>(smbReadings_.wheelReadings_);
       setActuatorGains();

       // Smb commands
       auto& smbDefaultCommands = smbDefaultPositionCommands_.wheelCommands_;

       for (const auto smbActuatorKey : ConcreteDescription::getKeys<smb_description::SmbTopology::SmbActuatorEnum>()) {
         const auto smbActuatorEnum = smbActuatorKey.getEnum();
         const auto actuatorEnum = smb_description::SmbTopology::mapSmbActuatorEnumToActuatorEnum::at(smbActuatorEnum);

         smbDefaultCommands[smbActuatorEnum].setWheelVelocity(0.0);
         smbDefaultCommands[smbActuatorEnum].setWheelTorque(0.0);
         smbDefaultCommands[smbActuatorEnum].setMode(smb_common::SmbMode::MODE_WHEEL_VELOCITY);
       }
//       previousActuatorMode_.fill(kinova_common::KinovaMode::MODE_JOINT_POSITION); // Should be initialized to anything non-freeze
       smbCommands_.wheelCommands_ = smbDefaultCommands;

       // Smb actuator readings
       auto& smbReadings = smbReadings_.wheelReadings_;

       for (const auto smbActuatorKey : ConcreteDescription::getKeys<smb_description::SmbTopology::SmbActuatorEnum>()) {
         const auto smbActuatorEnum = smbActuatorKey.getEnum();

         smbReadings[smbActuatorEnum].setWheelVelocity(0.0);
         smbReadings[smbActuatorEnum].setWheelTorque(0.0);
       }
     }

  void SmbGazeboPlugin::readParameters() {
  getParam(*nodeHandle_, "simulate_estimator", simulateEstimator_);

  getParam(*nodeHandle_, "frame/base/name", frameBase_);
  getParam(*nodeHandle_, "frame/odometry/name", frameOdometry_);
  getParam(*nodeHandle_, "frame/world/name", frameWorld_);
//  getParam(*nodeHandle_, "frame/world_gravity_aligned/name", frameWorldGravityAligned_);

  getParam(*nodeHandle_, "frame/odometry/offset/x", frameOdometryOffsetX_);
  getParam(*nodeHandle_, "frame/odometry/offset/y", frameOdometryOffsetY_);
  getParam(*nodeHandle_, "frame/odometry/offset/z", frameOdometryOffsetZ_);
  getParam(*nodeHandle_, "contact_forces/linear_velocity_threshold", contactLinearVelocityThreshold_);
  std::vector<double> jointPositionsDefault;
  getParam(*nodeHandle_, "joint_states/default_positions", jointPositionsDefault);
  for (const auto actuatorKey : SD::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    const auto actuatorId = actuatorKey.getId();
    jointPositionsDefault_[actuatorEnum] = jointPositionsDefault[actuatorId];
  }
  // Initialize map from collision link names to publisher names.
  collisionLinkNamesToPublisherNames_[SD::ContactEnum::BASE] = std::string{"contact_force_base"}; //TODO leave this?
  // Initialize smb model and frames generator.
//      framesGenerator_.initialize(smbModel_);
  smb_description_ros::initializeSmbState(smbMsgRos_);
//      smb_description_ros::initializeActuatorCommands(smbActuatorDefaultPositionCommands_); //TODO can we put this in?
}

void SmbGazeboPlugin::initSubscribers() {
  // Actuator commands.
  auto optionsActuatorCommands = std::make_shared<cosmo_ros::SubscriberRosOptions<SmbCommandsShm>>(
          "actuator_commands", std::bind(&SmbGazeboPlugin::actuatorCommandsCallback, this, std::placeholders::_1), *nodeHandle_);

  optionsActuatorCommands->autoSubscribe_ = false;
  optionsActuatorCommands->rosTransportHints_ = ros::TransportHints().tcpNoDelay();
  optionsActuatorCommands->tryRosResubscribing_ = false;


  smbActuatorCommandsSubscriber_ = cosmo_ros::subscribeShmRos<SmbCommandsShm,
          SmbCommandsRos,
          SmbCommandsConversionTrait>("actuator_commands", optionsActuatorCommands);
}


     void SmbGazeboPlugin::setActuatorGains() {
         // Not implemented for Smb atm
     }

     void SmbGazeboPlugin::publishActuatorReadings() {
       smbActuatorReadingsPublisher_->publish(smbReadings_, std::chrono::microseconds(200));
     }

     double SmbGazeboPlugin::computeActuatorCommand(const ActuatorEnum& actuatorEnum) {

       double wheelTorque = 0.0;

       if (smb_description::SmbTopology::mapSmbActuatorEnumToActuatorEnum::containsValue(actuatorEnum)) {
         const auto& wheelReadings = smbReadings_.wheelReadings_;
         const auto& wheelCommands = smbCommands_.wheelCommands_;
         const auto smbEnum = smb_description::SmbTopology::mapSmbActuatorEnumToActuatorEnum::find(actuatorEnum);

//         constexpr double kp_freeze = 100.0; //TODO get from param
//         constexpr double kd_freeze = 2.0;

         const auto& wheelMode = wheelCommands[smbEnum].getMode();
         if (wheelMode == smb_common::SmbMode::MODE_WHEEL_VELOCITY) {
           constexpr double kd = 50.0;
           const double velocityDesired = wheelCommands[smbEnum].getWheelVelocity();
           const double velocityMeasured = wheelReadings[smbEnum].getWheelVelocity();
           wheelTorque = kd * (velocityDesired - velocityMeasured);
         }
         else if (wheelMode == smb_common::SmbMode::MODE_WHEEL_TORQUE) {
           wheelTorque = wheelCommands[smbEnum].getWheelTorque();
         }
         else if (wheelMode == smb_common::SmbMode::FREEZE) {
             constexpr double kd = 100.0;
             const double velocityDesired = 0;
             const double velocityMeasured = wheelReadings[smbEnum].getWheelVelocity();
             wheelTorque = kd * (velocityDesired - velocityMeasured);
           }
         else {
           MELO_WARN_THROTTLE_STREAM(3, "[" << this->robotName_ << "SmbGazeboPlugin::computeActuatorCommand] No actuator controller has been implemented for this operation mode");
         }
       }
       else {
         MELO_WARN_STREAM("[" << this->robotName_ << "SmbGazeboPlugin::computeActuatorCommand] Unknown/invalid actuatorEnum ");
         wheelTorque = 0.0;
       }

       return wheelTorque;
     }

     void SmbGazeboPlugin::updateActuatorReading(
             const ActuatorEnum& actuatorEnum,
             const double jointPosition,
             const double jointVelocity,
             const double jointAcceleration,
             const double jointTorque)
     {
       if (smb_description::SmbTopology::mapSmbActuatorEnumToActuatorEnum::containsValue(actuatorEnum)) {
         const auto smbEnum = smb_description::SmbTopology::mapSmbActuatorEnumToActuatorEnum::find(actuatorEnum);

         smbReadings_.wheelReadings_[smbEnum].setWheelVelocity(jointVelocity);
         smbReadings_.wheelReadings_[smbEnum].setWheelTorque(jointTorque);

         smbReadings_.wheelReadings_[smbEnum].setStamp(this->getTime());
       }
       else {
         MELO_WARN_STREAM("[" << this->robotName_ << "SmbGazeboPlugin::updateActuatorReading] Unknown/invalid actuatorEnum ");
       }
     }

     void SmbGazeboPlugin::actuatorCommandsCallback(const SmbCommandsShm& msg) {
       smbCommands_ = msg;
     }

     void SmbGazeboPlugin::receiveMessages() {
       smbActuatorCommandsSubscriber_->receive(receiveMaxLockTime_);
     }

     void SmbGazeboPlugin::initPublishers() {

       MELO_INFO_STREAM("Initializing Publishers");

       // Actuator readings.
       if(simulateEstimator_) {
         auto actReadingOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("actuator_readings", *nodeHandle_);
         actReadingOptions->rosQueueSize_ = 10u;
         actReadingOptions->rosLatch_ = false;
         actReadingOptions->autoPublishRos_ = false;

         smbActuatorReadingsPublisher_ = cosmo_ros::advertiseShmRos<SmbReadingsShm,
                 SmbReadingsRos,
                 SmbReadingConversionTrait>("actuator_readings", actReadingOptions);
       }
       else {
         auto actReadingOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("actuator_readings", *nodeHandle_);
         actReadingOptions->rosQueueSize_ = 10u;
         actReadingOptions->rosLatch_ = false;
         actReadingOptions->autoPublishRos_ = false;

         smbActuatorReadingsPublisher_ = cosmo_ros::advertiseShmRos<SmbReadingsShm,
                 SmbReadingsRos,
                 SmbReadingConversionTrait>("actuator_readings", actReadingOptions);
       }

       // Contact forces
       if(simulateEstimator_) {
         std::string postFix = "_estimator";

         for(const auto contactKey : SD::getContactKeys()) {
           const auto contactEnum = contactKey.getEnum();
           auto publishOptions = std::make_shared<cosmo_ros::PublisherRosOptions>(collisionLinkNamesToPublisherNames_[contactEnum] + postFix, *nodeHandle_);
           publishOptions->rosQueueSize_ = 1u;
           publishOptions->rosLatch_ = false;
           publishOptions->autoPublishRos_ = false;

           contactForcePublishers_[contactEnum] = cosmo_ros::advertiseShmRos<WrenchShm, WrenchRos, any_measurements_ros::ConversionTraits>(
                   collisionLinkNamesToPublisherNames_[contactEnum] + postFix, publishOptions);
         }
       }
       else {
         for(const auto contactKey : SD::getContactKeys()) {
           const auto contactEnum = contactKey.getEnum();
           auto publishOptions = std::make_shared<cosmo_ros::PublisherRosOptions>(collisionLinkNamesToPublisherNames_[contactEnum], *nodeHandle_);
           publishOptions->rosQueueSize_ = 1u;
           publishOptions->rosLatch_ = false;
           publishOptions->autoPublishRos_ = false;

           contactForcePublishers_[contactEnum] = cosmo_ros::advertiseShmRos<WrenchShm, WrenchRos, any_measurements_ros::ConversionTraits>(
                   collisionLinkNamesToPublisherNames_[contactEnum], publishOptions);
         }
       }
       // Contact forces throttled
       for(const auto contactKey :  SD::getContactKeys()) {
         const auto contactEnum = contactKey.getEnum();
         contactForcePublishersThrottled_[contactEnum] = nodeHandle_->advertise<geometry_msgs::WrenchStamped>(
                 param<std::string>(*nodeHandle_, "publishers/"+collisionLinkNamesToPublisherNames_[contactEnum]+"_throttled/topic", "/default"),
                 param<unsigned int>(*nodeHandle_, "publishers/"+collisionLinkNamesToPublisherNames_[contactEnum]+"_throttled/queue_size", 1u),
                 param<bool>(*nodeHandle_, "publishers/"+collisionLinkNamesToPublisherNames_[contactEnum]+"_throttled/latch", false));

         contactForceThrottledDecimationMap_[contactEnum] = param<unsigned int>(*nodeHandle_, "publishers/"+collisionLinkNamesToPublisherNames_[contactEnum]+"_throttled/decimation", 40);
         contactForceThrottledCounterMap_[contactEnum] = 0;
       }
       if (simulateEstimator_) {
//         // Joint States
         auto jointStateOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("joint_states", *nodeHandle_);
         jointStateOptions->rosQueueSize_ = 1u;
         jointStateOptions->rosLatch_ = false;
         jointStateOptions->autoPublishRos_ = false;
         jointStatesPublisher_ = cosmo_ros::advertiseShmRos<JointStateShm, JointStateRos, any_measurements_ros::ConversionTraits>("joint_states", jointStateOptions);

         // Smb state.
         const std::string robotStateTopic = param<std::string>(*nodeHandle_, "publishers/smb_state/topic", "/default");
         auto smbStateOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("smb_state", *nodeHandle_);
         smbStateOptions->rosQueueSize_ = 1u;
         smbStateOptions->rosLatch_ = false;
         smbStateOptions->autoPublishRos_ = false;

         smbStatePublisher_ = cosmo_ros::advertiseShmRos<SmbStateShm, SmbStateRos, smb_description_ros::ConversionTraits>("smb_state", smbStateOptions);
         // Smb state throttled.
         smbStateThrottledPublisher_ = nodeHandle_->advertise<smb_msgs::SmbState>(
                 param<std::string>(*nodeHandle_, "publishers/smb_state_throttle/topic", "/default"),
                 param<unsigned int>(*nodeHandle_, "publishers/smb_state_throttle/queue_size", 1u),
                 param<bool>(*nodeHandle_, "publishers/smb_state_throttle/latch", false));
         getParam(*nodeHandle_, "publishers/smb_state_throttle/decimation", robotStateThrottledDecimation_);

         if (robotStateThrottledDecimation_ == 0) {
           ROS_WARN_STREAM_NAMED("gazebo_ros_control", "The smb state throttle decimation must not be 0, setting it to 1.");
           robotStateThrottledDecimation_ = 1;
         }

         basePoseMeasuredPublisher_ = nodeHandle_->advertise<geometry_msgs::PoseStamped>("/base_pose_measured", 11, true);
         baseTwistMeasuredPublisher_ = nodeHandle_->advertise<geometry_msgs::TwistStamped>("/base_twist_measured", 1, true);
//
//         // Poses.
//         auto odomOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("pose_in_odom", *nodeHandle_);
//         odomOptions->rosQueueSize_ = 1u;
//         odomOptions->rosLatch_ = false;
//         odomOptions->autoPublishRos_ = false;
//
//         poseInOdomPublisher_ = cosmo_ros::advertiseShmRos<PoseWithCovarianceShm, PoseWithCovarianceRos, any_measurements_ros::ConversionTraits>(
//                 "pose_in_odom", odomOptions);
//
//         poseInOdomPublisherThrottled_ = nodeHandle_->advertise<geometry_msgs::PoseWithCovarianceStamped>(
//                 param<std::string>(*nodeHandle_, "publishers/pose_in_odom_throttled/topic", "/default"),
//                 param<unsigned int>(*nodeHandle_, "publishers/pose_in_odom_throttled/queue_size", 1u),
//                 param<bool>(*nodeHandle_, "publishers/pose_in_odom_throttled/latch", false));
//
//         getParam(*nodeHandle_, "publishers/pose_in_odom_throttled/decimation", poseInOdomThrottledDecimation_);
//
//         auto mapOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("pose_in_map", *nodeHandle_);
//         mapOptions->rosQueueSize_ = 1u;
//         mapOptions->rosLatch_ = false;
//         mapOptions->autoPublishRos_ = false;

//         poseInMapPublisher_ = cosmo_ros::advertiseShmRos<PoseWithCovarianceShm, PoseWithCovarianceRos, any_measurements_ros::ConversionTraits>(
//                 "pose_in_map", mapOptions);
//
//         poseInMapPublisherThrottled_ = nodeHandle_->advertise<geometry_msgs::PoseWithCovarianceStamped>(
//                 param<std::string>(*nodeHandle_, "publishers/pose_in_map_throttled/topic", "/default"),
//                 param<unsigned int>(*nodeHandle_, "publishers/pose_in_map_throttled/queue_size", 1u),
//                 param<bool>(*nodeHandle_, "publishers/pose_in_map_throttled/latch", false));
//
//         getParam(*nodeHandle_, "publishers/pose_in_map_throttled/decimation", poseInMapThrottledDecimation_);
//
//         auto mapGaOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("pose_in_map_ga", *nodeHandle_);
//         mapGaOptions->rosQueueSize_ = 1u;
//         mapGaOptions->rosLatch_ = false;
//         mapGaOptions->autoPublishRos_ = false;
//
//         poseInMapGaPublisher_ = cosmo_ros::advertiseShmRos<PoseWithCovarianceShm, PoseWithCovarianceRos, any_measurements_ros::ConversionTraits>(
//                 "pose_in_map_ga", mapOptions);
//
//         poseInMapGaPublisherThrottled_ = nodeHandle_->advertise<geometry_msgs::PoseWithCovarianceStamped>(
//                 param<std::string>(*nodeHandle_, "publishers/pose_in_map_ga_throttled/topic", "/default"),
//                 param<unsigned int>(*nodeHandle_, "publishers/pose_in_map_ga_throttled/queue_size", 1u),
//                 param<bool>(*nodeHandle_, "publishers/pose_in_map_ga_throttled/latch", false));
//
//         getParam(*nodeHandle_, "publishers/pose_in_map_ga_throttled/decimation", poseInMapGaThrottledDecimation_);
//
//         // Twist.
//         auto twistOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("twist", *nodeHandle_);
//         twistOptions->rosQueueSize_ = 1u;
//         twistOptions->rosLatch_ = false;
//         twistOptions->autoPublishRos_ = false;
//
//         twistPublisher_ = cosmo_ros::advertiseShmRos<TwistShm, TwistRos, any_measurements_ros::ConversionTraits>("twist", twistOptions);
//
//         twistPublisherThrottled_ = nodeHandle_->advertise<geometry_msgs::TwistWithCovarianceStamped>(
//                 param<std::string>(*nodeHandle_, "publishers/twist_throttled/topic", "/default"),
//                 param<unsigned int>(*nodeHandle_, "publishers/twist_throttled/queue_size", 1u),
//                 param<bool>(*nodeHandle_, "publishers/twist_throttled/latch", false));
//
//         getParam(*nodeHandle_, "publishers/twist_throttled/decimation", twistThrottledDecimation_);
//
//         // Global localization state.
//         globalLocalizationStatePublisher_ = nodeHandle_->advertise<any_msgs::State>(
//                 param<std::string>(*nodeHandle_, "publishers/global_localization_state/topic", "/default"),
//                 param<unsigned int>(*nodeHandle_, "publishers/global_localization_state/queue_size", 1u),
//                 param<bool>(*nodeHandle_, "publishers/global_localization_state/latch", false));
//
//         // Publish the global localization state once.
//         publishGlobalLocalizationState();
       }
     }

     void SmbGazeboPlugin::initServices()
     {
       // Set robot pose.
       setRobotPoseServer_ = nodeHandle_->advertiseService(
               param<std::string>(*nodeHandle_, "servers/set_robot_pose/service", "/default"),
               &SmbGazeboPluginBase::setRobotPoseCb, (SmbGazeboPluginBase*)this);
     }

     void SmbGazeboPlugin::publishRobotState()
     {
       any_measurements::Time stamp = getTime();
       smbMsgShm_.time_ = stamp;
       smbMsgShm_.status_ = smb_description::StateStatus::STATUS_OK;

       for (const auto actuatorKey : SD::getActuatorKeys()) {
         const auto actuatorEnum = actuatorKey.getEnum();
         const auto actuatorId = actuatorKey.getId();

         if (smb_description::SmbTopology::mapSmbActuatorEnumToActuatorEnum::containsValue(actuatorEnum)) {
           const auto smbActuatorEnum = smb_description::SmbTopology::mapSmbActuatorEnumToActuatorEnum::find(actuatorEnum);

           smbMsgShm_.smbState_.getWheelVelocities()(actuatorId) = smbReadings_.wheelReadings_[smbActuatorEnum].getWheelVelocity();
           smbMsgShm_.smbState_.getWheelTorques()(actuatorId) = smbReadings_.wheelReadings_[smbActuatorEnum].getWheelTorque();
         }
         else {
           MELO_WARN_STREAM("[" << this->robotName_ << "SmbGazeboPlugin::publishRobotState] Unknown/invalid actuatorEnum ");
         }
       }

       kindr::RotationQuaternionD orientationBaseToWorld;
#if GAZEBO_MAJOR_VERSION >= 9
       orientationBaseToWorld = kindr::RotationQuaternionD(
             robotBaseLinkPose_.Rot().W(),
             robotBaseLinkPose_.Rot().X(),
             robotBaseLinkPose_.Rot().Y(),
             robotBaseLinkPose_.Rot().Z());
#else
       orientationBaseToWorld = kindr::RotationQuaternionD(
               robotBaseLinkPose_.rot.w,
               robotBaseLinkPose_.rot.x,
               robotBaseLinkPose_.rot.y,
               robotBaseLinkPose_.rot.z);
#endif


       const kindr::Velocity3D B_v_B(robotBaseLinkLinearVelocity_[0],
                                     robotBaseLinkLinearVelocity_[1],
                                     robotBaseLinkLinearVelocity_[2]);

       const kindr::LocalAngularVelocityPD B_w_IB(robotBaseLinkAngularVelocity_[0],
                                                  robotBaseLinkAngularVelocity_[1],
                                                  robotBaseLinkAngularVelocity_[2]);


       //-- Generalized positions
#if GAZEBO_MAJOR_VERSION >= 9
       smbMsgShm_.smbState_.setPositionWorldToBaseInWorldFrame(
             smb_model::Position(robotBaseLinkPose_.Pos()[0] - frameOdometryOffsetX_,
                                 robotBaseLinkPose_.Pos()[1] - frameOdometryOffsetY_,
                                 robotBaseLinkPose_.Pos()[2] - frameOdometryOffsetZ_));
#else
       smbMsgShm_.smbState_.setPositionWorldToBaseInWorldFrame(
             smb_model::Position(robotBaseLinkPose_.pos[0] - frameOdometryOffsetX_,
                                 robotBaseLinkPose_.pos[1] - frameOdometryOffsetY_,
                                 robotBaseLinkPose_.pos[2] - frameOdometryOffsetZ_));
#endif
       smbMsgShm_.smbState_.setOrientationBaseToWorld(orientationBaseToWorld);
       //--

       //-- Generalized velocities
       smbMsgShm_.smbState_.setLinearVelocityBaseInWorldFrame(orientationBaseToWorld.rotate(B_v_B));
       smbMsgShm_.smbState_.setAngularVelocityBaseInBaseFrame(B_w_IB);

       if (!simulateEstimator_) {
         return;
       }

       // update frames generators
//       framesGenerator_.update(kinovaModel_);
//       kindr::HomTransformQuatD transform;
//
//       // odom to map
//       transform.getPosition() = kindr::Position3D(-frameOdometryOffsetX_, -frameOdometryOffsetY_, -frameOdometryOffsetZ_);
//       transform.getRotation() = kindr::RotationQuaternionD(1.0, 0.0, 0.0, 0.0);
//       smbMsgShm_.smbState_.setFrameTransform(smb_description::SmbTopology::FrameTransformEnum::MapToOdom, transform);
//
//       // odom to map_ga
//       smbMsgShm_.smbState_.setFrameTransform(smb_description::SmbTopology::FrameTransformEnum::MapGaToOdom, transform);

       smb_description_ros::ConversionTraits<smb_description::SmbState, smb_msgs::SmbState>::convert(smbMsgShm_, smbMsgRos_);
       smbStatePublisher_->publish(smbMsgShm_, smbMsgRos_, std::chrono::microseconds(200));

       // Publish base state over Ros for path planning module
       basePoseMeasuredMsg_.header.stamp = ros::Time::now();
       basePoseMeasuredMsg_.header.frame_id = "odom";
       basePoseMeasuredMsg_.header.seq++;
       basePoseMeasuredMsg_.pose.position.x = smbMsgShm_.smbState_.getPositionWorldToBaseInWorldFrame().x();
       basePoseMeasuredMsg_.pose.position.y = smbMsgShm_.smbState_.getPositionWorldToBaseInWorldFrame().y();
       basePoseMeasuredMsg_.pose.position.z = smbMsgShm_.smbState_.getPositionWorldToBaseInWorldFrame().z();
       basePoseMeasuredMsg_.pose.orientation.x = smbMsgShm_.smbState_.getOrientationBaseToWorld().x();
       basePoseMeasuredMsg_.pose.orientation.y = smbMsgShm_.smbState_.getOrientationBaseToWorld().y();
       basePoseMeasuredMsg_.pose.orientation.z = smbMsgShm_.smbState_.getOrientationBaseToWorld().z();
       basePoseMeasuredMsg_.pose.orientation.w = smbMsgShm_.smbState_.getOrientationBaseToWorld().w();

       baseTwistMeasuredMsg_.header.stamp = ros::Time::now();
       baseTwistMeasuredMsg_.header.frame_id = "odom";
       baseTwistMeasuredMsg_.header.seq++;
       baseTwistMeasuredMsg_.twist.linear.x = smbMsgShm_.smbState_.getLinearVelocityBaseInWorldFrame().x();
       baseTwistMeasuredMsg_.twist.linear.y = smbMsgShm_.smbState_.getLinearVelocityBaseInWorldFrame().y();
       baseTwistMeasuredMsg_.twist.linear.z = smbMsgShm_.smbState_.getLinearVelocityBaseInWorldFrame().z();
       baseTwistMeasuredMsg_.twist.angular.x = smbMsgShm_.smbState_.getAngularVelocityBaseInBaseFrame().x();
       baseTwistMeasuredMsg_.twist.angular.y = smbMsgShm_.smbState_.getAngularVelocityBaseInBaseFrame().y();
       baseTwistMeasuredMsg_.twist.angular.z = smbMsgShm_.smbState_.getAngularVelocityBaseInBaseFrame().z();

       geometry_msgs::PoseStampedPtr basePoseMeasuredMsg(new geometry_msgs::PoseStamped(basePoseMeasuredMsg_));
       basePoseMeasuredPublisher_.publish(geometry_msgs::PoseStampedConstPtr(basePoseMeasuredMsg));

       geometry_msgs::TwistStampedPtr baseTwistMeasuredMsg(new geometry_msgs::TwistStamped(baseTwistMeasuredMsg_));
       baseTwistMeasuredPublisher_.publish(geometry_msgs::TwistStampedConstPtr(baseTwistMeasuredMsg));
     }

     void SmbGazeboPlugin::Reset() {
       MELO_INFO_STREAM("[" << robotName_ << "GazeboPlugin::Reset] Resetting gazebo model.");
       std::unique_lock<std::recursive_mutex> lock(gazeboMutex_);

       // Reset timings.
       lastStartUpdateSimTime_ = common::Time();
       lastStartUpdateSysTime_ = std::chrono::steady_clock::now();
       updateSimTimeStep_ = 0.0;

       for (const auto actuatorKey : ConcreteDescription::getActuatorKeys()) {
         const auto actuatorEnum = actuatorKey.getEnum();

       // Reset the joint state to the default one, then switch to disable and to freeze to hold it.
 #if GAZEBO_MAJOR_VERSION >= 7
         simJoints_[actuatorEnum]->SetPosition(0, jointPositionsDefault_[actuatorEnum]);
 #else
         simJoints_[actuatorEnum]->SetAngle(0, jointPositionsDefault_[actuatorEnum]);
 #endif

         jointVelocityFilter_[actuatorEnum].reset();

         if (smb_description::SmbTopology::mapSmbActuatorEnumToActuatorEnum::containsValue(actuatorEnum)) {
           const auto smbActuatorEnum = smb_description::SmbTopology::mapSmbActuatorEnumToActuatorEnum::find(actuatorEnum);
           smbCommands_.wheelCommands_[smbActuatorEnum].setMode(smb_common::SmbMode::MODE_WHEEL_VELOCITY);
         }
         else {
           MELO_WARN_STREAM("[" << this->robotName_ << "KinovaGazeboPlugin::Reset] Unknown/invalid actuatorEnum ");
         }
       }
     }

     void SmbGazeboPlugin::sendRos() {
       smbActuatorReadingsPublisher_->sendRos();

       for(const auto contactKey : SD::getContactKeys()) {
         const auto contactEnum = contactKey.getEnum();
         contactForcePublishers_[contactEnum]->sendRos();
       }

       if(simulateEstimator_) {
//         smbStatePublisher_->sendRos(); // visualization now with cosmo
//
//         if (robotStateThrottledCounter_ == robotStateThrottledDecimation_) {
//           if (smbStateThrottledPublisher_.getNumSubscribers() > 0u) {
//             smbStateThrottledPublisher_.publish(smb_description_ros::ConversionTraits<smb_description::SmbState, smb_msgs::SmbState>::convert(smbMsgShm_));
//           }
//           robotStateThrottledCounter_ = 0;
//         }
//         ++robotStateThrottledCounter_;
//
//         poseInOdomPublisher_->sendRos();
//         poseInMapPublisher_->sendRos();
//         poseInMapGaPublisher_->sendRos();
//         twistPublisher_->sendRos();
//
////         jointStatesPublisher_->sendRos();
       }
     }

     GZ_REGISTER_MODEL_PLUGIN(SmbGazeboPlugin)
 } // namespace gazebo
