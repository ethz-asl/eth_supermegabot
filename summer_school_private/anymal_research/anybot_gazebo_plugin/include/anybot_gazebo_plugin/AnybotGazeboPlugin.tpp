/** \file gazebo_ros_control.h
\brief This file defines the GazeboRosControl class which interfaces ROS
StarlETH locomotion controller with Gazebo simulator.
*/

#include <anybot_gazebo_plugin/AnybotGazeboPlugin.hpp>

namespace gazebo {

template<typename ConcreteDescription_>
AnybotGazeboPlugin<ConcreteDescription_>::~AnybotGazeboPlugin()
{
  stopUpdating_ = true;
  cvUpdate_.notify_all();
  publishWorker_->stop();

  signal_logger::logger->stopLogger();
  signal_logger::logger->saveLoggerData({signal_logger::LogFileType::CSV});
  signal_logger::logger->cleanup(); 
  nodeHandle_->shutdown();
  delete nodeHandle_;
}

template<typename ConcreteDescription_>
void AnybotGazeboPlugin<ConcreteDescription_>::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  std::unique_lock<std::recursive_mutex> lock(gazeboMutex_);
  // compulsory check to see if ROS is correctly initialized
  if (!ros::isInitialized()) {
    MELO_FATAL_STREAM("[" << robotName_ << "GazeboPlugin::Load] A ROS node for Gazebo has not been initialized, unable to load plugin. Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package).");
    return;
  }

  // read some parameters from the SDF model
  robotBaseLink_ = sdf->GetElement("robotBaseLink")->Get<std::string>();
  robotDescriptionParamName_ = sdf->GetElement("robotDescription")->Get<std::string>();
  const double statePublisherRate = sdf->GetElement("statePublisherRate")->Get<double>();
  publishingTimeStep_ = (statePublisherRate > 0.0) ? 1.0 / statePublisherRate : 0.0;


  // save the model pointer, refers to top-level, i.e. anymal...
  model_ = model;

  // create ROS node handle
  nodeHandle_ = new ros::NodeHandle("~");
  MELO_INFO_STREAM("[" << robotName_ << "GazeboPlugin::Load] Starting gazebo_ros_control plugin in namespace: '" << nodeHandle_->getNamespace() << "'.");

  // request the robot_description parameter
  robotDescriptionUrdfString_ = getUrdfRobotDescription(robotDescriptionParamName_);

  // parse the URDF string into a URDF model structure
  robotDescriptionUrdfModel_.initString(robotDescriptionUrdfString_);

  std::string syncName;
  nodeHandle_->param<std::string>("sync_name", syncName, "sync");
  MELO_INFO_STREAM("[SyncMaster] Sync signal name: " << syncName);
  syncMaster_ = std::unique_ptr<cosmo::SyncMaster>(new cosmo::SyncMaster(syncName, publishingTimeStep_, 79, false));

  // read configuration parameters for the plugin
  readParameters();

  // initialize internal data structures
  initJointStructures();
  initContactManager();

  initJointStateMessage();

  // initialize ROS pub/sub/services
  initPublishers();
  checkPublishers();
  syncMaster_->start();
  initSubscribers();
  initServices();

  signal_logger::setSignalLoggerRos(nodeHandle_);
  signal_logger::SignalLoggerOptions siloOptions;
  siloOptions.updateFrequency_ = static_cast<int>(1.0/publishingTimeStep_);
  siloOptions.maxLoggingTime_ = 100.0;
  // siloOptions.collectScriptFileName_ = loggingScriptFilename;
  signal_logger::logger->initLogger(siloOptions);
  signal_logger::add(iterDurationMs_, std::string{"GazeboIterDuration"});
  signal_logger::add(updateDurationMs_, std::string{"GazeboUpdateDuration"});
  signal_logger::logger->updateLogger();
  signal_logger::logger->startLogger();

  updateCounter_ = 01u;
  stopUpdating_ = false;

  any_worker::WorkerOptions pubOptions;
  pubOptions.callback_ = std::bind(&AnybotGazeboPlugin::publishWorker, this, std::placeholders::_1);
  pubOptions.defaultPriority_ = 0; // this has low priority
  pubOptions.name_ = "AnybotGazeboPlugin::publisherWorker";
  pubOptions.timeStep_ = std::numeric_limits<double>::infinity();
  publishWorker_ = std::unique_ptr<any_worker::Worker>(new any_worker::Worker(pubOptions));
  publishWorker_->start();

  // reset simulation variables
  Reset();

  // connect to world updates from Gazebo
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&AnybotGazeboPlugin::updateCb, this));
}


template<typename ConcreteDescription_>
void AnybotGazeboPlugin<ConcreteDescription_>::checkPublishers() {
  if (!poseInOdomPublisher_) {
    MELO_WARN_STREAM("poseInOdomPublisher has not been initialized in initPublishers(), will not publish pose in odom");
  }
  if (!twistPublisher_) {
    MELO_WARN_STREAM("twistPublisher has not been initialized in initPublishers(), will not publish twist");
  }
  if (!jointStatesPublisher_) {
    MELO_WARN_STREAM("jointStatePublisher has not been initialized in initPublishers(), will not publish joint_state");
  }
  for (const auto contactKey : ConcreteDescription::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    if (!contactForcePublishers_[contactEnum]) {
      MELO_WARN_STREAM("contactforcepublishers have not been initialized in initPublishers(), will not publish contact states");
    }
  }
}

template<typename ConcreteDescription_>
void AnybotGazeboPlugin<ConcreteDescription_>::Init() {
  std::unique_lock<std::recursive_mutex> lock(gazeboMutex_);
  MELO_INFO_STREAM("[" << robotName_ << "GazeboPlugin::Init] Initializing gazebo model.");
}

template<typename ConcreteDescription_>
void AnybotGazeboPlugin<ConcreteDescription_>::Reset() {
  MELO_INFO_STREAM("[" << robotName_ << "GazeboPlugin::Reset] Resetting gazebo model.");
  std::unique_lock<std::recursive_mutex> lock(gazeboMutex_);

  ModelPlugin::Reset();

  // Reset timings.
  lastStartUpdateSimTime_ = common::Time();
  lastStartUpdateSysTime_ = std::chrono::steady_clock::now();
  updateSimTimeStep_ = 0.0;

  // Reset the joint state to the default one, then switch to disable and to freeze to hold it.
  for (const auto actuatorKey : ConcreteDescription::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();
#if GAZEBO_MAJOR_VERSION >= 7
    simJoints_[actuatorEnum]->SetPosition(0, jointPositionsDefault_[actuatorEnum]);
#else
    simJoints_[actuatorEnum]->SetAngle(0, jointPositionsDefault_[actuatorEnum]);
#endif

    jointVelocityFilter_[actuatorEnum].reset();
    jointAccelerationFilter_[actuatorEnum].reset();
  }
}

template<typename ConcreteDescription_>
std::string AnybotGazeboPlugin<ConcreteDescription_>::getUrdfRobotDescription(const std::string& paramName) const {
  MELO_INFO_STREAM("[" << robotName_ << "GazeboPlugin::getUrdfRobotDescription] Loading urdf description.");

  std::string urdfString;
  const double timeOut = 5.0; // [s]
  auto start = std::chrono::steady_clock::now();
  while (urdfString.empty()) {
    std::string searchParamName;
    if (nodeHandle_->searchParam(paramName, searchParamName)) {
      MELO_INFO_STREAM("[" << robotName_ << "GazeboPlugin::getUrdfRobotDescription] Waiting for model URDF in parameter " << searchParamName << " on the ROS parameter server.");
      nodeHandle_->getParam(searchParamName, urdfString);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    auto now = std::chrono::steady_clock::now();

    if ((std::chrono::duration_cast<std::chrono::seconds>(now - start)).count() >= timeOut) {
      MELO_WARN_STREAM("[" << robotName_ << "GazeboPlugin::getUrdfRobotDescription] Timeout while loading urdf!");
      break;
    }
  }
  MELO_INFO_STREAM("[" << robotName_ << "GazeboPlugin::getUrdfRobotDescription] Received urdf from parameter server.");
  return urdfString;
}

template<typename ConcreteDescription_>
void AnybotGazeboPlugin<ConcreteDescription_>::initContactManager() {
  MELO_INFO_STREAM("[" << robotName_ << "GazeboPlugin::initContactManager] Initializing contact manager.");

  // get all the links from the model
  auto links = model_->GetLinks();

  std::vector<std::string> collisionScopedNames;

  for (const auto& link : links) {
    auto numCollisions = link->GetCollisions().size();
    for (size_t i = 0; i < numCollisions; ++i) {
      auto scopedName = link->GetCollision(i)->GetScopedName();          

      // for (const auto& collisionLinkName : collisionLinkNames_)
      for (const auto contactKey :  ConcreteDescription::getContactKeys())
        if (scopedName.find(contactKey.getName()) != std::string::npos) {
          const auto contactEnum = contactKey.getEnum();
          link->GetCollision(i)->SetMaxContacts(1);  // TODO: Check if not working.
          collisionLinkNamesToScopedNames_[contactEnum] = scopedName;
          // collisionLinkNamesToScopedNames_.insert(std::make_pair(collisionLinkName, scopedName));
          collisionScopedNames.push_back(scopedName);
#if (GAZEBO_MAJOR_VERSION >= 8)
          contactForces_[contactEnum] = ignition::math::Vector3d::Zero;
          contactTorques_[contactEnum] = ignition::math::Vector3d::Zero;
          contactPositions_[contactEnum] = ignition::math::Vector3d::Zero;
          contactNormals_[contactEnum] = ignition::math::Vector3d::Zero;
          contactLinearVelocities_[contactEnum] = ignition::math::Vector3d::Zero;
          contactFlags_[contactEnum] = false;
#else
          contactForces_[contactEnum] = math::Vector3::Zero;
          contactTorques_[contactEnum] = math::Vector3::Zero;
          contactPositions_[contactEnum] = math::Vector3::Zero;
          contactNormals_[contactEnum] = math::Vector3::Zero;
          contactLinearVelocities_[contactEnum] = math::Vector3::Zero;
          contactFlags_[contactEnum] = false;
#endif

# if GAZEBO_MAJOR_VERSION <= 2
          frictionCoefficients_[contactEnum] = link->GetCollision(i)->GetSurface()->mu1;
          restitutionCoefficients_[contactEnum] = link->GetCollision(i)->GetSurface()->bounce;
# else
# if GAZEBO_MAJOR_VERSION >= 6
          frictionCoefficients_[contactEnum] = link->GetCollision(i)->GetSurface()->FrictionPyramid()->MuPrimary();
          restitutionCoefficients_[contactEnum] = 0.0;
# else
          frictionCoefficients_[contactEnum] = 0.7;
          restitutionCoefficients_[contactEnum] = 0.0;
# endif
# endif
        }
    }
  }

  // Gazebo will provide us on the required contact forces with this filter
#if (GAZEBO_MAJOR_VERSION >= 8)
  model_->GetWorld()->Physics()->GetContactManager()->CreateFilter(model_->GetName(), collisionScopedNames);
#else
  model_->GetWorld()->GetPhysicsEngine()->GetContactManager()->CreateFilter(model_->GetName(), collisionScopedNames);
#endif
}

template<typename ConcreteDescription_>
void AnybotGazeboPlugin<ConcreteDescription_>::sendRos() {

  if (poseInOdomPublisher_) {
    poseInOdomPublisher_->sendRos();
  }

  if (twistPublisher_) {
    twistPublisher_->sendRos();
  }

  if (jointStatesPublisher_) {
    jointStatesPublisher_->sendRos();
  }

  for (const auto contactKey : ConcreteDescription::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    if (contactForcePublishers_[contactEnum]) {
      contactForcePublishers_[contactEnum]->sendRos();
    }
  }
}

template<typename ConcreteDescription_>
void AnybotGazeboPlugin<ConcreteDescription_>::publishContactWrenches() {
  for (const auto contactKey : ConcreteDescription::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    if (contactForcePublishers_[contactEnum]->getNumSubscribers() == 0 && 
        contactForcePublishersThrottled_[contactEnum].getNumSubscribers() == 0) {

      continue;
    }

    WrenchShm contactWrench = createContactWrenchMessage(contactEnum);
    WrenchRos contactWrenchRos = any_measurements_ros::toRos(contactWrench);

    contactForcePublishers_[contactEnum]->publish(contactWrench, contactWrenchRos);

    if (contactForceThrottledCounterMap_[contactEnum] == contactForceThrottledDecimationMap_[contactEnum]) {
      if (contactForcePublishersThrottled_[contactEnum].getNumSubscribers() > 0u) {
        contactForcePublishersThrottled_[contactEnum].publish(contactWrenchRos); 
      }
      
      contactForceThrottledCounterMap_[contactEnum] = 0;
    }
    ++contactForceThrottledCounterMap_[contactEnum];
  }    
}


template<typename ConcreteDescription_>
bool AnybotGazeboPlugin<ConcreteDescription_>::publishWorker(const any_worker::WorkerEvent& workerEvent) {

  std::chrono::time_point<std::chrono::steady_clock> start, end;
  unsigned long localCounter = 0lu;

  while (!stopUpdating_)
  {
    /* Suspend the thread until new quadruped state message has arrived.
    * We use the conditional variable for that, but we need to check the counter
    * to handle spurious wake-ups.
    */
    std::unique_lock<std::mutex> lock{mutexPublishUpdate_};
    cvUpdate_.wait(lock,[this,localCounter](){
           if (stopUpdating_) return true;
           return (updateCounter_ > localCounter);
    });
    localCounter = updateCounter_;
    // Stop immediately
    if (stopUpdating_) {
      return true;
    }

    //-- Start measuring computation time.
    start = std::chrono::steady_clock::now();
    // Publish to ROS.
    sendRos();

    //-- Measure computation time.
    end = std::chrono::steady_clock::now();
    int64_t elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    const int64_t timeStepNSecs = (int64_t)(publishingTimeStep_*1e9);

    if (elapsedTimeNSecs > timeStepNSecs) {
       MELO_WARN("Computation of publish worker in simulation is not real-time! Computation time: %lf ms", (double)elapsedTimeNSecs*1e-6);
    }
  }
  return true;    
}


template<typename ConcreteDescription_>
void AnybotGazeboPlugin<ConcreteDescription_>::updateCb() {
  std::unique_lock<std::recursive_mutex> lock(gazeboMutex_);

  any_measurements::Time time(lastStartUpdateSimTime_.sec, lastStartUpdateSimTime_.nsec);
  // Update timing.
  const auto startUpdateSysTime = std::chrono::steady_clock::now();
#if (GAZEBO_MAJOR_VERSION >= 8)
  const auto startUpdateSimTime = model_->GetWorld()->SimTime();
#else
  const auto startUpdateSimTime = model_->GetWorld()->GetSimTime();
#endif
  updateSimTimeStep_ = (startUpdateSimTime - lastStartUpdateSimTime_).Double();
  // MELO_INFO_STREAM("startUpdateSimTime: " << startUpdateSimTime.Double());
  lastStartUpdateSimTime_ = startUpdateSimTime;
#if (GAZEBO_MAJOR_VERSION >= 8)
  gazeboTimeStep_ = model_->GetWorld()->Physics()->GetMaxStepSize();
#else
  gazeboTimeStep_ = model_->GetWorld()->GetPhysicsEngine()->GetMaxStepSize();
#endif

  receiveMessages();

  // Read/write simulation states.
  readSimulation();
  writeSimulation();

  if (!stateHasToBePublished())
    return;

  // publish over shm
  publishRobotState();
  publishActuatorReadings();
  publishPoses();
  publishTwist();
  publishContactWrenches();
  publishJointStates();

  updateCounter_++;

  cvUpdate_.notify_all();
  syncMaster_->sync(time, publishingTimeStep_);

  std::chrono::time_point<std::chrono::steady_clock> timeCurrIteration = std::chrono::steady_clock::now();
  const int64_t elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(timeCurrIteration - timePrevIteration_).count();
  iterDurationMs_ = (double)elapsed*1e-6;
  timePrevIteration_ = timeCurrIteration;

  // Update the diagnostics. (Only for update steps with biggest execution time incl. publishing)
  // diagnosticUpdater_.update();
  const auto endUpdateSysTime = std::chrono::steady_clock::now();
  const int64_t elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(endUpdateSysTime - startUpdateSysTime).count();
  updateDurationMs_ = (double)elapsedTimeNSecs*1e-6;
  // updateSysTimeAccumulator_(elapsedTimeNSecs);
  lastStartUpdateSysTime_ = startUpdateSysTime;
  signal_logger::logger->collectLoggerData();
}

template<typename ConcreteDescription_>
void AnybotGazeboPlugin<ConcreteDescription_>::readSimulation() {
  for (const auto actuatorKey : ConcreteDescription::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    // Read joint states.
    auto simJoint = simJoints_[actuatorEnum];

    // Temporary hack for evaluating joint velocities
    // 2016/12/30, RD: Does not seem to be fixed in Gazebo 7 so far.
    // TODO: fix
    //    state.setJointVelocity(simJoint->GetVelocity(0));
    // state.setJointVelocity(simJoint->GetVelocity(0)) seems to be broken
    jointVelocityFilter_[actuatorEnum].setSamplingTime(
        updateSimTimeStep_, jointVelocityFilter_[actuatorEnum].getFilteredValue());

#if (GAZEBO_MAJOR_VERSION >= 8)
    currJointPositions_[actuatorEnum] = simJoint->Position(0);
#else
    currJointPositions_[actuatorEnum] = simJoint->GetAngle(0).Radian();
#endif

    double currVel = jointVelocityFilter_[actuatorEnum].advance(
      (currJointPositions_[actuatorEnum] - prevJointPositions_[actuatorEnum])/updateSimTimeStep_);

    currJointVelocities_[actuatorEnum] = currVel;

    // same for velocities
    jointAccelerationFilter_[actuatorEnum].setSamplingTime(
        updateSimTimeStep_, jointAccelerationFilter_[actuatorEnum].getFilteredValue());

    double currAcc = jointAccelerationFilter_[actuatorEnum].advance(
      (currJointVelocities_[actuatorEnum] - prevJointVelocities_[actuatorEnum])/updateSimTimeStep_);

    currJointAccelerations_[actuatorEnum] = currAcc;

    currJointTorques_[actuatorEnum] = simJoint->GetForce(0);

    updateActuatorReading(actuatorEnum, currJointPositions_[actuatorEnum], currJointVelocities_[actuatorEnum], currJointAccelerations_[actuatorEnum], currJointTorques_[actuatorEnum]);

    // update the previous joint positions and velocities
    prevJointPositions_[actuatorEnum] =  currJointPositions_[actuatorEnum];
    prevJointVelocities_[actuatorEnum] =  currJointVelocities_[actuatorEnum];
  }

  // reset contact forces
  for (const auto contactKey : ConcreteDescription::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
#if (GAZEBO_MAJOR_VERSION >= 8)
    contactForces_[contactEnum] = ignition::math::Vector3d::Zero;
    contactTorques_[contactEnum] = ignition::math::Vector3d::Zero;
    contactPositions_[contactEnum] = ignition::math::Vector3d::Zero;
    contactNormals_[contactEnum] = ignition::math::Vector3d::Zero;
    contactLinearVelocities_[contactEnum] = ignition::math::Vector3d::Zero;
#else
    contactForces_[contactEnum] = math::Vector3::Zero;
    contactTorques_[contactEnum] = math::Vector3::Zero;
    contactPositions_[contactEnum] = math::Vector3::Zero;
    contactNormals_[contactEnum] = math::Vector3::Zero;
    contactLinearVelocities_[contactEnum] = math::Vector3::Zero;
#endif
    contactFlags_[contactEnum] = false;
  }

  // read contact forces
#if (GAZEBO_MAJOR_VERSION >= 8)
  auto contactManager = model_->GetWorld()->Physics()->GetContactManager();
#else
  auto contactManager = model_->GetWorld()->GetPhysicsEngine()->GetContactManager();
#endif

  for (unsigned int contactIndex = 0u; contactIndex < contactManager->GetContactCount(); ++contactIndex) {
    const auto& contact = contactManager->GetContact(contactIndex);
    for (auto contactKey : ConcreteDescription::getContactKeys()) {
      const auto contactEnum = contactKey.getEnum();

      // Returns contact information about Point 1 in contact
      if (collisionLinkNamesToScopedNames_[contactEnum].compare(contact->collision1->GetScopedName()) == 0) {
#if (GAZEBO_MAJOR_VERSION >= 8)
        const ignition::math::Quaterniond orientationCollisionToWorld = contact->collision1->GetLink()->WorldPose().Rot();
#else
        const math::Quaternion orientationCollisionToWorld = contact->collision1->GetLink()->GetWorldPose().rot;
#endif
        contactForces_[contactEnum] = orientationCollisionToWorld.RotateVector(contact->wrench[0].body1Force);
        contactTorques_[contactEnum] = orientationCollisionToWorld.RotateVector(contact->wrench[0].body1Torque);
        contactPositions_[contactEnum] = contact->positions[0];
        contactNormals_[contactEnum] = contact->normals[0];
#if (GAZEBO_MAJOR_VERSION >= 8)
        contactLinearVelocities_[contactEnum] = contact->collision1->WorldLinearVel();
#else
        contactLinearVelocities_[contactEnum] = contact->collision1->GetWorldLinearVel();
#endif
        contactFlags_[contactEnum] = true;
      }

      // Returns contact information about Point 2 in contact
      else if (collisionLinkNamesToScopedNames_[contactEnum].compare(contact->collision2->GetScopedName()) == 0) {
#if (GAZEBO_MAJOR_VERSION >= 8)
        const ignition::math::Quaterniond orientationCollisionToWorld = contact->collision2->GetLink()->WorldPose().Rot();
#else
        const math::Quaternion orientationCollisionToWorld = contact->collision2->GetLink()->GetWorldPose().rot;
#endif
        contactForces_[contactEnum] = orientationCollisionToWorld.RotateVector(contact->wrench[0].body2Force);
        contactTorques_[contactEnum] = orientationCollisionToWorld.RotateVector(contact->wrench[0].body2Torque);
        contactPositions_[contactEnum] = contact->positions[0];
        contactNormals_[contactEnum] = contact->normals[0];
#if (GAZEBO_MAJOR_VERSION >= 8)
        contactLinearVelocities_[contactEnum] = contact->collision2->WorldLinearVel();
#else
        contactLinearVelocities_[contactEnum] = contact->collision2->GetWorldLinearVel();
#endif
        contactFlags_[contactEnum] = true;
      }
    }
  }

  // read robot's pose and velocities
  auto baseLink = model_->GetLink(robotBaseLink_);
  if (!baseLink) {
    MELO_ERROR_STREAM("[" << robotName_ << "GazeboPlugin::readSimulation] Base link " << robotBaseLink_ << " does not exist in Gazebo.");
    return;
  }

#if (GAZEBO_MAJOR_VERSION >= 8)
  robotBaseLinkPose_ = baseLink->WorldPose();
  robotBaseLinkLinearVelocity_ = baseLink->RelativeLinearVel(); // in body coordinate system
  robotBaseLinkAngularVelocity_ = baseLink->RelativeAngularVel(); // in body coordinate system
#else
  robotBaseLinkPose_ = baseLink->GetWorldPose();
  robotBaseLinkLinearVelocity_ = baseLink->GetRelativeLinearVel(); // in body coordinate system
  robotBaseLinkAngularVelocity_ = baseLink->GetRelativeAngularVel(); // in body coordinate system
#endif
}

template<typename ConcreteDescription_>
void AnybotGazeboPlugin<ConcreteDescription_>::writeSimulation() {
  for (const auto actuatorKey : ConcreteDescription::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    simJoints_[actuatorEnum]->SetForce(0, computeActuatorCommand(actuatorEnum));
  }
}

template<typename ConcreteDescription_>
geometry_msgs::Pose AnybotGazeboPlugin<ConcreteDescription_>::createPoseMessage() {
  geometry_msgs::Pose pose;
#if (GAZEBO_MAJOR_VERSION >= 8)
  pose.position.x = robotBaseLinkPose_.Pos().X();
  pose.position.y = robotBaseLinkPose_.Pos().Y();
  pose.position.z = robotBaseLinkPose_.Pos().Z();
  pose.orientation.x = robotBaseLinkPose_.Rot().X();
  pose.orientation.y = robotBaseLinkPose_.Rot().Y();
  pose.orientation.z = robotBaseLinkPose_.Rot().Z();
  pose.orientation.w = robotBaseLinkPose_.Rot().W();
#else
  pose.position.x = robotBaseLinkPose_.pos[0];
  pose.position.y = robotBaseLinkPose_.pos[1];
  pose.position.z = robotBaseLinkPose_.pos[2];
  pose.orientation.x = robotBaseLinkPose_.rot.x;
  pose.orientation.y = robotBaseLinkPose_.rot.y;
  pose.orientation.z = robotBaseLinkPose_.rot.z;
  pose.orientation.w = robotBaseLinkPose_.rot.w;
#endif
  return pose;
}

template<typename ConcreteDescription_>
geometry_msgs::Twist AnybotGazeboPlugin<ConcreteDescription_>::createTwistMessage() {
  geometry_msgs::Twist twist;
  twist.linear.x = robotBaseLinkLinearVelocity_[0];
  twist.linear.y = robotBaseLinkLinearVelocity_[1];
  twist.linear.z = robotBaseLinkLinearVelocity_[2];
  twist.angular.x = robotBaseLinkAngularVelocity_[0];
  twist.angular.y = robotBaseLinkAngularVelocity_[1];
  twist.angular.z = robotBaseLinkAngularVelocity_[2];
  return twist;
}

template<typename ConcreteDescription_>
any_measurements::Wrench AnybotGazeboPlugin<ConcreteDescription_>::createContactWrenchMessage(const typename ConcreteDescription::ContactEnum& contactEnum) {
  any_measurements::Wrench contactWrench;
  contactWrench.time_ = getTime();
  auto force = contactForces_[contactEnum];
  auto torque = contactTorques_[contactEnum];
  contactWrench.wrench_.getForce().x() = force[0]; 
  contactWrench.wrench_.getForce().y() = force[1];
  contactWrench.wrench_.getForce().z() = force[2];

  contactWrench.wrench_.getTorque().x() = torque[0];
  contactWrench.wrench_.getTorque().y() = torque[1];
  contactWrench.wrench_.getTorque().z() = torque[2];

  return contactWrench;
}

template<typename ConcreteDescription_>
bool AnybotGazeboPlugin<ConcreteDescription_>::stateHasToBePublished() {
  const auto period = publishingTimeStep_;
#if (GAZEBO_MAJOR_VERSION >= 8)
  const auto step = model_->GetWorld()->Physics()->GetMaxStepSize();
#else
  const auto step = model_->GetWorld()->GetPhysicsEngine()->GetMaxStepSize();
#endif
  if (period == 0.0)
    return true;
#if (GAZEBO_MAJOR_VERSION >= 8)
  const auto fraction = std::fmod((model_->GetWorld()->SimTime()).Double() + (step / 2.0), period);
#else
  const auto fraction = std::fmod((model_->GetWorld()->GetSimTime()).Double() + (step / 2.0), period);
#endif
  return (fraction >= 0.0) && (fraction < step);
}


template<typename ConcreteDescription_>
void AnybotGazeboPlugin<ConcreteDescription_>::publishPoses() {
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.stamp = ros::Time(lastStartUpdateSimTime_.sec, lastStartUpdateSimTime_.nsec);
  pose.pose.pose = createPoseMessage();

  if (simulateEstimator_) {
    if (poseInOdomPublisher_) {
      if (poseInOdomPublisher_->getNumSubscribers() > 0u) {
        currPoseInOdomMsg_ = pose;
        currPoseInOdomMsg_.header.frame_id = frameOdometry_;
        currPoseInOdomMsg_.pose.pose.position.x -= frameOdometryOffsetX_;
        currPoseInOdomMsg_.pose.pose.position.y -= frameOdometryOffsetY_;
        currPoseInOdomMsg_.pose.pose.position.z -= frameOdometryOffsetZ_;
        poseInOdomPublisher_->publish(any_measurements_ros::fromRos(currPoseInOdomMsg_), currPoseInOdomMsg_);
      }
    }
  
    // Covariance left at 0.0;
    if (poseInOdomPublisherThrottled_.getNumSubscribers() > 0u) {
      currPoseInOdomMsg_ = pose;
      currPoseInOdomMsg_.header.frame_id = frameOdometry_;
      currPoseInOdomMsg_.pose.pose.position.x -= frameOdometryOffsetX_;
      currPoseInOdomMsg_.pose.pose.position.y -= frameOdometryOffsetY_;
      currPoseInOdomMsg_.pose.pose.position.z -= frameOdometryOffsetZ_;
  
      if (poseInOdomThrottledCounter_ == poseInOdomThrottledDecimation_) {
        if (poseInOdomPublisherThrottled_.getNumSubscribers() > 0u) {
          poseInOdomPublisherThrottled_.publish(currPoseInOdomMsg_);
        }
        
        poseInOdomThrottledCounter_ = 0;
      }
      ++poseInOdomThrottledCounter_;
    }
  }
  
  if (simulateLocalizer_ && localizerIsEnabled_) {
    if (localizationPublisherCounter_ == localizationPublisherDecimation_) {
      if (localizationPublisher_.getNumSubscribers() > 0u ||
          localizationPublisher_.isLatched()) {
        geometry_msgs::PoseWithCovarianceStamped mapPoseInOdom;
        mapPoseInOdom.header.stamp = ros::Time(lastStartUpdateSimTime_.sec, lastStartUpdateSimTime_.nsec);
        mapPoseInOdom.header.frame_id = frameOdometry_;
        mapPoseInOdom.pose.pose.position.x = -frameOdometryOffsetX_;
        mapPoseInOdom.pose.pose.position.y = -frameOdometryOffsetY_;
        mapPoseInOdom.pose.pose.position.z = -frameOdometryOffsetZ_;
        mapPoseInOdom.pose.pose.orientation.w = 1.0;
        mapPoseInOdom.pose.pose.orientation.x = 0.0;
        mapPoseInOdom.pose.pose.orientation.y = 0.0;
        mapPoseInOdom.pose.pose.orientation.z = 0.0;
        localizationPublisher_.publish(mapPoseInOdom);
      }
      localizationPublisherCounter_ = 0;
    }
    ++localizationPublisherCounter_;
  }
}

template<typename ConcreteDescription_>
void AnybotGazeboPlugin<ConcreteDescription_>::publishTwist() {
  if (!simulateEstimator_) {
    return;
  }

  if (twistPublisher_) {
    if (twistPublisher_->getNumSubscribers() == 0 && 
        twistPublisherThrottled_.getNumSubscribers() == 0) {
      return;
    }
  }

  currTwistMsg_.header.stamp = ros::Time(lastStartUpdateSimTime_.sec, lastStartUpdateSimTime_.nsec);
  currTwistMsg_.header.frame_id = frameBase_;
  currTwistMsg_.twist.twist = createTwistMessage();

  // Covariance left at 0.0;
  if (twistPublisher_) {
    twistPublisher_->publish(any_measurements_ros::fromRos(currTwistMsg_), currTwistMsg_);
  }

  if (twistThrottledCounter_ == twistThrottledDecimation_) {
    if (twistPublisherThrottled_.getNumSubscribers() > 0u) {
      twistPublisherThrottled_.publish(currTwistMsg_);
    }

    twistThrottledCounter_ = 0;
  }
  ++twistThrottledCounter_;
}

template<typename ConcreteDescription_>
void AnybotGazeboPlugin<ConcreteDescription_>::initJointStateMessage() {
  constexpr auto numJoints = ConcreteDescription::getActuatorsDimension();
  currJointstateMsg_.name.resize(numJoints);
  currJointstateMsg_.position.resize(numJoints);
  currJointstateMsg_.velocity.resize(numJoints);
  currJointstateMsg_.acceleration.resize(numJoints);
  currJointstateMsg_.effort.resize(numJoints);
  for (const auto actuatorKey : ConcreteDescription::getActuatorKeys()) {
    const auto actuatorId = actuatorKey.getId();
    currJointstateMsg_.name[actuatorId] = actuatorKey.getName();
  }
}

template<typename ConcreteDescription_>
void AnybotGazeboPlugin<ConcreteDescription_>::publishJointStates() {
  if (!simulateEstimator_)
    return;

  if (jointStatesPublisher_->getNumSubscribers() == 0) {
    return;
  }

  currJointstateMsg_.header.stamp = ros::Time(lastStartUpdateSimTime_.sec, lastStartUpdateSimTime_.nsec);
  for (const auto actuatorKey : ConcreteDescription::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    const auto actuatorId = actuatorKey.getId();
    currJointstateMsg_.position[actuatorId] = currJointPositions_[actuatorEnum];
    currJointstateMsg_.velocity[actuatorId] = currJointVelocities_[actuatorEnum];
    currJointstateMsg_.acceleration[actuatorId] = currJointAccelerations_[actuatorEnum];
    currJointstateMsg_.effort[actuatorId] = currJointTorques_[actuatorEnum];
  }

  JointStateShm jointStateShm;
  any_measurements_ros::fromRos(currJointstateMsg_, jointStateShm);

  jointStatesPublisher_->publish(jointStateShm, currJointstateMsg_);
}

template<typename ConcreteDescription_>
void AnybotGazeboPlugin<ConcreteDescription_>::setRobotPose(const geometry_msgs::Pose& pose) {
  std::unique_lock<std::recursive_mutex> lock(gazeboMutex_);
  
  // see:
  // https://github.com/ros-simulation/gazebo_ros_pkgs/blob/jade-devel/gazebo_ros/src/gazebo_ros_api_plugin.cpp
  auto baseLink = model_->GetLink(robotBaseLink_);
#if (GAZEBO_MAJOR_VERSION >= 8)
  const ignition::math::Pose3d poseGazebo(
      ignition::math::Vector3d(
          pose.position.x,
          pose.position.y,
          pose.position.z),
      ignition::math::Quaterniond(
          pose.orientation.w,
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z));
    model_->SetWorldPose(poseGazebo);
    model_->SetLinearVel(ignition::math::Vector3d());
    model_->SetAngularVel(ignition::math::Vector3d());
#else
  const math::Pose poseGazebo(
      math::Vector3(
          pose.position.x,
          pose.position.y,
          pose.position.z),
      math::Quaternion(
          pose.orientation.w,
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z));
  model_->SetWorldPose(poseGazebo);
  model_->SetLinearVel(math::Vector3());
  model_->SetAngularVel(math::Vector3());
#endif
  Reset();
}

template<typename ConcreteDescription_>
bool AnybotGazeboPlugin<ConcreteDescription_>::setRobotPoseCb(
    any_gazebo_msgs::SetRobotPoseRequest& req,
    any_gazebo_msgs::SetRobotPoseResponse& res)
{
  setRobotPose(req.pose);
  return true;
}

template<typename ConcreteDescription_>
void AnybotGazeboPlugin<ConcreteDescription_>::toggleLocalizer(const bool enable) {
  localizerIsEnabled_ = enable;
}

template<typename ConcreteDescription_>
bool AnybotGazeboPlugin<ConcreteDescription_>::toggleLocalizerCb(
    any_msgs::ToggleRequest& req,
    any_msgs::ToggleResponse& res)
{
  toggleLocalizer(req.enable);
  res.success = true;
  return true;
}

}
