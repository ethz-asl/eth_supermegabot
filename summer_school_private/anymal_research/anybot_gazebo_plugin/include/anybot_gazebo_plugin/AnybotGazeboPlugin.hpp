/** \file gazebo_ros_control.h
 \brief This file defines the GazeboRosControl class which interfaces ROS
 StarlETH locomotion controller with Gazebo simulator.
 */

#pragma once

// c++
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <atomic>
#include <condition_variable>

// gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

// ros
#include <ros/ros.h>
#include <ros/callback_queue.h>

// geometry msgs
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

// urdf
#include <urdf/model.h>

// std_utils
#include <std_utils/containers/EnumArray.hpp>

// any msgs
#include <any_msgs/State.h>
#include <any_msgs/Toggle.h>
#include <any_msgs/ExtendedJointState.h>

// any gazebo msgs
#include <any_gazebo_msgs/SetRobotPose.h>

// any measurements
#include <any_measurements/ExtendedJointState.hpp>
#include <any_measurements/PoseWithCovariance.hpp>
#include <any_measurements/TwistWithCovariance.hpp>
#include <any_measurements/Wrench.hpp>
#include <any_measurements_ros/any_measurements_ros.hpp>

// robot utils
#include <robot_utils/filters/filters.hpp>

// message logger
#include <message_logger/message_logger.hpp>

// cosmo
#include <cosmo_ros/cosmo_ros.hpp>
#include <cosmo/SyncMaster.hpp>

// signal logger
#include <signal_logger/signal_logger.hpp>

namespace gazebo {

template<typename ConcreteDescription_>
class AnybotGazeboPlugin : public ModelPlugin
{
protected:
  using ConcreteDescription = ConcreteDescription_;
  using PoseWithCovarianceShm = any_measurements::PoseWithCovariance;
  using PoseWithCovarianceRos = geometry_msgs::PoseWithCovarianceStamped;
  using TwistShm = any_measurements::TwistWithCovariance;
  using TwistRos = geometry_msgs::TwistWithCovarianceStamped;
  using JointStateShm = std::array<any_measurements::ExtendedJointState, ConcreteDescription::getActuatorsDimension()>;
  using JointStateRos = any_msgs::ExtendedJointState;
  using WrenchShm = any_measurements::Wrench;
  using WrenchRos = geometry_msgs::WrenchStamped;

  using ActuatorEnum = typename ConcreteDescription::ActuatorEnum;
  using ContactEnum = typename ConcreteDescription::ContactEnum;

  template<typename ValueType_>
  using JointEnumContainer = std_utils::EnumArray<ActuatorEnum, ValueType_>;

  template<typename ValueType_>
  using ContactEnumContainer = std_utils::EnumArray<ContactEnum, ValueType_>;

public:
  // Constructor.
  explicit AnybotGazeboPlugin(const std::string& robotName)
  : robotName_(robotName)
  {}

  // Destructor.
  virtual ~AnybotGazeboPlugin();

  // Implements Gazebo virtual load function.
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

  // Overrides Gazebo init function.
  virtual void Init();

  // Overrides Gazebo reset function.
  virtual void Reset();

  // Set robot pose service callback.
  virtual bool setRobotPoseCb(
      any_gazebo_msgs::SetRobotPoseRequest& req,
      any_gazebo_msgs::SetRobotPoseResponse& res);

  // Toggle localizer server callback.
  virtual bool toggleLocalizerCb(
      any_msgs::ToggleRequest& req,
      any_msgs::ToggleResponse& res);

protected:
  // Interface methods
   
  // Reads parameters from the parameter server.
  virtual void readParameters() = 0;

  // Inits joint structures.
  virtual void initJointStructures() = 0;

  // Inits the ROS subscribers.
  virtual void initSubscribers() = 0;
  // Inits the ROS publishers.
  virtual void initPublishers() = 0;
  // Inits the ROS services.
  virtual void initServices() = 0;

  // Publishes robot state over ROS.
  virtual void publishRobotState() = 0;

  // Publishes actuator readings over ROS.
  virtual void publishActuatorReadings() = 0;

  virtual double computeActuatorCommand(const ActuatorEnum& actuatorEnum) = 0;

  virtual void updateActuatorReading(
    const ActuatorEnum& actuatorEnum,
    const double jointPosition,
    const double jointVelocity,
    const double jointAcceleration,
    const double jointTorque) = 0;

  // Member methods
  // Callback on gazebo simulation iteration
  virtual void updateCb();
  
  // Retrieves URDF robot description from ROS parameter server.
  virtual std::string getUrdfRobotDescription(const std::string& paramName) const;

  // Inits Gazebo contact manager.
  virtual void initContactManager();

  // Receive messages over shm
  virtual void receiveMessages(){}

  // Publishes messages over ROS
  virtual void sendRos();

  // Publishes contact wrenches over ROS.
  virtual void publishContactWrenches();

  // ROS publisher worker
  virtual bool publishWorker(const any_worker::WorkerEvent& workerEvent);

  // Reads simulation state.
  virtual void readSimulation();

  // Writes simulation state.
  virtual void writeSimulation();

  // Creates ROS pose message.
  virtual geometry_msgs::Pose createPoseMessage();

  // Creates ROS twist message.
  virtual geometry_msgs::Twist createTwistMessage();

  // Creates ROS contact wrench message.
  virtual WrenchShm createContactWrenchMessage(const ContactEnum& contactEnum);

  // Checks if state should be published
  virtual bool stateHasToBePublished();

  // Publishes pose over ROS.
  virtual void publishPoses();

  // Publishes twist over ROS.
  virtual void publishTwist();

  virtual void initJointStateMessage();

  // Publishes joint states over ROS.
  virtual void publishJointStates();

  // Set robot pose.
  virtual void setRobotPose(const geometry_msgs::Pose& pose);

  // Toggle localizer.
  virtual void toggleLocalizer(const bool enable);

  any_measurements::Time getTime() {
    return any_measurements::Time(lastStartUpdateSimTime_.sec, lastStartUpdateSimTime_.nsec);
  }

  // Checks whether publishers are initialized
  void checkPublishers();

  // Name of the robot.
  std::string robotName_;
  // Ensures gazebo methods are called sequentially
  std::recursive_mutex gazeboMutex_;
  // ROS robot description parameter name.
  std::string robotDescriptionParamName_;
  // Robot base link.
  std::string robotBaseLink_;
  // ROS robot description URDF string.
  std::string robotDescriptionUrdfString_;
  // ROS robot description URDF model.
  urdf::Model robotDescriptionUrdfModel_;
  // Simulate the state estimator including localization. If true, this plugin will publish all estimator output containing ground truth data.
  bool simulateEstimator_ = false;
  // Simulate the localizer. If true, this plugin will publish localizer output containing ground truth data.
  bool simulateLocalizer_ = false;
  // If the state estimator is not simulated and this is set to true, the plugin will still publish a groundtruth quadruped state
  bool publishGroundtruth_ = false;
  // Gazebo time step.
  double gazeboTimeStep_ = 0.0;
  // Time step for publishing simulation state.
  double publishingTimeStep_ = 0.0;
  // Simulation time stamp taken at the start of the last updateCb() function call.
  common::Time lastStartUpdateSimTime_;
  // System time stamp taken at the start of the last updateCb() function call.
  std::chrono::time_point<std::chrono::steady_clock> lastStartUpdateSysTime_;
  // Current inter-update simulation time step.
  double updateSimTimeStep_ = 0.0;

  // Types of the joints to be controlled.
  JointEnumContainer<int> jointTypes_;
  // Default positions of the joints (from parameters).
  JointEnumContainer<double> jointPositionsDefault_;
  // Reset positions of the joints.
  JointEnumContainer<double> jointPositionsReset_;
  // Lower limits of the joints to be controlled.
  JointEnumContainer<double> jointPositionLimitsLow_;
  // Upper limits of the joints to be controlled.
  JointEnumContainer<double> jointPositionLimitsHigh_;
  // Maximum velocities of the joints to be controlled.
  JointEnumContainer<double> jointVelocityLimits_;
  // Maximum torques of the joints to be controlled.
  JointEnumContainer<double> jointTorqueLimits_;
  // Current joint positions
  JointEnumContainer<double> currJointPositions_;
  // Current joint velocity
  JointEnumContainer<double> currJointVelocities_;
  // Current joint accelerations
  JointEnumContainer<double> currJointAccelerations_;
  // Current joint torques
  JointEnumContainer<double> currJointTorques_;
  // Previous joint positions, used to estimate joint velocity
  JointEnumContainer<double> prevJointPositions_;
  // Previous joint velocity
  JointEnumContainer<double> prevJointVelocities_;
  // Pointers to the simulated joints to be controlled.
  JointEnumContainer<gazebo::physics::JointPtr> simJoints_;
  // Since Gazebo outputs wrong joint velocities, we use these filters to derive velocities and accelerations from the joint positions.
  JointEnumContainer<robot_utils::FirstOrderFilterD> jointVelocityFilter_;
  JointEnumContainer<robot_utils::FirstOrderFilterD> jointAccelerationFilter_;

  // Mapping between collision link names and publisher names.
  ContactEnumContainer<std::string> collisionLinkNamesToPublisherNames_;
  // Mapping between collision link names and scoped names.
  ContactEnumContainer<std::string> collisionLinkNamesToScopedNames_;
  // Contact flags.
  ContactEnumContainer<bool> contactFlags_;
#if (GAZEBO_MAJOR_VERSION >= 8)
  // Contact forces.
  ContactEnumContainer<ignition::math::Vector3d> contactForces_;
  // Contact torques.
  ContactEnumContainer<ignition::math::Vector3d> contactTorques_;
  // Contact positions.
  ContactEnumContainer<ignition::math::Vector3d> contactPositions_;
  // Contact normals.
  ContactEnumContainer<ignition::math::Vector3d> contactNormals_;
  // Contact linear velocities.
  ContactEnumContainer<ignition::math::Vector3d> contactLinearVelocities_;
#else
    // Contact forces.
  ContactEnumContainer<math::Vector3> contactForces_;
  // Contact torques.
  ContactEnumContainer<math::Vector3> contactTorques_;
  // Contact positions.
  ContactEnumContainer<math::Vector3> contactPositions_;
  // Contact normals.
  ContactEnumContainer<math::Vector3> contactNormals_;
  // Contact linear velocities.
  ContactEnumContainer<math::Vector3> contactLinearVelocities_;
#endif
  // Friction coefficients.
  ContactEnumContainer<double> frictionCoefficients_;
  // Restitution coefficients.
  ContactEnumContainer<double> restitutionCoefficients_;

  // ROS node handle.
  ros::NodeHandle* nodeHandle_ = nullptr;

  // ROS publisher worker
  std::unique_ptr<any_worker::Worker> publishWorker_;
  std::condition_variable cvUpdate_;
  std::mutex mutexPublishUpdate_;
  std::atomic<bool> stopUpdating_;
  std::atomic<unsigned long> updateCounter_;

  // Sends sync signal every update step
  std::unique_ptr<cosmo::SyncMaster> syncMaster_;

  // COSMO pose publishers
  cosmo_ros::PublisherRosPtr<PoseWithCovarianceShm, PoseWithCovarianceRos, any_measurements_ros::ConversionTraits> poseInOdomPublisher_;
  cosmo_ros::PublisherRosPtr<TwistShm, TwistRos, any_measurements_ros::ConversionTraits> twistPublisher_;
  // COSMO joint states publisher.
  cosmo_ros::PublisherRosPtr<JointStateShm, JointStateRos, any_measurements_ros::ConversionTraits> jointStatesPublisher_;
  // COSMO contact force publishers.
  ContactEnumContainer<cosmo_ros::PublisherRosPtr<WrenchShm, WrenchRos, any_measurements_ros::ConversionTraits>> contactForcePublishers_;

  // Throttled ROS publishers
  ros::Publisher poseInOdomPublisherThrottled_;
  unsigned int poseInOdomThrottledCounter_ = 0;
  unsigned int poseInOdomThrottledDecimation_ = 0;  

  ros::Publisher twistPublisherThrottled_;
  unsigned int twistThrottledCounter_ = 0;
  unsigned int twistThrottledDecimation_ = 0;

  ros::Publisher localizationPublisher_;
  unsigned int localizationPublisherCounter_ = 0;
  unsigned int localizationPublisherDecimation_ = 0;
  bool localizerIsEnabled_ = true;

  // Contact force publisher 
  ContactEnumContainer<ros::Publisher> contactForcePublishersThrottled_;
  ContactEnumContainer<unsigned int> contactForceThrottledCounterMap_;
  ContactEnumContainer<unsigned int> contactForceThrottledDecimationMap_;

  // ROS set robot pose service.
  ros::ServiceServer setRobotPoseServer_;

  // ROS service server to toggle the localizer.
  ros::ServiceServer toggleLocalizerServer_;

  // Frame names.
  std::string frameBase_;
  std::string frameOdometry_;
  std::string frameWorld_;

  // ROS messages 
  geometry_msgs::PoseWithCovarianceStamped currPoseInOdomMsg_;
  geometry_msgs::TwistWithCovarianceStamped currTwistMsg_;
  any_msgs::ExtendedJointState currJointstateMsg_;
  
  // Frame offsets.
  double frameOdometryOffsetX_ = 0.0;
  double frameOdometryOffsetY_ = 0.0;
  double frameOdometryOffsetZ_ = 0.0;

  // Linear velocity threshold for slipping contact.
  double contactLinearVelocityThreshold_ = 0.0;

  // Model.
  physics::ModelPtr model_;
  // World update event.
  event::ConnectionPtr updateConnection_;
#if (GAZEBO_MAJOR_VERSION >= 8)
  // Robot base link pose (transforms vectors from base to world).
  ignition::math::Pose3d robotBaseLinkPose_;
  // Robot base link linear velocity in world frame.
  ignition::math::Vector3d robotBaseLinkLinearVelocity_;
  // Robot base link angular velocity in base frame.
  ignition::math::Vector3d robotBaseLinkAngularVelocity_;
#else
  // Robot base link pose (transforms vectors from base to world).
  math::Pose robotBaseLinkPose_;
  // Robot base link linear velocity in world frame.
  math::Vector3 robotBaseLinkLinearVelocity_;
  // Robot base link angular velocity in base frame.
  math::Vector3 robotBaseLinkAngularVelocity_;
#endif

  std::chrono::time_point<std::chrono::steady_clock> timePrevIteration_ = std::chrono::steady_clock::now();
  double iterDurationMs_ = 0.0;
  double updateDurationMs_ = 0.0;  
};

}

#include <anybot_gazebo_plugin/AnybotGazeboPlugin.tpp>
