/*!
 * @file    AnyStateEstimator.hpp
 * @author  Markus Staeuble
 * @date    Apr, 2018
 */

#pragma once

// ros
#include <ros/ros.h>

// ros messages
#include <any_state_estimator_msgs/ResetStateEstimator.h>

// ros srvs
#include <std_srvs/Trigger.h>

// any measurements
#include "any_measurements/Time.hpp"
#include "any_measurements_ros/any_measurements_ros.hpp"

// cosmo
#include <cosmo/SyncSlave.hpp>
#include <cosmo/SyncMaster.hpp>
#include <cosmo_node/Node.hpp>
#include <cosmo_ros/cosmo_ros.hpp>

// signal logger
#include <signal_logger/signal_logger.hpp>

// kindr
#include <kindr/Core>

// message logger
#include <message_logger/message_logger.hpp>

// stl
#include <chrono>
#include <mutex>

namespace any_state_estimator {

/**
 * @brief      Base class for a generic state estimator using an imu and actuator
 *             readings as input and publishes a robot state as output
 *
 * @tparam     RobotContainersRos_  romo::RobotContainerRos
 */
template<typename RobotContainersRos_>
class AnyStateEstimator : public cosmo_node::Node {
 public:
  using RobotContainer = typename RobotContainersRos_::ConcreteContainers;
  using RobotContainerRos = RobotContainersRos_;

  using RobotStateContainer = typename RobotContainerRos::RobotStateRos;
  using ActuatorReadingsContainer = typename RobotContainerRos::ActuatorReadingsRos;
  using ImuContainer = typename RobotContainerRos::ImuRos;

  using ImuShm = typename ImuContainer::type;
  using ImuRos = typename ImuContainer::msgType;
  using RobotStateShm = typename RobotStateContainer::type;
  using RobotStateRos = typename RobotStateContainer::msgType;
  using ActuatorReadingsShm = typename ActuatorReadingsContainer::type;
  using ActuatorReadingsRos = typename ActuatorReadingsContainer::msgType;

  AnyStateEstimator() = delete;

  /**
   * @brief      Constructor
   *
   * @param[in]  nh    NodeHandle
   */
  explicit AnyStateEstimator(any_node::Node::NodeHandlePtr nh);

  ~AnyStateEstimator() override = default;

  bool init() override;
  void preCleanup() override;
  void cleanup() override;

  /**
   * @brief      Is called periodically to update the estimator when running standalone
   *
   * @param[in]  event  The event
   *
   * @return     true if successful
   */
  bool update(const any_worker::WorkerEvent& event);

 protected:

  /**
   * @brief      Resets the state estimator to pose
   *
   * @param[in]  pose  The initial pose
   *
   * @return     true if successful
   */
  virtual bool resetEstimator(const kindr::HomTransformQuatD& pose) = 0;

  /**
   * @brief      Resets the state estimator at the current pose
   *
   * @return     true if successful
   */
  virtual bool resetEstimatorHere() = 0;

  /**
   * @brief      Receives measurements from your subscriber. Gets called before preprocessMeasurements.
   */
  virtual void receiveMeasurements() = 0;

  /**
   * @brief      Preprocesses the measurements. Gets called before advancing the estimator
   */
  virtual void preprocessMeasurements() = 0;

  /**
   * @brief      Advances the estimator by updating the filter. Gets called
   *             before setting the estimator output
   */
  virtual void advanceEstimator() = 0;

  /**
   * @brief      Prepare the output to be published. Gets called before
   *             publishing
   */
  virtual void setOutput() = 0;

  /**
   * @brief      Publishes desired output via cosmo
   */
  virtual void publish() = 0;
  
  /**
   * @brief      Initializes further objects
   */
  virtual void initImpl() { /*do nothing*/ }

  /**
   * @brief      Initializes messages in derived class
   */
  virtual void initializeMessages() { /*do nothing*/ }

  /**
   * @brief      Reads parameters from the parameter server
   */
  virtual void readParameters() { /*do nothing*/ }

  /**
   * @brief      Initializes additional publishers in the derived class
   */
  virtual void initializePublishers() { /*do nothing*/ }

  /**
   * @brief      Initializes additional subscribers in the derived class
   */
  virtual void initializeSubscribers() { /*do nothing*/ }

  /**
   * @brief      Initializes services 
   */
  virtual void advertiseServices() { /*do nothing*/ }

  /**
   * @brief      Adds variables to the signal logger
   */
  virtual void addVariablesToLog() { /*do nothing*/ }

  /**
   * @brief      Publishes data over ros
   */
  virtual void publishRos() { /*do nothing*/ }


  /**
   * @brief      The main update function of the estimator. Calls
   *             receiveMeasurements, preprocessMeasurements, advanceEstimator,
   *             setOutput and publish
   */
  void update();

  /**
   * @brief      Starts workers for the synced update and ros publishing
   */
  void startWorkers();

  /**
   * @brief      Calls the update method and afterwards sleeps until the next
   *             iteration
   *
   * @param[in]  event  The event
   *
   * @return     true if successful
   */
  bool updateSynced(const any_worker::WorkerEvent& event);

  /**
   * @brief      Publishes data over ros via publishRos() and afterwards sleeps until the next update has finished
   *
   * @param[in]  event  The event
   *
   * @return     true if successful
   */
  bool publishRos(const any_worker::WorkerEvent& event);

  /**
   * @brief      Handles incoming imu messages
   *
   * @param[in]  msg   The message
   */
  void imuCallback(const ImuShm& msg);

  /**
   * @brief      Handles incoming actuator reading messages
   *
   * @param[in]  msg   The message
   */
  void actuatorReadingsCallback(const ActuatorReadingsShm& msg);

  /**
   * @brief      Wraps reset estimator, is called by a ros service
   *
   * @param      req   The reset request
   * @param      res   The reset response
   *
   * @return     true if successful
   */
  bool resetService(any_state_estimator_msgs::ResetStateEstimator::Request& req, any_state_estimator_msgs::ResetStateEstimator::Response& res);

  /**
   * @brief      Wraps reset estimator here, is called by a ros service
   *
   * @param      req   The reset request
   * @param      res   The reset response
   *
   * @return     true if successful
   */
  bool resetHereService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

 protected:
  double timeStep_{0.0};
  double initializationDuration_{0.0};
  double iterDurationMs_{0.0};
  double updateDurationMs_{0.0};

  RobotStateShm estimatedState_;
  ImuShm imu_;
  ActuatorReadingsShm actuatorReadings_;

  std::string syncSlaveName_;
  std::string syncMasterName_;

  std::unique_ptr<cosmo::SyncSlave> syncSlave_;
  std::unique_ptr<cosmo::SyncMaster> syncMaster_;

  const std::chrono::microseconds receiveMaxLockTime_{std::chrono::microseconds{50}};
  const std::chrono::microseconds sendMaxLockTime_{std::chrono::microseconds{50}};
  
  cosmo_ros::SubscriberRosPtr<ImuShm, ImuRos, ImuContainer::template ConversionTrait> imuSubscriber_;
  cosmo_ros::SubscriberRosPtr<ActuatorReadingsShm, ActuatorReadingsRos, ActuatorReadingsContainer::template ConversionTrait> actuatorReadingsSubscriber_;

  // Publishers
  cosmo_ros::PublisherRosPtr<RobotStateShm, RobotStateRos, RobotStateContainer::template ConversionTrait> robotStatePublisher_;

  // Services
  ros::ServiceServer resetService_;
  ros::ServiceServer resetHereService_;

  bool isSimulation_;
  bool isStandalone_;
  
  std::condition_variable_any cvUpdate_;
  std::atomic<bool> stopUpdating_{true};
  std::mutex mutexUpdate_;

  //mutex to lock non-timecritical roscallbacks (e.g. reset services) and the se update
  std::mutex mutexStateEstimator_;

  std::chrono::time_point<std::chrono::steady_clock> timePrevIteration_;
  std::atomic<unsigned long> updateCounter_{0lu};
};

} /* namespace any_state_estimator */

#include <any_state_estimator/AnyStateEstimator.tpp>
