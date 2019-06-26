//
// Created by tim on 07.10.18.
//

#ifndef WACO_WACOSTATEESTIMATORINTERFACE_H
#define WACO_WACOSTATEESTIMATORINTERFACE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <tf/transform_broadcaster.h>
#include <mutex>
#include <any_state_estimator/AnyStateEstimator.hpp>
#include <smb_description_ros/smb_description_ros.hpp>
#include <smb_description_ros/ContainersRos.hpp>
#include <smb_confusor/BaseMotionModel.h>
#include <smb_confusor/BmmPropagator.h>
#include <smb_msgs/SmbState.h>
//#include <smb_confusor/SmbConFusor.hpp>

#include <confusion/utilities/ImuPropagator.h>
#include <confusion/utilities/ros_conversions.h>


namespace smb_state_estimator {

/**
 * @brief      Interface between the smb state estimator and the robot controller
 */
class SmbStateEstimator
    : public any_state_estimator::AnyStateEstimator<smb_description_ros::SmbContainersRos> {

 public:
  using SmbDescription = smb_model::SmbModel::SmbDescription;

  SmbStateEstimator(any_node::Node::NodeHandlePtr nh);

 protected:
  /**
 * @brief      Resets the state estimator to pose. This is not used!
 *
 * @param[in]  pose  The initial pose
 *
 * @return     true if successful
 */
  bool resetEstimator(const kindr::HomTransformQuatD &pose) { return false; }

  /**
   * @brief      Resets the state estimator at the current pose. This is not used!
   *
   * @return     true if successful
   */
  bool resetEstimatorHere() { return false; }

  /**
   * @brief      Receives measurements from your subscriber. Gets called before preprocessMeasurements.
   */
  void receiveMeasurements();

  /**
   * @brief      Preprocesses the measurements. Gets called before advancing the estimator
   */
  void preprocessMeasurements();

  /**
   * @brief      Advances the estimator by updating the filter. Gets called
   *             before setting the estimator output
   */
  void advanceEstimator();

  /**
   * @brief      Prepare the output to be published. Gets called before
   *             publishing
   */
  void setOutput();

  /**
   * @brief      Publishes desired output via cosmo
   */
  void publish();

  /**
   * @brief      Initializes further objects
   */
  void initImpl() { }

  /**T_i_b_
   * @brief      Initializes messages in derived class
   */
  void initializeMessages() { /*do nothing*/ }

  /**
   * @brief      Reads parameters from the parameter server
   */
  void readParameters();

  /**
   * @brief      Initializes additional publishers in the derived class
   */
  void initializePublishers() { /*do nothing*/ }

  /**
   * @brief      Initializes additional subscribers in the derived class
   */
  void initializeSubscribers() { /*do nothing*/ }

  /**
   * @brief      Initializes services
   */
  void advertiseServices() { /*do nothing*/ }

  /**
   * @brief      Adds variables to the signal logger
   */
  void addVariablesToLog() { /*do nothing*/ }

  /**
   * @brief      Publishes data over ros
   */
  void publishRos();

  void stateEstimateCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

 private:
  std::string configFile_ = "/config/smb_calibration.cfg"; // In smb_confusor repo

  any_measurements::Time currentTime_;
  double lastStateEstimatorTime_ = 0.0;
  bool stateEstimateValid_ = false;
  double stateEstimatorDropoutTime_;

  smb_msgs::SmbState smbMsgRos_;

  confusion::ImuPropagator imuPropagator_;
  //todo temp dummy objects to pass to ImuMeas's. Need to change the ImuMeas class to avoid having to do this.
  confusion::ImuCalibration imuCalibration_;
  Eigen::Matrix2d gravityRot_;

  smb_confusor::BmmPropagator bmmPropagator_;
  BaseMotionModelCalibration bmmCalibration_;
  BaseMotionModelParameters baseState_now_;

  confusion::ImuStateParameters imuState_now_;
  bool stateEstimateReceived_ = false;

  confusion::Pose<double> T_i_b_; //Calibrated offset from the IMU to the base of the robot

  ros::Subscriber stateEstimateSubscriber_;
  ros::Subscriber imuSubscriber_;
  ros::Publisher basePosePublisher_; //todo Is there a nicer way for the base motion planner to get this?
  tf::TransformBroadcaster tfBroadcaster_;

  //IMU communication
  //todo Make a dedicated any_node? But then there will additional latency.
  size_t imuDriverQueueSize_ = 400; //Size of the imu driver's internal buffer
  bool imuConnected_ = false;

  std::mutex publishRosMtx_;
  confusion::Pose<double> _T_w_b_;
  confusion::Pose<double> _T_w_i_;
  confusion::Pose<double> _T_w_b_odom_;
};

} // namespace smb_state_estimator

#endif //WACO_WACOSTATEESTIMATORINTERFACE_H
