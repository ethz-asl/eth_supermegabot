#pragma once

// smb model
#include <smb_model/SmbModel.hpp>

// anybot gazebo plugin
#include <anybot_gazebo_plugin/AnybotGazeboPlugin.hpp>

// smb msgs
#include <smb_msgs/SmbState.h>

// cosmo ros
#include <cosmo_ros/cosmo_ros.hpp>

// smb description
#include <smb_description/SmbDescription.hpp>

// smb description ros
#include <smb_description_ros/smb_description_ros.hpp>
#include <romo_std/common/container_utils.hpp>
namespace gazebo {

  // using SeaGazeboPluginBase = SeabotGazeboPlugin<quadruped_model::QuadrupedModel::QuadrupedDescription>;
  using SmbGazeboPluginBase = AnybotGazeboPlugin<smb_model::SmbModel::SmbDescription>;

// The SmbGazeboPlugin class interfaces ANYmal with the Gazebo simulator.
  class SmbGazeboPlugin : public SmbGazeboPluginBase
  {
  public:
    using SD = smb_model::SmbModel::SmbDescription;
    using SmbStateShm = smb_description::SmbState;
    using SmbStateRos = smb_msgs::SmbState;

    template<typename Msg_, typename MsgRos_>
    using SmbReadingConversionTrait = smb_description_ros::ConversionTraits<Msg_, MsgRos_>;

    template<typename Msg_, typename MsgRos_>
    using SmbCommandsConversionTrait = smb_description_ros::ConversionTraits<Msg_, MsgRos_>;

    using SmbCommands = smb_description::SmbCommands;
    using SmbReadings = smb_description::SmbReadings;

    using SmbReadingsShm = SmbReadings;
    using SmbReadingsRos = smb_msgs::SmbReadings;

    using SmbCommandsShm = SmbCommands;
    using SmbCommandsRos = smb_msgs::SmbCommands;

    // TODO do we need these?
    using ActuatorEnum = typename ConcreteDescription::ActuatorEnum;
    using ActuatorArray = std_utils::EnumArray<SD::ConcreteTopology::SmbActuatorEnum, double>;
    using ActuatorModeArray = std_utils::EnumArray<SD::ConcreteTopology::SmbActuatorEnum, smb_common::SmbMode>;

    // Constructor.
    SmbGazeboPlugin();
    // Destructor.
    virtual ~SmbGazeboPlugin() = default;

  protected:
    // Inits joint structures.
    void initJointStructures() override;

    // Publishes actuator readings over ROS.
    void publishActuatorReadings() override;

    void updateActuatorReading(
            const ActuatorEnum& actuatorEnum,
            const double jointPosition,
            const double jointVelocity,
            const double jointAcceleration,
            const double jointTorque) override;

    double computeActuatorCommand(const ActuatorEnum& actuatorEnum) override;

    void receiveMessages() override;

    void actuatorCommandsCallback(const SmbCommandsShm& msg);

    // Reads parameters from the parameter server.
    void readParameters() override;

    void initSubscribers() override;
    // Inits the ROS publishers.
    void initPublishers() override;
    // Inits the ROS services.
    void initServices() override;

    // Publishes robot state over ROS.
    void publishRobotState() override;

    void Reset() override;

    void sendRos() override;

    void setActuatorGains();

    const std::chrono::microseconds receiveMaxLockTime_;

    // Frames generator.
    // quadruped_model::FramesGenerator<anymal_description::AnymalConcreteDescription, quadruped_model::QuadrupedState> framesGenerator_;

    cosmo_ros::PublisherRosPtr<SmbStateShm, SmbStateRos, smb_description_ros::ConversionTraits> smbStatePublisher_;

    ros::Publisher smbStateThrottledPublisher_;
    unsigned int robotStateThrottledCounter_ = 0;
    unsigned int robotStateThrottledDecimation_ = 0;

    // ROS actuator readings publisher.
    cosmo_ros::PublisherRosPtr<SmbReadingsShm, SmbReadingsRos, SmbReadingConversionTrait> smbActuatorReadingsPublisher_;

    // Subscriber.
    cosmo_ros::SubscriberRosPtr<SmbCommandsShm, SmbCommandsRos, SmbCommandsConversionTrait> smbActuatorCommandsSubscriber_;

    SmbCommands smbCommands_;
    SmbCommands smbDefaultPositionCommands_;
    SmbReadings smbReadings_;
    SmbStateShm smbMsgShm_;
    SmbStateRos smbMsgRos_;

    ros::Publisher basePoseMeasuredPublisher_;
    ros::Publisher baseTwistMeasuredPublisher_;
    geometry_msgs::PoseStamped basePoseMeasuredMsg_;
    geometry_msgs::TwistStamped baseTwistMeasuredMsg_;

  };

}




//
//using KinovaGazeboPluginBase = AnybotGazeboPlugin<kinova_model::KinovaModel::RD>;
//class KinovaGazeboPlugin : public KinovaGazeboPluginBase
//{
//public:
//  using ConcreteDescription = kinova_model::KinovaModel::KinovaDescription ;
//  using KinovaStateShm = kinova_description::KinovaState;
//  using KinovaStateRos = kinova_msgs::KinovaState;
//  using KD = kinova_model::KinovaModel::KinovaDescription;
//
//  template<typename Msg_, typename MsgRos_>
//  using KinovaReadingConversionTrait = kinova_description_ros::ConversionTraits<Msg_, MsgRos_>;
//
//  template<typename Msg_, typename MsgRos_>
//  using KinovaCommandsConversionTrait = kinova_description_ros::ConversionTraits<Msg_, MsgRos_>;
//
////  using AlmaSeaActuatorContainer = romo_std::ActuatorPerfectTorqueSourceRobotContainer<ConcreteDescription, series_elastic_actuator_sim::SeActuatorPerfectTorqueSource>; //contains entries for all joints although only SEAs are actively retrieved/used
//  using KinovaCommands = kinova_description::KinovaCommands;
//  using KinovaReadings = kinova_description::KinovaMeasurements;
//
//  using KinovaReadingsShm = KinovaReadings;
//  using KinovaReadingsRos = kinova_msgs::KinovaReadings;
//
//  using KinovaCommandsShm = KinovaCommands;
//  using KinovaCommandsRos = kinova_msgs::KinovaCommands;
//
//  using ActuatorEnum = typename ConcreteDescription::ActuatorEnum;
//
//  using ActuatorArray = std_utils::EnumArray<KD::ConcreteTopology::KinovaActuatorEnum, double>;
//  using ActuatorModeArray = std_utils::EnumArray<KD::ConcreteTopology::KinovaActuatorEnum, kinova_common::KinovaMode>;
//  using FingerArray = std_utils::EnumArray<KD::ConcreteTopology::KinovaFingerActuatorEnum, double>;
//
//  // Constructor.
//  KinovaGazeboPlugin();
//
//  // Destructor.
//  virtual ~KinovaGazeboPlugin() = default;
//
//protected:
//  // Inits joint structures.
//  void initJointStructures() override;
//
//  // Publishes actuator readings over ROS.
//  void publishActuatorReadings() override;
//
//  void updateActuatorReading(
//          const ActuatorEnum& actuatorEnum,
//          const double jointPosition,
//          const double jointVelocity,
//          const double jointAcceleration,
//          const double jointTorque) override;
//
//  double computeActuatorCommand(const ActuatorEnum& actuatorEnum) override;
//
//  void receiveMessages() override;
//
//  void actuatorCommandsCallback(const KinovaCommandsShm& msg);
//
//  void setActuatorGains();
//
//  void readParameters() override;
//
//  void initSubscribers() override;
//
//  void initPublishers() override;
//
//  void initServices() override;
//
//  void publishRobotState() override;
//
//  void Reset() override;
//
//  void sendRos() override;
//
//  KinovaGazeboPlugin::WrenchShm createContactWrenchMessage(const ContactEnum& contactEnum);
//
//
//  // ROS actuator readings publisher.
//  cosmo_ros::PublisherRosPtr<KinovaReadingsShm, KinovaReadingsRos, KinovaReadingConversionTrait> kinovaActuatorReadingsPublisher_;
//
//  // Subscriber.
//  cosmo_ros::SubscriberRosPtr<KinovaCommandsShm, KinovaCommandsRos, KinovaCommandsConversionTrait> kinovaActuatorCommandsSubscriber_;
//
//  KinovaCommands kinovaCommands_;
//  KinovaCommands kinovaDefaultPositionCommands_;
//  KinovaReadings kinovaReadings_;
//
//  // hack to include freeze behavior for kinova joints
//  ActuatorModeArray previousActuatorMode_;
//  ActuatorArray kinovaDesiredJointPositionsFreeze_;
//
//  FingerArray fingerPositionsDesired_;
//  const double openFingerAngle = 0.0;
//  const double closedFingerAngle = 2.0;
//  const double fingerPositionScaleToGazebo_ = closedFingerAngle-openFingerAngle; //converts generalized 0-1 input to 0-2 rad in Gazebo
//
//  const std::chrono::microseconds receiveMaxLockTime_;
//
//  // Kinova model.
//  kinova_model::KinovaModel kinovaModel_;
//
//  // Frames generator.
////  kinova_model::FramesGenerator framesGenerator_;
//
//  cosmo_ros::PublisherRosPtr<KinovaStateShm, KinovaStateRos, kinova_description_ros::ConversionTraits> kinovaStatePublisher_;
//
//  ros::Publisher kinovaStateThrottledPublisher_;
//  unsigned int robotStateThrottledCounter_ = 0;
//  unsigned int robotStateThrottledDecimation_ = 0;
//
//  KinovaStateShm kinovaMsgShm_;
//  KinovaStateRos kinovaMsgRos_;





