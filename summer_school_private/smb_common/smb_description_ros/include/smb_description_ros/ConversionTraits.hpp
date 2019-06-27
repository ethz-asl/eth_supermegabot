/*!
 * @file     ConversionTraits.hpp
 * @author   Koen Kraemer
 * @date     Aug, 2018
 */

#pragma once

#include <any_measurements_ros/any_measurements_ros.hpp>
#include <smb_description/Containers.hpp>
#include <smb_description_ros/smb_state.hpp>

#include <kindr_ros/kindr_ros.hpp>
#include <smb_model_ros/SmbStateRosConverter.hpp>
#include <smb_msgs/SmbState.h>
#include <romo/common/RobotContainersRos.hpp>
#include <std_utils/std_utils.hpp>
#include <smb_ros/ConvertRosMessages.hpp>


namespace smb_description_ros {

template<typename Msg_, typename MsgRos_>
class ConversionTraits;

template<>
class ConversionTraits<smb_description::SmbState, smb_msgs::SmbState> {
public:
  using Msg = smb_description::SmbState;
  using MsgRos = smb_msgs::SmbState ;

  // Calling this function requires an initialized rosMsg
  inline static void convert(const Msg& msg, MsgRos& rosMsg) {
    using namespace any_measurements_ros;
    using namespace kindr_ros;

    smb_model_ros::SmbStateRosConverter::toMessage(msg.smbState_, rosMsg);

    ros::Time timeMsg = toRos(msg.time_);
    rosMsg.header.stamp = timeMsg;
    // rosMsg.joints.header.stamp = timeMsg;
    rosMsg.pose.header.stamp = timeMsg;
    rosMsg.twist.header.stamp = timeMsg;

//    for(const auto contactKey : smb_model::SmbModel::RD::getContactKeys()) {
//      const auto contactEnum = contactKey.getEnum();
//      if (contactEnum != kinova_model::KinovaModel::RD::ConcreteTopology::ContactEnum::GRIPPER) { // hand contact cannot be handled by anymal_state
//        const auto contactId = contactKey.getId();
//        auto& contactRos = rosMsg.anymal_state.contacts[contactId];
//        const auto& contactShm = msg.contacts_[contactEnum];
//        contactRos.header.stamp = timeMsg;
//        convertToRosGeometryMsg(contactShm.normal_, contactRos.normal);
//        convertToRosGeometryMsg(contactShm.position_, contactRos.position);
//        toRos(contactShm.wrench_, contactRos.wrench);
//        contactRos.state = static_cast<int8_t>(contactShm.state_);
//      }
//    }

    for(auto& tf : rosMsg.frame_transforms) {
      tf.header.stamp = timeMsg;
    }

    rosMsg.state = static_cast<int8_t>(msg.status_);
  }

  inline static void convert(const MsgRos& rosMsg, Msg& msg) {
    using namespace any_measurements_ros;
    using namespace kindr_ros;

    msg.time_ = fromRos(rosMsg.header.stamp);

//    for(const auto contactKey : smb_model::SmbModel::RD::getContactKeys()) {
//      const auto contactEnum = contactKey.getEnum();
//      if (contactEnum != kinova_model::KinovaModel::RD::ConcreteTopology::ContactEnum::GRIPPER) {
//        const auto contactId = contactKey.getId();
//        auto& contactShm = msg.contacts_[contactEnum];
//        const auto& contactRos = rosMsg.anymal_state.contacts[contactId];
//        contactShm.time_ = fromRos(contactRos.header.stamp);
//        convertFromRosGeometryMsg(contactRos.normal, contactShm.normal_);
//        convertFromRosGeometryMsg(contactRos.position, contactShm.position_);
//        contactShm.wrench_ = fromRos(contactRos.wrench);
//        contactShm.state_ = static_cast<int>(contactRos.state);
//      }
//    }

    smb_model_ros::SmbStateRosConverter::fromMessage(rosMsg, msg.smbState_);
    msg.status_ = static_cast<smb_description::StateStatus>(rosMsg.state);

  }

  inline static MsgRos convert(const Msg& msg) {
    MsgRos rosMsg;
    smb_description_ros::initializeSmbState(rosMsg);
    convert(msg, rosMsg);
    return rosMsg;
  }

  inline static Msg convert(const MsgRos& rosMsg) {
    Msg msg;
    convert(rosMsg, msg);
    return msg;
  }
};


template<>
class ConversionTraits<smb_description::SmbCommands, smb_msgs::SmbCommands> {
public:
  using Msg = smb_description::SmbCommands;
  using MsgRos = smb_msgs::SmbCommands;
  using RD = smb_description::SmbDescription;
  using RT = smb_description::SmbTopology;

  inline static MsgRos convert(const Msg& msg) {
    MsgRos rosMsg;
    smb_ros::ConvertRosMessages<smb_model::SmbModel::SmbDescription>::writeToMessage(rosMsg, msg);
    return rosMsg;
  }

inline static Msg convert(const MsgRos& rosMsg) {
    Msg shmMsg;
    smb_ros::ConvertRosMessages<smb_model::SmbModel::SmbDescription>::readFromMessage(shmMsg, rosMsg);
    return shmMsg;
  }
};

template<>
class ConversionTraits<smb_description::SmbReadings, smb_msgs::SmbReadings> {
public:
  using Msg = smb_description::SmbReadings ;
  using MsgRos = smb_msgs::SmbReadings;
  using RD = smb_description::SmbDescription;
  using RT = smb_description::SmbTopology;

  inline static MsgRos convert(const Msg& msg) {
    MsgRos rosMsg;
    smb_ros::ConvertRosMessages<smb_model::SmbModel::SmbDescription>::writeToMessage(rosMsg, msg);
    return rosMsg;
  }

inline static Msg convert(const MsgRos& rosMsg) {
    Msg shmMsg;
    smb_ros::ConvertRosMessages<smb_model::SmbModel::SmbDescription>::readFromMessage(shmMsg, rosMsg);
    return shmMsg;
  }

};

//template<>
//class ConversionTraits<smb_description::SmbJointState, sensor_msgs::JointState> {
//public:
//  using Msg = smb_description::SmbJointState;
//  using MsgRos = sensor_msgs::JointState;
//
//  inline static void convert(const Msg& msg, MsgRos& msgRos) {
//   for(auto jointKey : smb_description::SmbDescription::getJointKeys()){
//     const auto jointEnum = jointKey.getEnum();
//     const auto jointId = jointKey.getId();
//     msgRos.position[jointId] = msg[jointEnum].position_;
//     msgRos.velocity[jointId] = msg[jointEnum].velocity_;
//     msgRos.effort[jointId] = msg[jointEnum].effort_;
//   }
//   any_measurements_ros::toRos(msg[smb_description::SmbDescription ::ConcreteTopology::JointEnum::kinova_joint_1].time_, msgRos.header.stamp);
//  }
//
//  inline static void convert(const MsgRos& msgRos, Msg& msg) {
//   for(auto jointKey : smb_description::SmbDescription::getJointKeys()){
//     const auto jointEnum = jointKey.getEnum();
//     const auto jointId = jointKey.getId();
//     any_measurements_ros::fromRos(msgRos.header.stamp, msg[jointEnum].time_);
//     msg[jointEnum].position_ = msgRos.position[jointId];
//     msg[jointEnum].velocity_ = msgRos.velocity[jointId];
//     msg[jointEnum].effort_ = msgRos.effort[jointId];
//   }
//  }
//
//  inline static MsgRos convert(const Msg& msg) {
//   MsgRos rosMsg;
//   convert(msg, rosMsg);
//   return rosMsg;
//  }
//
//  inline static Msg convert(const MsgRos& rosMsg) {
//   Msg msg;
//   convert(rosMsg, msg);
//   return msg;
//  }
//};

}  // namespace smb_description_ros
