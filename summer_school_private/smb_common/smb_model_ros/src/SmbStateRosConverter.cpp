/*!
 * @file     SmbStateRosConverter.cpp
 * @author   Koen Kraemer
 * @date     Aug, 2018
 */

// description
#include "smb_description/SmbDescription.hpp"

// model ros
#include "smb_model_ros/SmbStateRosConverter.hpp"

// kindr
#include <kindr_ros/kindr_ros.hpp>

using namespace smb_model;

namespace smb_model_ros {

    bool SmbStateRosConverter::fromMessage(const smb_msgs::SmbState& message,
                                            smb_model::SmbState& state)
    {
      using RD = smb_description::SmbDescription;
      using RT = smb_description::SmbTopology;

      // Wheels.
      for (const auto smbActuatorKey : RD::getKeys<RT::SmbActuatorEnum>()) {
//        const auto smbActuatorEnum = smbActuatorKey.getEnum();
        const auto smbWheelId = smbActuatorKey.getId();
        state.getWheelVelocities()(smbWheelId) = message.wheelVelocities[smbWheelId];
	state.getWheelTorques()(smbWheelId) = message.wheelVelocities[smbWheelId];
      }

//      // Frame transforms.
//      for (const auto& tf : message.anymal_state.frame_transforms) {
//        const std::string transformName = tf.child_frame_id + "_to_" + tf.header.frame_id;
//        alma_model::Pose transform;
//        kindr_ros::convertFromRosGeometryMsg(tf.transform, transform);
//
//        constexpr auto frameKeys = RD::getKeys<RT::FrameTransformEnum>();
//        state.setFrameTransform(frameKeys.atName(transformName).getEnum(), transform);
//      }

      return true;
    }

    bool SmbStateRosConverter::toMessage(const smb_model::SmbState& state,
                                          smb_msgs::SmbState& message)
    {
      using RD = smb_description::SmbDescription;
      using RT = smb_description::SmbTopology;

      // Wheels.
      for (const auto smbActuatorKey : RD::getKeys<RT::SmbActuatorEnum>()) {
//        const auto smbActuatorEnum = smbActuatorKey.getEnum();
        const auto smbWheelId = smbActuatorKey.getId();
        message.wheelVelocities[smbWheelId] = static_cast<float>(state.getWheelVelocities()(smbWheelId));
        message.wheelTorques[smbWheelId] = static_cast<float>(state.getWheelTorques()(smbWheelId));
      }

//      // Frame transforms. Frame names need to be initialized by the concrete
//      // robot description, otherwise some (slow) string operations will be
//      // necessary
//      for(unsigned int i = 0; i < message.anymal_state.frame_transforms.size(); ++i) {
//        kindr_ros::convertToRosGeometryMsg(state.getFrameTransform(
//                static_cast<RT::FrameTransformEnum>(i)), message.anymal_state.frame_transforms[i].transform);
//      }

      return true;
    }

} /* namespace smb_model_ros */
