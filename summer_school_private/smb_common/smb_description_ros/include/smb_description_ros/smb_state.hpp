/*!
 * @file     smb_state.hpp
 * @author   Koen Kraemer
 * @date     Aug, 2018
 */

#pragma once

// msgs
#include <smb_msgs/SmbState.h>

#include <smb_description_ros/smb_readings.hpp>

namespace smb_description_ros {

static void initializeSmbState(smb_msgs::SmbState& smbState) {

  smbState.state = smb_msgs::SmbState::STATE_ERROR_UNKNOWN;
  smbState.header.frame_id = "odom";
  smbState.pose.header.frame_id = "odom";
  smbState.pose.pose.orientation.w = 1.0;
  smbState.pose.pose.orientation.x = 0.0;
  smbState.pose.pose.orientation.y = 0.0;
  smbState.pose.pose.orientation.z = 0.0;
  smbState.pose.pose.position.x = 0.0;
  smbState.pose.pose.position.y = 0.0;
  smbState.pose.pose.position.z = 0.0;
  // anymal_description_ros::initializeContacts(quadrupedState.contacts);
  // anymal_description_ros::initializeFrameTransforms(quadrupedState.frame_transforms);

  // constexpr auto numJoints = smb_description::SmbDescription::getJointsDimension();
  // smbState.joints.position.assign(numJoints, 0.0);
  // smbState.joints.velocity.assign(numJoints, 0.0);
  // smbState.joints.acceleration.assign(numJoints, 0.0);
  // smbState.joints.effort.assign(numJoints, 0.0);
  // smbState.joints.name.clear();
  // smbState.joints.name.reserve(numJoints);
  // for (const auto & key : smb_description::SmbDescription::getJointKeys()) {
  //   smbState.joints.name.emplace_back(key.getName());
  // }

  smbState.wheelVelocities.assign(4, 0.0);
  smbState.wheelTorques.assign(4, 0.0);
}

} /* smb_description_ros */
