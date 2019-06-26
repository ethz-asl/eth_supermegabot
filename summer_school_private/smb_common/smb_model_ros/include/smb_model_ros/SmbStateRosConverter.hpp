/*!
 * @file     SmbStateRosConverter.hpp
 * @author   Koen Kraemer
 * @date     Aug, 2018
 */

#pragma once

// smb model
#include <smb_model/SmbState.hpp>

// smb msgs
#include <smb_msgs/SmbState.h>

// kindr
#include <kindr_ros/kindr_ros.hpp>

namespace smb_model_ros {

class SmbStateRosConverter
{
 public:
  SmbStateRosConverter() = delete;

  static bool fromMessage(const smb_msgs::SmbState& message,
                          smb_model::SmbState& state);
  static bool toMessage(const smb_model::SmbState& state,
                        smb_msgs::SmbState& message);
};

} /* namespace smb_model_ros */
