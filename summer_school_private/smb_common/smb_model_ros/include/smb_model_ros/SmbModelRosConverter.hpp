/*!
 * @file     SmbModelRosConverter.hpp
 * @author   Koen Kraemer
 * @date     Aug, 2018
 */

#pragma once

// model
#include <smb_model/SmbModel.hpp>

// msgs
#include <smb_msgs/SmbState.h>

namespace smb_model_ros {

class SmbModelRosConverter
{
 public:
  SmbModelRosConverter() = delete;

  static bool fromMessage(const smb_msgs::SmbState& message,
                          smb_model::SmbModel& model);
  static bool toMessage(const smb_model::SmbModel& model,
                        smb_msgs::SmbState& message);
};

} /* namespace smb_model */
