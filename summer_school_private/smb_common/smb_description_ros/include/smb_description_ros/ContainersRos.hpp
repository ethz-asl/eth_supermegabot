/*!
 * @file     ContainersRos.hpp
 * @author   Tim Sandy
 * @date     Sept 20, 2018
 */

#pragma once

#include <smb_description/Containers.hpp>

// romo
#include <romo/common/RobotContainersRos.hpp>

// std utils
#include <smb_msgs/SmbState.h>
#include <any_measurements_ros/any_measurements_ros.hpp>
#include <smb_description_ros/ConversionTraits.hpp>

namespace smb_description_ros {
/**
 * @brief Static base class for all robot descriptions:
 */
struct SmbContainersRosImpl {
  //! Delete constructor, class consists only of static members
  SmbContainersRosImpl() = delete;

  //! Expose Msg Types
  struct RobotStateRos {
    using msgType = smb_msgs::SmbState;
    template <typename RobotState_, typename RobotStateRos_>
    using ConversionTrait = ConversionTraits<RobotState_, RobotStateRos_>;
  };

  struct ActuatorReadingsRos {
    using msgType = smb_msgs::SmbReadings;
    template <typename ActuatorReadings_, typename ActuatorReadingsRos_>
    using ConversionTrait = ConversionTraits<ActuatorReadings_, ActuatorReadingsRos_>;
  };

  struct ActuatorCommandsRos {
    using msgType = smb_msgs::SmbCommands;
    template <typename ActuatorCommands_, typename ActuatorCommandsRos_>
    using ConversionTrait = ConversionTraits<ActuatorCommands_, ActuatorCommandsRos_>;    
  };

  struct ImuRos {
    using msgType = sensor_msgs::Imu;
    template<typename Imu_, typename ImuRos_>
    using ConversionTrait = any_measurements_ros::ConversionTraits<Imu_, ImuRos_>;
  };
};

using SmbContainersRos = romo::RobotContainersRos<smb_description::SmbContainers, SmbContainersRosImpl>;

}  // namespace smb_description_ros
