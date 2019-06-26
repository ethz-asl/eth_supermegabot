#pragma once

#include <romo/common/RobotContainers.hpp>

#include <any_measurements/ImuWithCovariance.hpp>
#include <any_measurements/Time.hpp>
#include <any_measurements/PointContact.hpp>
//#include <any_measurements/ExtendedJointState.hpp>

#include <smb_description/SmbDescription.hpp>
#include <smb_model/SmbModel.hpp>
#include <smb_model/common/containers.hpp>

#include <smb_common/SmbReadings.hpp>
#include <smb_common/SmbCommands.hpp>

#include <std_utils/containers/EnumArray.hpp>

namespace smb_description {

enum class StateStatus : int {
    STATUS_ERROR_SENSOR=-3,
    STATUS_ERROR_ESTIMATOR=-2,
    STATUS_ERROR_UNKNOWN=-1,
    STATUS_OK=0,
    STATUS_WARN_SENSOR_POSE=1
  };

struct SmbState
{
  any_measurements::Time time_;
  StateStatus status_;
  smb_model::SmbState smbState_;
  std_utils::EnumArray<typename smb_model::SmbModel::ContactEnum, any_measurements::PointContact> contacts_;
};

// using SmbJointState = std_utils::EnumArray<smb_description::SmbDescription::JointEnum, any_measurements::ExtendedJointState>;
using SmbReadings = smb_common::SmbReadings<smb_model::SmbModel::SmbDescription>;
using SmbCommands = smb_common::SmbCommands<smb_model::SmbModel::SmbDescription>;

struct SmbContainersImpl {

    struct RobotState {
        using type = SmbState;
    };

    struct ActuatorReadings {
        using type = SmbReadings;
    };

    struct ActuatorCommands {
        using type = SmbCommands;
    };

    struct Imu {
        using type = any_measurements::ImuWithCovariance;
    };

};

using SmbContainers = romo::RobotContainers<SmbContainersImpl>;

}
