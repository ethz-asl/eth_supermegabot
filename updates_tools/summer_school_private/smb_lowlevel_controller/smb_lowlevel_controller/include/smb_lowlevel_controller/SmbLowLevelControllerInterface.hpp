/*!
 * @file     SmbLowLevelControllerInterface.hpp
 * @author   Tim Sandy, Koen Kraemer
 * @date     Oct, 2018
 */

#pragma once

#include <mutex>

// smb_common
#include <smb_common/SmbCommands.hpp>
#include <smb_common/SmbReadings.hpp>

#include <smb_description/SmbDescription.hpp>
#include <smb_model/SmbModel.hpp>
#include <smb_msgs/SmbCommands.h>
#include <smb_msgs/SmbReadings.h>
#include <any_measurements_ros/any_measurements_ros.hpp>

#include <smb_driver/SmbController.h>

namespace smb_lowlevel_controller {

//todo How to accept an external command to change the Smb command mode?

  template<typename ConcreteDescription_>
  class SmbLowLevelControllerInterface
  {
  public:
    using RD = typename ConcreteDescription_::ConcreteDefinitions;
    using RT = typename ConcreteDescription_::ConcreteTopology;
    using SmbMode = smb_common::SmbMode;

    using SmbCommandsShm = smb_common::SmbCommands<ConcreteDescription_>;
    using SmbReadingsShm = smb_common::SmbReadings<ConcreteDescription_>;

    SmbLowLevelControllerInterface();
    virtual ~SmbLowLevelControllerInterface() = default;

    bool init(std::string port, ros::NodeHandle &nh, bool commandSmb = true);
    void preCleanup();
    void cleanup();

    bool updateCommands(const SmbCommandsShm& smbCommands);
    bool updateMeasurements(SmbReadingsShm& smbReadings);

    void setFreeze();

  protected:
    std::shared_ptr<smb_driver::SmbController> smb_;
//    bool smbConnected = false; //todo Should use a mode flag from the smb driver
    int motorControllerTimeoutUs_ = 500; // [microsec]

    std::mutex smbDriverMutex_;

  smb_common::SmbModeType lastControlMode_;
  };

} /* smb_lowlevel_controller */

#include <smb_lowlevel_controller/SmbLowLevelControllerInterface.tpp>
