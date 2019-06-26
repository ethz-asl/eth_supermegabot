/*
 * UnitTestShmMsg.hpp
 *
 *  Created on: Jun 30, 2017
 *      Author: gehrinch
 */

#pragma once

#include <cosmo_ros/UnitTestRosMsg.h>

namespace cosmo_ros {

//! Message to store in shared memory
class UnitTestShmMsg {
public:
  UnitTestShmMsg() : a(0) {}

    int a;
};


// Forward declaration
template<typename Msg_, typename MsgRos_>
class ConversionTraits;

//! Conversions between shared memory message and ROS message.
template<>
class ConversionTraits<UnitTestShmMsg, UnitTestRosMsg> {
public:
  using MsgShm = UnitTestShmMsg;
  using MsgRos = UnitTestRosMsg;

  inline static MsgRos convert(const MsgShm& shmMsg) {
    MsgRos rosMsg;
    rosMsg.a = shmMsg.a;
    return rosMsg;
  }

  inline static MsgShm convert(const MsgRos& rosMsg) {
    MsgShm shmMsg;
    shmMsg.a = rosMsg.a;
    return shmMsg;
  }
};

} // namespace
