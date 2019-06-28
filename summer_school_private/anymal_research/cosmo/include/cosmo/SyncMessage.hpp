/*
 * SyncMessage.hpp
 *
 *  Created on: Jul 2, 2017
 *      Author: Christian Gehring
 */

#pragma once

#include <any_measurements/Time.hpp>

namespace cosmo {

class SyncMessage {
 public:
  SyncMessage() {

  }

  virtual ~SyncMessage() {

  }

  friend std::ostream& operator << (std::ostream& out, const SyncMessage& msg) {
    out << "seq: " << msg.counter_ << ", sync time: " << msg.currentSyncTime_ << " next sync time: " << msg.nextSyncTime_;
    return out;
  }

 public:
  unsigned long int counter_ = 0lu;
  any_measurements::Time currentSyncTime_;
  any_measurements::Time nextSyncTime_;


};


} // namespace
