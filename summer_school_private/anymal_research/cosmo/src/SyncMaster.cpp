/*
 * SyncMaster.cpp
 *
 *  Created on: Jun 30, 2017
 *      Author: Philipp Leemann, Christian Gehring
 */

#include "cosmo/SyncMaster.hpp"

namespace cosmo {

SyncMaster::SyncMaster(const std::string& topic,
                       const double timeStep,
                       const int syncThreadPriority,
                       bool autoSync):
        BaseType(topic),
        worker_(any_worker::WorkerOptions("SyncMasterWorker", timeStep, std::bind(&SyncMaster::callback, this, std::placeholders::_1), syncThreadPriority)),
        cndLocalSync_(),
        mutexSyncMessage_(),
        syncMessage_(),
        localSyncCounter_{0lu},
        autoSync_(autoSync)
{
}

SyncMaster::~SyncMaster() {
  if (autoSync_) {
    stop(true);
  }
}


void SyncMaster::start() {
    // No other thread is accessing the sync message, no need to lock mutex
    syncMessage_.counter_ = 0lu;
    localSyncCounter_ = 0lu;

    if (autoSync_) {
      worker_.start();
    }
}

void SyncMaster::sync(const any_measurements::Time& syncTime, double timeStep) {
  {
      std::unique_lock<std::mutex> lockMsg{mutexSyncMessage_};
      syncMessage_.counter_++;
      syncMessage_.currentSyncTime_ = syncTime;
      syncMessage_.nextSyncTime_ = syncMessage_.currentSyncTime_ + timeStep;
      this->publish(syncMessage_);
  }
  cndLocalSync_.notify_all();
}

void SyncMaster::stop(const bool wait) {
  if (autoSync_) {
    worker_.stop(false);
    cndLocalSync_.notify_all();
    worker_.stop(wait);
  }
}

SyncMessage SyncMaster::waitForSync() {
    std::unique_lock<std::mutex> lock{mutexSyncMessage_};
    cndLocalSync_.wait(lock, [=]() { return (syncMessage_.counter_ > localSyncCounter_) || !worker_.isRunning(); });
    localSyncCounter_ = syncMessage_.counter_ ;
    return syncMessage_;
}


bool SyncMaster::callback(const any_worker::WorkerEvent& event) {
    sync(any_measurements::Time(event.timeStamp), event.timeStep);
    return true;
}

} /* namespace cosmo */
