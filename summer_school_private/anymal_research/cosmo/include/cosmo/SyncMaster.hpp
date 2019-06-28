/*
 * SyncMaster.cpp
 *
 *  Created on: Jun 30, 2017
 *      Author: Philipp Leemann, Christian Gehring
 */

#pragma once

#include "cosmo/Publisher.hpp"
#include "cosmo/SyncMessage.hpp"

#include "any_worker/Worker.hpp"

#include <condition_variable>
#include <mutex>
#include <memory>

namespace cosmo {

class SyncMaster : virtual private Publisher<SyncMessage, DefaultAlignment> {
public:
    using BaseType = Publisher<SyncMessage, DefaultAlignment>;

    SyncMaster() = delete;

    /**
     * @brief      Creates the syncmaster
     *
     * @param[in]  topic               The synchronization topic
     * @param[in]  timeStep            The synchronization timestep
     * @param[in]  syncThreadPriority  The priority of the synchronization
     *                                 thread
     * @param[in]  autoSync            The sync msg is sent automatically in a
     *                                 separate worker thread if autosync is set to true
     */
    SyncMaster(const std::string& topic,
               const double timeStep,
               const int syncThreadPriority = 79,
               const bool autoSync = true);

    SyncMaster(SyncMaster&&) = default;

    ~SyncMaster() override;


    /**
     * @brief      Starts the syncmaster. If autoSync is set to true, this
     *             method starts the synchronization thread
     */
    void start();

    /**
     * @brief      Stops the syncmaster and the worker thread if autosync is set
     *             to true
     *
     * @param[in]  wait  Keeps the worker thread alive if set to true
     */
    void stop(const bool wait=true);

    /**
     * @brief      Publishes the synchronization message. Call this method
     *             manually if autoSync is set to false
     *
     * @param[in]  syncTime  The absolute time of the sync call
     * @param[in]  timeStep  The synchronization timestep
     */
    void sync(const any_measurements::Time& syncTime, const double timeStep);

    /**
     * @brief      The syncmaster waits for the synchronization signal. This is
     *             useful if autosync is set to true
     *
     * @return     The synchronization message
     */
    SyncMessage waitForSync();


protected:

    bool callback(const any_worker::WorkerEvent& event);

protected:
    any_worker::Worker worker_;

    std::condition_variable_any cndLocalSync_;
    std::mutex mutexSyncMessage_; // protects syncMessage and localSyncCounter
    SyncMessage syncMessage_;
    unsigned long int localSyncCounter_;
    bool autoSync_;

};


using SyncMasterPtr = std::shared_ptr<SyncMaster>;

} /* namespace cosmo */
