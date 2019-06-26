/*
 * SynchronizerSlave.hpp
 *
 *  Created on: Jul 2, 2017
 *      Author: Philipp Leemann, Christian Gehring
 */

#pragma once


#include "message_logger/message_logger.hpp"
#include "cosmo/Subscriber.hpp"
#include "cosmo/SyncMessage.hpp"

namespace cosmo {

class SyncSlave : virtual private Subscriber<SyncMessage, DefaultAlignment> {
public:
    using BaseType = Subscriber<SyncMessage, DefaultAlignment>;
    using Options = SubscriberOptions<SyncMessage>;

    SyncSlave() = delete;

    /**
     * @brief      Creates the synchronization slave
     *
     * @param[in]  topic               The synchronization topic
     * @param[in]  syncThreadPriority  The synchronization thread priority
     */
    SyncSlave(const std::string& topic,
              const int syncThreadPriority = 59):
            BaseType(std::make_shared<Options>(topic, std::bind(&SyncSlave::syncCallback, this, std::placeholders::_1))),
            syncMessage_()
    {
    }

    SyncSlave(SyncSlave&&) = default;
    ~SyncSlave() override = default;

    using BaseType::start;
    using BaseType::stop;

    /**
     * @brief      This method blocks until a new syncsignal has arrived
     *
     * @param[in]  timeout  The maximum timeout for the receive method
     *
     * @return     Returns the new sync message 
     */
    inline SyncMessage waitForSync(const std::chrono::microseconds& timeout = std::chrono::microseconds{200}) {
        while(this->active_ && !BaseType::receive( timeout )) {
        }

        return syncMessage_;
    }


protected:
    inline void syncCallback(const SyncMessage& msg) {
        syncMessage_ = msg;
    }

protected:
    SyncMessage syncMessage_;
};

} // namespace
