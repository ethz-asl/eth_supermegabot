/*!
 * @file	Publisher.hpp
 * @author	Philipp Leemann, Christian Gehring
 * @date	May 24, 2017
 */

#pragma once

#include "cosmo/Communicator.hpp"
#include "cosmo/PublisherOptions.hpp"

#include "message_logger/message_logger.hpp"

#include <boost/thread/thread_time.hpp> // for boost::get_system_time

#include <chrono>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

namespace cosmo {

/**
 *
 * @tparam     MessageType_  Message that will be stored in shared memory. Don not use dynamic sized memory and std::string!
 * @tparam     Alignment_    Alignment of the memory allocation, has to be a multiple of size of a SIMD memory read instruction (128bits)
 */
template<typename MessageType_, unsigned int Alignment_ = DefaultAlignment>
class Publisher :  public Communicator<MessageType_, true, Alignment_> {
public:
    using BaseType = Communicator<MessageType_, true, Alignment_>;
    using Options = PublisherOptions;

	Publisher() = delete;
    Publisher(const std::string& topic):
        Publisher(std::make_shared<Options>(topic))
    {
    }

    Publisher(const PublisherOptionsPtr& options):
        BaseType(options),
        options_(options)
    {
    }

    Publisher(Publisher&&) = default;
	~Publisher() override = default;


    /**
     * @brief      Writes a message to shared memory
     *
     * @param[in]  message      the message gets written to shared memory
     * @param[in]  maxLockTime  the maxmimum lock time
     *
     * @return     returns true if the message has been written to the shared
     *             memory within the maxLockTime
     */
    virtual bool publish(const MessageType_& message, const std::chrono::microseconds& maxLockTime = std::chrono::microseconds{50}) {
        bool sendSuccessful = false;
        {
            const boost::posix_time::ptime abs_time = boost::get_system_time() + boost::posix_time::microseconds(maxLockTime.count());
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_sharable_mutex> lock(*(this->smMutex_), boost::interprocess::defer_lock);
            if(lock.timed_lock(abs_time)) {
                if(*this->smNumSubscribers_ > 0u) {
                    ++(*this->smCounter_);
                    *this->smMessage_ = message;

#ifdef COSMO_DEBUG_LEVEL2
                    MELO_INFO("\e[1;36m<== Sent message over shm: Idx: %ld, Topic: %s \e[0m", *(this->smCounter_), options_->topic_.c_str());
#endif
                    sendSuccessful = true;
                    this->smCnd_->notify_all();
                }else{
#ifdef COSMO_DEBUG_LEVEL2
                    MELO_INFO("\e[0;31m<== No shm subscriber, not sending message. Topic %s \e[0m", options_->topic_.c_str());
#endif
                }
            }
        }

        // notify PoolSubscribers about new message
        this->smPoolCnd_->notify_all();
        return sendSuccessful;
    }

protected:
    const std::shared_ptr<const Options> options_;
};

template<typename MessageType_, unsigned int Alignment_ = DefaultAlignment>
using PublisherPtr = std::shared_ptr<Publisher<MessageType_, Alignment_>>;


} // namespace 
