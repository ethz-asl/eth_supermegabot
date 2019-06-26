/*!
 * @file	Subscriber.hpp
 * @author	Philipp Leemann, Christian Gehring
 * @date	May 24, 2017
 */

#pragma once

#include "cosmo/Communicator.hpp"
#include "cosmo/SubscriberOptions.hpp"

#include "message_logger/message_logger.hpp"

#include <boost/thread/thread_time.hpp> // for boost::get_system_time

#include <thread>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>

namespace cosmo {

/**
 * @brief      Class for subscriber.
 *
 * @tparam     MessageType_  Message that will be stored in shared memory. Don not use dynamic sized memory and std::string!
 * @tparam     Alignment_    Alignment of the memory allocation, has to be a multiple of size of a SIMD memory read instruction (128bits)
 */
template<typename MessageType_, unsigned int Alignment_ = DefaultAlignment>
class Subscriber : public Communicator<MessageType_, false, Alignment_> {
public:
    using BaseType = Communicator<MessageType_, false, Alignment_>;
    using Options = SubscriberOptions<MessageType_>;
    using OptionsPtr = std::shared_ptr<Options>;
    using SubscriberCallback = typename Options::SubscriberCallback;

    Subscriber() = delete;
    Subscriber(const std::string& topic, const SubscriberCallback& callback):
        Subscriber(std::make_shared<Options>(topic, callback))
    {
    }

    Subscriber(const OptionsPtr& options):
            BaseType(options),
            options_(options),
            active_{false},
            thread_(),
            localCounter_{0lu},
            smLock_{*(this->smMutex_), boost::interprocess::defer_lock_type()}
    {
        boost::interprocess::sharable_lock<boost::interprocess::interprocess_sharable_mutex> lock(*(this->smMutex_));

        if(!options_->latch_) {
            localCounter_ = *(this->smCounter_);
        }
    }

    Subscriber(Subscriber&& other):
            BaseType(std::move(other)),
            options_{std::move(other.options_)},
            active_{other.active_.load()},
            thread_{std::move(other.thread_)},
            localCounter_{other.localCounter_.load()},
            smLock_{std::move(other.smLock_)}
    {
    }

	~Subscriber() override
    {
        stop();
    }

    /**
     * @brief      Enables the subscriber to start receiving messages. If
     *             autosubsribe is set to true in the subscriber options, this
     *             method starts calling receive in a separate thread
     */
    void start() {
        if(active_) {
            // start the thread only once
            return;
        }
        // Wait for thread to finish. Do this before setting active_ to true, otherwise the thread may never finish
        if(thread_.joinable()) {
            thread_.join();
        }

        active_ = true;

        if(options_->autoSubscribe_) {
            thread_ = std::thread(&Subscriber::workerFunction, this);

            // Change priority of the thread.
            sched_param sched;
            sched.sched_priority = options_->autoSubscriberPriority_;
            if (sched.sched_priority != 0) {
                if (pthread_setschedparam(thread_.native_handle(), SCHED_FIFO, &sched) != 0) {
                    MELO_WARN("Failed to set thread priority: %s", strerror(errno));
                }
            }
        }
    }

    /**
     * @brief      Stops the subscriber to receive messages. Stops the
     *             subscriber thread if using autosubscribe
     *
     * @param[in]  wait  Keeps the subsriber thread alive when set to false
     */
    virtual void stop(const bool wait=true) {
        active_ = false;
        this->smCnd_->notify_all();
        if(wait && thread_.joinable()) {
            thread_.join();
        }
    }

    /*!
     * Waits maxLockTime for incoming messages and calls the callback on
     * reception.
     *
     * @param[in]  maxLockTime  The maximum lock time
     *
     * @return     Returns true if a new message has been received within
     *             maxLockTime
     */
    virtual bool receive(const std::chrono::microseconds& maxLockTime = std::chrono::microseconds{200}) {
        // Wait until the readings have been updated, timeout occurred or shutdown requested
        const boost::posix_time::ptime abs_time = boost::get_system_time() + boost::posix_time::microseconds(maxLockTime.count());
        smLock_.lock();
        bool hasNewMessage = this->smCnd_->timed_wait(smLock_, abs_time,
                                                      [=]() {return (*(this->smCounter_) > localCounter_ || !this->active_); });
        if (!this->active_) {
            unlockMutex();
            return false; // Stop immediately.
        }

        if (hasNewMessage) {
#ifdef COSMO_DEBUG_LEVEL2
            MELO_INFO("\e[1;32m==> Received message over shm: Idx: %ld, Topic: %s \e[0m", *(this->smCounter_), options_->topic_.c_str());
#endif
            // fixme: copy message out of shared memory and pass the copy to the callback?
            localCounter_ = *(this->smCounter_);
            this->options_->callback_(*(this->smMessage_));
        }

        unlockMutex();

        return hasNewMessage;
    }


    /**
     * @brief      Returns a local copy of the shared memory message count
     *
     * @return     The shared memory message count
     */
    unsigned long int getMessageCounter() const {
        return localCounter_;
    }

    /**
     * @brief      Unlocks the mutex protecting the message allocated in shared
     *             memory. May be used in subscriber callback before doing
     *             time-consuming operations
     */
    void unlockMutex() const {
        if(smLock_.owns()) {
            smLock_.unlock();
        }
    }

protected:

    void workerFunction() {
        while(active_) {
            // call receive function with a large timeout -> 1min
            receive( std::chrono::microseconds{60000000} );
        }
    }


protected:
    const std::shared_ptr<const Options> options_;
    std::atomic<bool> active_;
    std::thread thread_;
    std::atomic<unsigned long int> localCounter_;

    mutable boost::interprocess::sharable_lock<boost::interprocess::interprocess_sharable_mutex> smLock_;
};

template<typename MessageType_, unsigned int Alignment_ = DefaultAlignment>
using SubscriberPtr = std::shared_ptr<Subscriber<MessageType_, Alignment_>>;


} // namespace 
