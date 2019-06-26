/*!
 * @file	PublisherRos.hpp
 * @author	Philipp Leemann, Christian Gehring
 * @date	May 24, 2017
 */

#pragma once

#include "cosmo/Publisher.hpp"
#include "cosmo_ros/PublisherRosOptions.hpp"
#include "cosmo_ros/conversion.hpp"

#include <ros/publisher.h>
#include <memory>
#include <thread>
#include <queue>
#include <atomic>
#include <condition_variable>
#include <type_traits>
#include <mutex>

namespace cosmo_ros {

using namespace cosmo;

//!
/*
 * @tparam Msg_         Message that will be stored in shared memory. Don not use dynamic sized memory and std::string!
 * @tparam MsgRos_      Message that will be published over ROS
 * @tparam Converter_   Optional converter which can convert the two messages.
 */
template<typename Msg_, typename MsgRos_, template<typename, typename> class Converter_ = ConversionTraits, unsigned int Alignment_ = DefaultAlignment>
class PublisherRos : public Publisher<Msg_, Alignment_> {
public:
    using BaseType = Publisher<Msg_, Alignment_>;

    PublisherRos() = delete;

    /*!
     * @param topic          ROS topic name and identifier of the shared memory.
     */
    PublisherRos(const std::string& topic):
        PublisherRos(std::make_shared<PublisherRosOptions>(topic))
    {
    }

    /*!
     * @param options       Options
     */
    PublisherRos(const PublisherRosOptionsPtr& options):
        BaseType(options),
        options_(options),
        rosPublisher_(options->nodeHandle_.advertise<MsgRos_>(options_->topic_, options_->rosQueueSize_, options_->rosLatch_)),
        isRosPublisherThreadRunning_(false)
    {
        if(options_->autoPublishRos_) {
            // Create a separate thread to publish the ROS messages.
            // We don't want to do this in-place because the serialization can block the publish method too long.
            rosPublisherThread_ = std::thread{std::bind(&PublisherRos::sendRosWorker, this)};
            isRosPublisherThreadRunning_ = true;
        }
    }

    PublisherRos(PublisherRos&&) = default;

	~PublisherRos() override {
		isRosPublisherThreadRunning_ = false;
		cndPublishRosMsg_.notify_all();
		if (rosPublisherThread_.joinable()) {
			rosPublisherThread_.join();
		}
  }

  /*! Call this method to immediately publish a message over shared memory and enque to ROS.
   *  The message will be automatically converted to a ROS message.
   *  If the publisher is not configured with autoPublishRos_ = true, sendRos() needs to be called manually.
   *
   *  Note that this method is designed for a realtime thread, and thus non-blocking.
   *
   *  @param   message		message to publish
   *  @param   maxLockTime    maximum lock time of this method.
   *
   */
  template<typename T = Msg_, typename U = MsgRos_, template<typename, typename> class C = Converter_>
  bool publish(const Msg_& message, const std::chrono::microseconds& maxLockTime = std::chrono::microseconds{1000}, typename std::enable_if<hasConvert<T, U, C>::value>::type* = 0) {
      // Write message to shared memory.
      bool sendSuccessful = BaseType::publish(message, maxLockTime);

      // Stage the message.
      if(getNumRosSubscribers() > 0u || options_->rosLatch_) {
          sendSuccessful &= stageMessage(Converter_<Msg_,MsgRos_>::convert(message), maxLockTime);
      }else{
#ifdef COSMO_DEBUG_LEVEL2
          MELO_INFO("\e[0;31m<== No ROS subscriber, not sending message: Topic: %s \e[0m", options_->topic_.c_str());
#endif
      }
      return sendSuccessful;
  }


  /*! Call this method to immediately publish a message over shared memory and enque to ROS.
   *  If the publisher is not configured with autoPublishRos_ = true, sendRos() needs to be called manually.
   *
   *  Note that this method is designed for a realtime thread, and thus non-blocking.
   *
   *
   *  @param   shmMessage	  message to publish over shared memory
   *  @param   rosMessage     message to publish over ROS
   *  @param   maxLockTime    maximum lock time of this method.
   *
   */
  virtual bool publish(const Msg_& shmMessage, const MsgRos_& rosMessage, const std::chrono::microseconds& maxLockTime = std::chrono::microseconds{1000}) {
      // Write message to shared memory.
      bool sendSuccessful = BaseType::publish(shmMessage, maxLockTime);

      // Stage the message, but only if there is a  ROS subscriber
      if(rosPublisher_.getNumSubscribers() > 0u || options_->rosLatch_) {
          sendSuccessful &= stageMessage(rosMessage, maxLockTime);
      } else {
#ifdef COSMO_DEBUG_LEVEL2
          MELO_INFO("\e[0;31m<== No ROS subscriber, not sending message: Topic: %s \e[0m", options_->topic_.c_str());
#endif
      }

      return sendSuccessful;
  }



    /*!
     * Call this method to immediately publish a message over shared memory and ROS.
     * The message will be automatically converted to a ROS message.
     *
     * Note that this method directly publishes over ROS, it may be not suitable for realtime critical loops.
     * Use publish(..) and sendRos() separately to increase realtime capability.
     *
     * @param message       message to publish
     */
    template<typename T = Msg_, typename U = MsgRos_, template<typename, typename> class C = Converter_>
    bool publishAndSend(const Msg_& message, const std::chrono::microseconds& maxLockTime = std::chrono::microseconds{1000}, typename std::enable_if<hasConvert<T, U, C>::value>::type* = 0)
    {
        bool sendSuccessful = BaseType::publish(message, maxLockTime);
        if(rosPublisher_.getNumSubscribers() > 0u || options_->rosLatch_) {
            rosPublisher_.publish( Converter_<Msg_,MsgRos_>::convert(message) );
#ifdef COSMO_DEBUG_LEVEL2
            MELO_INFO("\e[1;36m<== Sent message over ROS: Topic: %s \e[0m", options_->topic_.c_str());
#endif
        } else {
#ifdef COSMO_DEBUG_LEVEL2
            MELO_INFO("\e[0;31m<== No ROS subscriber, not sending message: Topic: %s \e[0m", options_->topic_.c_str());
#endif
        }
        return sendSuccessful;
    }

    /*!
     * Call this method to immediately publish a message over shared memory and ROS.
     *
     * Note that this method directly publishes over ROS, it may be not suitable for realtime critical loops.
     * Use publish(..) and sendRos() separately to increase realtime capability.
     *
     * @param shmMessage    message to publish over shared memory
     * @param rosMessage    message to publish over ROS
     */
    bool publishAndSend(const Msg_& shmMessage, const MsgRos_& rosMessage, const std::chrono::microseconds& maxLockTime = std::chrono::microseconds{1000}) {
        bool sendSuccessful = BaseType::publish(shmMessage, maxLockTime);
        if(rosPublisher_.getNumSubscribers() > 0u || options_->rosLatch_) {
            rosPublisher_.publish( rosMessage );
#ifdef COSMO_DEBUG_LEVEL2
            MELO_INFO("\e[1;36m<== Sent message over ROS: Topic: %s \e[0m", options_->topic_.c_str());
#endif
        } else {
#ifdef COSMO_DEBUG_LEVEL2
            MELO_INFO("\e[0;31m<== No ROS subscriber, not sending message: Topic: %s \e[0m", options_->topic_.c_str());
#endif
        }
        return sendSuccessful;
    }

    /*!
     *  Send all messages in the queue over ROS. If the publisher is not configured with autoPublishRos_ = true, this method needs to be called manually.
     */
  void sendRos() {
    std::lock_guard<std::timed_mutex> guard(mutexMsgQueue_);
    sendRosWithoutLock();
  }

    /*!
     *
     * @return the number of ROS subscribers.
     */
    inline uint32_t getNumRosSubscribers() const {
        return rosPublisher_.getNumSubscribers();
    }

    /*!
     * Returns the total number of subscribers, including all channels (shared memory, ros, ..)
     * @return  Total number of publishers
     */
    unsigned int getNumSubscribers() const override {
        return this->getNumShmSubscribers() + getNumRosSubscribers();
    }

protected:

  /*!
   *  This method puts the message on the queue and notifies the publisher worker to publish the message over ROS.
   *  Note that this method is designed for a realtime thread, and thus non-blocking.
   */
  bool stageMessage(const MsgRos_& rosMessage, const std::chrono::microseconds& maxLockTime) {
      bool stageSuccessful = false;
      std::unique_lock<std::timed_mutex> lock(mutexMsgQueue_, std::defer_lock);
      if (lock.try_lock_for(maxLockTime)) {
          msgQueue_.emplace(rosMessage);
          stageSuccessful = true;
          lock.unlock();
          cndPublishRosMsg_.notify_all();
      }
      return stageSuccessful;
  }

  void sendRosWithoutLock() {
    // Remove old messages. We want to do this here and not in the publish() method.
    while(msgQueue_.size() > options_->rosQueueSize_) {
      msgQueue_.pop();
    }

    // Publish all messages that are queued.
    while (!msgQueue_.empty()) {
      // no need to check for number of subscribers here, this is done before putting new messages in the queue
      rosPublisher_.publish( msgQueue_.front() );
      msgQueue_.pop();

#ifdef DEBUG_COSMO
      MELO_INFO("\e[1;36m<== Sent message over ROS: Topic: %s \e[0m", options_->topic_.c_str());
#endif
    }
  }

  /*!
   * This worker runs in a separate thread and publishes all messages in the queue over ROS.
   */
  void sendRosWorker() {
     while (isRosPublisherThreadRunning_) {
         std::unique_lock<std::timed_mutex> lock(mutexMsgQueue_);
         cndPublishRosMsg_.wait(lock,[this]() { return (!msgQueue_.empty() || !isRosPublisherThreadRunning_); });
         // Stop thread immediately, if it has been stopped.
         if (!isRosPublisherThreadRunning_) {
             return;
         }
         sendRosWithoutLock();
     }
  }

protected:
    const std::shared_ptr<const PublisherRosOptions> options_;

    ros::Publisher rosPublisher_;

    std::thread rosPublisherThread_;
    std::atomic<bool> isRosPublisherThreadRunning_;
    std::condition_variable_any cndPublishRosMsg_;
    std::queue<MsgRos_> msgQueue_;
    std::timed_mutex mutexMsgQueue_;
};

template<typename Msg_, typename MsgRos_, template<typename, typename> class Converter_ = ConversionTraits, unsigned int Alignment_ = DefaultAlignment>
using PublisherRosPtr = std::shared_ptr<PublisherRos<Msg_, MsgRos_, Converter_, Alignment_>>;

} // namespace
