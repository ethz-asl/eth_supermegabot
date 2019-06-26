/*!
 * @file	SubscriberRos.hpp
 * @author	Philipp Leemann, Christian Gehring
 * @date	May 24, 2017
 */

#pragma once

#include "cosmo/Subscriber.hpp"
#include "cosmo_ros/SubscriberRosOptions.hpp"
#include "cosmo_ros/conversion.hpp"

#include "message_logger/message_logger.hpp"

#include <ros/subscriber.h>
#include <ros/callback_queue.h>

#include <memory>
#include <mutex>

namespace cosmo_ros {

using namespace cosmo;


/**
 * @brief      Class for subscriber ros.
 *
 * @tparam     Msg_        Message that will be stored in shared memory. Don not
 *                         use dynamic sized memory and std::string!
 * @tparam     MsgRos_     Message that will be published over ROS
 * @tparam     Converter_  Optional converter which can convert the two
 *                         messages.
 * @tparam     Alignment_  Alignment of the memory allocation, has to be a
 *                         multiple of size of a SIMD memory read instruction
 *                         (128bits)
 */
template<typename Msg_, typename MsgRos_, template<typename, typename> class Converter_ = ConversionTraits, unsigned int Alignment_ = DefaultAlignment>
class SubscriberRos : public Subscriber<Msg_, Alignment_> {

  static_assert(hasConvert<Msg_, MsgRos_, Converter_>::value, "Subscriber needs a valid Converter_!");

public:
  using BaseType = Subscriber<Msg_, Alignment_>;
  using SubscriberCallback = typename BaseType::SubscriberCallback;
  using Options = SubscriberRosOptions<Msg_>;

  SubscriberRos() = delete;

  /**
   *
   * @param[in]  topic       ROS topic name and identifier of the shared memory.
   * @param[in]  callback    Subscriber callback
   * @param[in]  nodeHandle  ROS node handle
   */
  SubscriberRos(const std::string& topic, const SubscriberCallback& callback, const ros::NodeHandle& nodeHandle):
      SubscriberRos(std::make_shared<Options>(topic, callback, nodeHandle))
  {
  }

  /**
   *
   * @param[in]  options  Options
   */
  SubscriberRos(const std::shared_ptr<Options>& options):
      BaseType(options),
      options_(options),
      receivedRosMsg_{false},
      rosSubscriber_(),
      rosCallbackQueue_(),
      rosCommunicationActive_(false)
  {
      if(this->getNumShmPublishers() == 0) {
          subscribeRos();
      }
  }

  SubscriberRos(SubscriberRos&&) = default;

  ~SubscriberRos() override
  {
    this->stop(true);
  }

  /**
   * @brief      Call this method to receive a message over shared memory and
   *             over ros
   *
   * @param[in]  maxLockTime  Maximum lock time for the base class receive
   *                          method
   *
   * @return     Returns true if a new message has been received
   */
  bool receive(const std::chrono::microseconds& maxLockTime = std::chrono::microseconds{200}) override {

    if(this->getNumShmPublishers() == 0) {
      // No shared memory publisher is available.
      if(!rosCommunicationActive_ && options_->tryRosResubscribing_) {
          subscribeRos();
      }

      receivedRosMsg_ = false;
      rosCallbackQueue_.callAvailable(ros::WallDuration(maxLockTime.count()/1000000, (maxLockTime.count()%1000000)*1000));

      return receivedRosMsg_;
    } else {
      /* shut down ros subscriber if we have a communication over shared memory,
       * to save the shm+ros publisher the effort to do the ros publishing
       */
      if(rosCommunicationActive_) {
          unsubscribeRos();
      }

      return BaseType::receive(maxLockTime);
    }
  }

  /**
   * @brief      Wraps the shared memory converter into a ROS callback
   *
   * @param[in]  msgRos  The incoming ROS message
   */
  void rosCallback(const boost::shared_ptr<MsgRos_ const>& msgRos) {
#ifdef COSMO_DEBUG_LEVEL2
      MELO_INFO("\e[1;32m==> Received message over ROS: Topic: %s \e[0m", options_->topic_.c_str());
#endif
      receivedRosMsg_ = true;
      options_->callback_( Converter_<Msg_,MsgRos_>::convert(*msgRos) );
  }


    inline unsigned int getNumRosPublishers() const {
        return rosSubscriber_.getNumPublishers();
    }

    /*!
     * Returns the total number of publishers, including all channels (shared memory, ros, ..)
     * @return      Total number of publishers
     */
    unsigned int getNumPublishers() const override {
        return this->getNumShmPublishers() + getNumRosPublishers();
    }

    void stop(const bool wait=true) override {
        rosCallbackQueue_.disable(); // wake up blocking rosCallbackQueue
        BaseType::stop(wait);
        unsubscribeRos();
    }

protected:

  inline void subscribeRos() {
#ifdef COSMO_DEBUG_LEVEL2
    MELO_INFO("Resubscribing to ROS: Topic: %s", options_->topic_.c_str());
#endif
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<MsgRos_>(options_->topic_,
                                                                       options_->rosQueueSize_,
                                                                       boost::bind(&SubscriberRos<Msg_, MsgRos_, Converter_, Alignment_>::rosCallback, this, _1),
                                                                       ros::VoidPtr(),
                                                                       &rosCallbackQueue_);

    ops.transport_hints = options_->rosTransportHints_;

    rosSubscriber_ = options_->nodeHandle_.subscribe( ops );
    rosCommunicationActive_ = true;
  }

  inline void unsubscribeRos() {
      // note that functions called in here have to be threadsafe with respect to rosCallbackQueue_.callAvailable
#ifdef COSMO_DEBUG_LEVEL2
      MELO_INFO("Unsubscribing from ROS: Topic: %s", options_->topic_.c_str());
#endif
      rosSubscriber_.shutdown();
      rosCommunicationActive_ = false;
  }

protected:
  const std::shared_ptr<const Options> options_;

  bool receivedRosMsg_;

  ros::Subscriber rosSubscriber_;
  ros::CallbackQueue rosCallbackQueue_;

  //! If true, a ROS subscriber has been created.
  bool rosCommunicationActive_;
};

template<typename Msg_, typename MsgRos_, template<typename, typename> class Converter_ = ConversionTraits, unsigned int Alignment_ = DefaultAlignment>
using SubscriberRosPtr = std::shared_ptr<SubscriberRos<Msg_, MsgRos_, Converter_, Alignment_>>;

} // namespace
