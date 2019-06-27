/*!
 * @file	SubscriberOptions.hpp
 * @author	Philipp Leemann, Christian Gehring
 * @date	May 24, 2017
 */

#pragma once

#include "cosmo/CommunicatorOptions.hpp"

#include <memory>
#include <functional>
#include <chrono>

namespace cosmo {

template<typename MessageType_>
class SubscriberOptions : public CommunicatorOptions
{
public:
    using SubscriberCallback = std::function<void(const MessageType_&)>;

    /**
     * @brief      Creates subscriber options
     *
     * @param[in]  topic     The message topic
     * @param[in]  callback  A function object for handling incoming messages
     */
    SubscriberOptions(const std::string& topic, const SubscriberCallback& callback):
      SubscriberOptions(topic, callback, false, 0)
    {
    }

    /**
     * @brief      Creates subscriber options
     *
     * @param[in]  topic                   The message topic
     * @param[in]  callback                A function object for handling
     *                                     incoming messages
     * @param[in]  autoSubscribe           The callback method gets called in a
     *                                     separate thread if autoSubscribe is
     *                                     set to true
     * @param[in]  autoSubscriberPriority  Defines the priority of the
     *                                     subscriber thread if autosubscribe is
     *                                     set to true
     */
    SubscriberOptions(const std::string& topic,
                      const SubscriberCallback& callback,
                      bool autoSubscribe,
                      bool autoSubscriberPriority):
            CommunicatorOptions(topic),
            callback_(callback),
            autoSubscribe_(autoSubscribe),
            autoSubscriberPriority_(autoSubscriberPriority)
    {
    }

    ~SubscriberOptions() override = default;

public:
    //! Callback method, which is called when a new message has arrived.
    //! The arrived message is passed as const reference to the callback function. Note that the reference points to shared memory,
    //! so you should copy the data in the callback and not e.g. save it down as a pointer
    SubscriberCallback callback_;

    //! If true, a background thread waits for incoming messages.
    bool autoSubscribe_;

    //! Priority of the subscriber thread (0...99) 0 = lowest prio
    int autoSubscriberPriority_;

    //! If set to true, the last message published to the topic will be received by the subscriber, even if it was sent before subscribing.
    bool latch_ = false;

    //! If set to true, the subscribe helper functions will not call start() on the created subscriber.
    bool deferStart_ = false;
};

template<typename MessageType_>
using SubscriberOptionsPtr = std::shared_ptr<SubscriberOptions<MessageType_>>;

} // namespace 
