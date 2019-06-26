/*!
 * @file    ThreadedPublisher.hpp
 * @author  Remo Diethelm
 * @date    Nov 30, 2016
 */

#pragma once


// c++
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

// ros
#include <ros/ros.h>

// message logger
#include "message_logger/message_logger.hpp"


namespace any_node {


template <typename MessageType>
class ThreadedPublisher
{
protected:
    mutable std::mutex publisherMutex_;
    ros::Publisher publisher_;

    std::mutex messageBufferMutex_;
    std::queue<MessageType> messageBuffer_;
    unsigned int maxMessageBufferSize_ = 0;
    bool autoPublishRos_;

    std::thread thread_;
    std::mutex notifyThreadMutex_;
    std::condition_variable notifyThreadCv_;
    std::atomic<bool> notifiedThread_;
    std::atomic<bool> shutdownRequested_;

public:
    ThreadedPublisher(const ros::Publisher& publisher, unsigned int maxMessageBufferSize = 10, bool autoPublishRos = true)
    :   publisher_(publisher),
        maxMessageBufferSize_(maxMessageBufferSize),
        autoPublishRos_(autoPublishRos),
        notifiedThread_(false),
        shutdownRequested_(false)
    {
        if(autoPublishRos_) {
            thread_ = std::thread(&ThreadedPublisher::threadedPublish, this);
        }
    }

    virtual ~ThreadedPublisher()
    {
        shutdown();
    }

    void publish(const boost::shared_ptr<MessageType>& message)
    {
        addMessageToBuffer(*message);
    }

    void publish(const MessageType& message)
    {
        addMessageToBuffer(message);
    }

    void shutdown()
    {
        // Prohibit shutting down twice.
        if (shutdownRequested_)
          return;

        // Shutdown thread if autopublishing
        if(autoPublishRos_) {
            shutdownRequested_ = true;
            notifyThread();
            thread_.join();
        }
        
        std::lock_guard<std::mutex> publisherLock(publisherMutex_);
        publisher_.shutdown();
    }

    std::string getTopic() const
    {
        std::lock_guard<std::mutex> publisherLock(publisherMutex_);
        return publisher_.getTopic();
    }

    uint32_t getNumSubscribers() const
    {
        std::lock_guard<std::mutex> publisherLock(publisherMutex_);
        return publisher_.getNumSubscribers();
    }

    bool isLatched() const
    {
        std::lock_guard<std::mutex> publisherLock(publisherMutex_);
        return publisher_.isLatched();
    }

    /*!
     * Send all messages in the buffer.
     */
    void sendRos()
    {
        // Publish all messages in the buffer; stop the thread in case of a shutdown.
        while (!shutdownRequested_)
        {
            // Execute the publishing with a copied message object.
            MessageType message;
            {
                std::lock_guard<std::mutex> messageBufferLock(messageBufferMutex_);
                if (messageBuffer_.empty())
                    break;
                message = messageBuffer_.front();
                messageBuffer_.pop();
            }
            {
                std::lock_guard<std::mutex> publisherLock(publisherMutex_);
                publisher_.publish(message);
            }
        }
    }

protected:
    void addMessageToBuffer(const MessageType& message)
    {
        {
            std::lock_guard<std::mutex> messageBufferLock(messageBufferMutex_);
            if (messageBuffer_.size() == maxMessageBufferSize_)
            {
                MELO_ERROR("Threaded publisher: Message buffer reached max size, discarding oldest message without publishing.");
                messageBuffer_.pop();
            }
            messageBuffer_.push(message);
        }
        notifyThread();
    }

    void notifyThread()
    {
        std::unique_lock<std::mutex> lock(notifyThreadMutex_);
        notifiedThread_ = true;
        notifyThreadCv_.notify_all();
    }

    void threadedPublish()
    {
        // Stop the thread in case of a shutdown.
        while (!shutdownRequested_)
        {
            // Wait for the notification.
            {
                std::unique_lock<std::mutex> notifyThreadLock(notifyThreadMutex_);
                while (!notifiedThread_) // Additional bool protecting against spurious wake ups.
                    notifyThreadCv_.wait(notifyThreadLock);
                notifiedThread_ = false;
            }

            // Publish all messages in the buffer; stop the thread in case of a shutdown.
            sendRos();
        }
    }
};

template <typename MessageType>
using ThreadedPublisherPtr = std::shared_ptr<ThreadedPublisher<MessageType>>;


} // any_node
