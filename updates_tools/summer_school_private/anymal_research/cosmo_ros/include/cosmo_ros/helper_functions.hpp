/*!
 * @file	helper_functions.hpp
 * @author	Philipp Leemann, Christian Gehring
 * @date	May, 2017
 */

#pragma once

#include <string>
#include <cstdint>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include "param_io/get_param.hpp"
#include "param_io/set_param.hpp"

#include "cosmo_ros/PublisherRos.hpp"
#include "cosmo_ros/SubscriberRos.hpp"

#include "cosmo/helper_functions.hpp"

namespace cosmo_ros {

using namespace cosmo;

inline void addNamespaceToTopicName(ros::NodeHandle& nh, std::string& topicName) {
    if(topicName.at(0) != '/') {
        topicName = nh.getNamespace() + "/" + topicName;
    }
}

inline std::string getNamespacedTopicName(ros::NodeHandle& nh, const std::string& paramName, const std::string& defaultTopic) {
    std::string topicName = param_io::param<std::string>(nh, paramName, defaultTopic);
    addNamespaceToTopicName(nh, topicName);
    return topicName;
}

/*!
 * Creates a Publisher with topic name from the ROS parameter server.
 * @tparam Msg_         Shared memory message struct type
 * @param nh            ROS nodehandle
 * @param name          Name of the parameter
 * @param defaultTopic  Default topic name
 * @return              Shared pointer containing the Publisher
 */
template<typename Msg_>
inline PublisherPtr<Msg_> advertiseShm(ros::NodeHandle& nh, const std::string& name, const std::string& defaultTopic)
{
    return advertiseShm<Msg_>( getNamespacedTopicName(nh, "publishers/"+name+"/topic", defaultTopic) );
}

/*!
 * Creates a PublisherRos with topic name, ros queue size and 'latch' from ROS parameter server
 * @tparam Msg_         Shared memory message struct type
 * @tparam MsgRos_      ROS message type
 * @tparam Converter_   Converter between shared memory and ROS message
 * @param nh            ROS node handle
 * @param name          Name of the parameter
 * @param defaultTopic  Default topic name
 * @return              Shared pointer containing the PublisherRos
 */
template<typename Msg_, typename MsgRos_, template<typename, typename> class Converter_ = ConversionTraits>
inline PublisherRosPtr<Msg_, MsgRos_, Converter_> advertiseShmRos(ros::NodeHandle& nh,
                                                                  const std::string& name,
                                                                  const std::string& defaultTopic)
{
    return advertiseShmRos<Msg_, MsgRos_, Converter_>(name, std::make_shared<PublisherRosOptions>(param_io::param<std::string>(nh, "publishers/"+name+"/topic", defaultTopic), nh));
}

/*! Creates a PublisherRos with topic name, ros queue size and 'latch' from the ROS parameter server.
 * The ROS publisher options (topic, queue size, latch) will be read from the ROS parameter "publishers/parameterName".
 * If the parameters do not exist, the ones in the options are used by default.
 * @tparam Msg_         Shared memory message struct type
 * @tparam MsgRos_      ROS message type
 * @tparam Converter_   Converter between shared memory and ROS message
 * @param name          name of the ROS parameter
 * @param options       default and additional options, containing the nodeHandle
 * @return              Shared pointer containing the PublisherRos
 */
template<typename Msg_, typename MsgRos_, template<typename, typename> class Converter_ = ConversionTraits>
inline PublisherRosPtr<Msg_, MsgRos_, Converter_> advertiseShmRos(const std::string& name,
                                                                  const PublisherRosOptionsPtr& options)
{
    options->topic_        = param_io::param<std::string>(options->nodeHandle_, "publishers/"+name+"/topic", options->topic_);
    options->rosQueueSize_ = param_io::param<int>(options->nodeHandle_, "publishers/"+name+"/queue_size", options->rosQueueSize_);
    options->rosLatch_     = param_io::param<bool>(options->nodeHandle_, "publishers/"+name+"/latch", options->rosLatch_);
    return advertiseShmRos<Msg_, MsgRos_, Converter_>(options);
}


/*!
 * Creates a PublisherRos with given topic name
 * @tparam Msg_         Shared memory message struct type
 * @tparam MsgRos_      ROS message type
 * @tparam Converter_   Converter between shared memory and ROS message
 * @param nh            ROS node handle
 * @param name          name of the topic
 * @return              Shared pointer containing the PublisherRos
 */
template<typename Msg_, typename MsgRos_, template<typename, typename> class Converter_ = ConversionTraits>
inline PublisherRosPtr<Msg_, MsgRos_, Converter_> advertiseShmRos(ros::NodeHandle& nh,
                                                                  const std::string& name)
{
    return advertiseShmRos<Msg_, MsgRos_, Converter_>( std::make_shared<PublisherRosOptions>(name, nh) );
}


/*!
 * Creates a PublisherRos with given Options
 * @tparam Msg_         Shared memory message struct type
 * @tparam MsgRos_      ROS message type
 * @tparam Converter_   Converter between shared memory and ROS message
 * @param options       Shared pointer to the Options
 * @return              Shared pointer to the PublisherRos
 */
template<typename Msg_, typename MsgRos_, template<typename, typename> class Converter_ = ConversionTraits>
inline PublisherRosPtr<Msg_, MsgRos_, Converter_> advertiseShmRos(const std::shared_ptr<PublisherRosOptions>& options)
{
    addNamespaceToTopicName(options->nodeHandle_, options->topic_);
    return PublisherRosPtr<Msg_, MsgRos_, Converter_>( new PublisherRos<Msg_, MsgRos_, Converter_>(options) );
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////


/*!
 * Creates a Subscriber with topic name from ROS parameter server.
 * @tparam Msg_         Shared memory message struct type
 * @tparam Obj_         Type of the object which owns the callback function
 * @param nh            ROS node handle
 * @param name          name of the ROS parameter
 * @param defaultTopic  Default name of the topic
 * @param fp            callback function pointer
 * @param obj           pointer to the object owning the callback function
 * @return              Shared pointer to the Subscriber
 */
template<class Msg_, class Obj_>
inline SubscriberPtr<Msg_> subscribeShm(ros::NodeHandle& nh,
                                        const std::string& name,
                                        const std::string& defaultTopic,
                                        void(Obj_::*fp)(const Msg_&), Obj_* obj)
{
    auto options = std::make_shared<SubscriberOptions>(getNamespacedTopicName(nh, "subscribers/"+name+"/topic", defaultTopic),
                                                       std::bind(fp, obj, std::placeholders::_1));
    options->latch_ = param_io::param<bool>(nh, "subscribers/"+name+"/latch", options->latch_);
    return subscribeShm<Msg_>(options);
}


/*!
 * Creates a SubscriberRos with topic name, ros queue size and 'latch' from ROS parameter server.
 * @tparam Msg_         Shared memory message struct type
 * @tparam MsgRos_      ROS message type
 * @tparam Converter_   Converter between shared memory and ROS message
 * @tparam Obj_         Type of the object which owns the callback function
 * @param nh            ROS node handle
 * @param name          name of the ROS parameter
 * @param defaultTopic  Default name of the topic
 * @param fp            callback function pointer
 * @param obj           pointer to the object owning the callback function
 * @return              Shared pointer to the SubscriberRos
 */
template<typename Msg_, typename MsgRos_, template<typename, typename> class Converter_ = ConversionTraits, typename Obj_>
inline SubscriberRosPtr<Msg_, MsgRos_, Converter_> subscribeShmRos(ros::NodeHandle& nh,
                                                                   const std::string& name,
                                                                   const std::string& defaultTopic,
                                                                   void(Obj_::*fp)(const Msg_&), Obj_* obj)
{
    auto options = std::make_shared<SubscriberRosOptions<Msg_>>(defaultTopic,
                                                                std::bind(fp, obj, std::placeholders::_1),
                                                                nh);
    return subscribeShmRos<Msg_, MsgRos_, Converter_>(name, options);
}

/*! Creates a SubscriberRos with topic name, ros queue size and 'latch' from the ROS parameter server.
 * The ROS subscriber options (topic, queue size) will be read from the ROS parameter "subscribers/parameterName".
 * If the parameters do not exist, the ones in the options are used by default.
 * @tparam Msg_         Shared memory message struct type
 * @tparam MsgRos_      ROS message type
 * @tparam Converter_   Converter between shared memory and ROS message
 * @param name          Name of the ROS parameter
 * @param options       Default and additional options, containing the ROS nodehandle
 * @return              Shared pointer to the SubscriberRos
 */
template<typename Msg_, typename MsgRos_,  template<typename, typename> class Converter_ = ConversionTraits>
inline SubscriberRosPtr<Msg_, MsgRos_, Converter_> subscribeShmRos(const std::string& name,
                                                                   const SubscriberRosOptionsPtr<Msg_>& options)
{
    options->topic_         = param_io::param<std::string>(options->nodeHandle_, "subscribers/"+name+"/topic", options->topic_);
    options->rosQueueSize_  = param_io::param<int>(options->nodeHandle_, "subscribers/"+name+"/queue_size", options->rosQueueSize_);
    options->latch_         = param_io::param<bool>(options->nodeHandle_, "subscribers/"+name+"/latch", options->latch_);
    return subscribeShmRos<Msg_, MsgRos_, Converter_>(options);
}


/*!
 * Creates a SubscriberRos with given options
 * @tparam Msg_         Shared memory message struct type
 * @tparam MsgRos_      ROS message type
 * @tparam Converter_   Converter between shared memory and ROS message
 * @param options       Shared pointer to the Options
 * @return              Shared pointer to the SubscriberRos
 */
template<typename Msg_, typename MsgRos_,  template<typename, typename> class Converter_ = ConversionTraits>
inline SubscriberRosPtr<Msg_, MsgRos_, Converter_> subscribeShmRos(const SubscriberRosOptionsPtr<Msg_>& options)
{
    addNamespaceToTopicName(options->nodeHandle_, options->topic_);
    SubscriberRosPtr<Msg_, MsgRos_, Converter_> sub(new SubscriberRos<Msg_, MsgRos_, Converter_>( options ));
    if(!options->deferStart_) {
        sub->start();
    }
    return sub;
}

} // namespace
