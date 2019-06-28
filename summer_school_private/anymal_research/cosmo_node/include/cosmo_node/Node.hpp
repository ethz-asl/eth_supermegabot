/*!
 * @file	Node.hpp
 * @author	Philipp Leemann
 * @date	July, 2016
 */

#pragma once

#include "any_node/Node.hpp"

#include "cosmo_ros/cosmo_ros.hpp"

namespace cosmo_node {

class Node : public any_node::Node {
 public:
    using BaseType = any_node::Node;

    Node() = delete;

    Node(NodeHandlePtr nh):
            BaseType(nh)
    {
    }

    ~Node() override = default;

    /*
     * forwarding to Topic.hpp functions
     */
    // todo: add wrappers for all possible helper functions

    /*!
     * Creates a PublisherRos with topic name form ROS parameter server
     * @tparam Msg_         Shared memory message struct type
     * @tparam MsgRos_      ROS message type
     * @tparam Converter_   Converter between shared memory and ROS message
     * @param name          Name of the ROS parameter
     * @param defaultTopic  Default name of the topic
     * @return              A shared pointer to the PublisherRos
     */
    template<typename Msg_, typename MsgRos_, template<typename, typename> class Converter_ = cosmo_ros::ConversionTraits>
    inline cosmo_ros::PublisherRosPtr<Msg_, MsgRos_, Converter_> advertiseShmRos(const std::string& name, const std::string& defaultTopic)
    {
        return cosmo_ros::advertiseShmRos<Msg_, MsgRos_, Converter_>(*nh_, name, defaultTopic);
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
    template<typename Msg_, typename MsgRos_, template<typename, typename> class Converter_ = cosmo_ros::ConversionTraits>
    inline cosmo_ros::PublisherRosPtr<Msg_, MsgRos_, Converter_> advertiseShmRos(const std::string& name,
                                                                                 const cosmo_ros::PublisherRosOptionsPtr& options)
    {
        return cosmo_ros::advertiseShmRos<Msg_, MsgRos_, Converter_>(name, options);
    }

    /*!
     * Creates a PublisherRos with given options. This function overwrites the nodehandle given in the options.
     * @tparam Msg_         Shared memory message struct type
     * @tparam MsgRos_      ROS message type
     * @tparam Converter_   Converter between shared memory and ROS message
     * @param options       Shared pointer to the options
     * @return              Shared pointer to the PublisherRos
     */
    template<typename Msg_, typename MsgRos_, template<typename, typename> class Converter_ = cosmo_ros::ConversionTraits>
    inline cosmo_ros::PublisherRosPtr<Msg_, MsgRos_, Converter_> advertiseShmRos(const std::shared_ptr<cosmo_ros::PublisherRosOptions>& options)
    {
        options->nodeHandle_ = *nh_;
        return cosmo_ros::advertiseShmRos<Msg_, MsgRos_, Converter_>(options);
    }



    /*!
     *
     * @tparam Obj_         Type of the object which owns the callback function
     * @tparam Msg_         Shared memory message struct type
     * @tparam MsgRos_      ROS message type
     * @tparam Converter_   Converter between shared memory and ROS message
     * @param name          Name of the ROS parameter
     * @param defaultTopic  Default name of the topic
     * @param fp            Pointer to the callback function
     * @param obj           Pointer to the object owning the callback function
     * @return              Shared pointer to the SubscriberRos
     */
    template<typename Msg_, typename MsgRos_, template<typename, typename> class Converter_ = cosmo_ros::ConversionTraits, typename Obj_>
    inline cosmo_ros::SubscriberRosPtr<Msg_, MsgRos_, Converter_> subscribeShmRos(const std::string& name,
                                                                            const std::string& defaultTopic,
                                                                            void(Obj_::*fp)(const Msg_&), Obj_* obj)
    {
        return cosmo_ros::subscribeShmRos<Msg_, MsgRos_, Converter_, Obj_>(*nh_, name, defaultTopic, fp, obj);
    }


    /*! Creates a SubscriberRos with topic name from the ROS parameter server. This function overwrites the nodehandle given in the options.
     * The ROS subscriber options (topic, queue size) will be read from the ROS parameter "subscribers/parameterName".
     * If the parameters do not exist, the ones in the options are used by default.
     * @tparam Msg_         Shared memory message struct type
     * @tparam MsgRos_      ROS message type
     * @tparam Converter_   Converter between shared memory and ROS message
     * @param name          Name of the ROS parameter
     * @param options       Default and additional options, containing the ROS nodehandle
     * @return              Shared pointer to the SubscriberRos
     */
    template<typename Msg_, typename MsgRos_,  template<typename, typename> class Converter_ = cosmo_ros::ConversionTraits>
    inline cosmo_ros::SubscriberRosPtr<Msg_, MsgRos_, Converter_> subscribeShmRos(const std::string& name,
                                                                                  const cosmo_ros::SubscriberRosOptionsPtr<Msg_>& options)
    {
        options->nodeHandle_ = *nh_;
        return cosmo_ros::subscribeShmRos<Msg_, MsgRos_, Converter_>(name, options);
    }

    /*!
     * Creates a SubscriberRos with given options. This function overwrites the nodehandle given in the options.
     * @tparam Msg_         Shared memory message struct type
     * @tparam MsgRos_      ROS message type
     * @tparam Converter_   Converter between shared memory and ROS message
     * @param options       Shared pointer to the options
     * @return              Shared pointer to the SubscriberRos
     */
    template<typename Msg_, typename MsgRos_, template<typename, typename> class Converter_ = cosmo_ros::ConversionTraits>
    inline cosmo_ros::SubscriberRosPtr<Msg_, MsgRos_, Converter_> subscribeShmRos(const std::shared_ptr<cosmo_ros::SubscriberRosOptions<Msg_>>& options)
    {
        options->nodeHandle_ = *nh_;
        return cosmo_ros::subscribeShmRos<Msg_, MsgRos_, Converter_>(options);
    }
};

} // namespace any_node
