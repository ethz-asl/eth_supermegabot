/*!
 * @file	Topic.hpp
 * @author	Philipp Leemann
 * @date	Jun 30, 2016
 */

#pragma once

#include <string>
#include <cstdint>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/service_server.h>
#include <ros/service_client.h>

#include "any_node/Param.hpp"
#include "any_node/ThreadedPublisher.hpp"
#include "any_node/ThrottledSubscriber.hpp"

namespace any_node {

template<typename msg>
ros::Publisher advertise(ros::NodeHandle& nh, const std::string& name, const std::string& defaultTopic, uint32_t queue_size, bool latch = false)
{
	return nh.advertise<msg>(
			param<std::string>(nh, "publishers/"+name+"/topic", defaultTopic),
			param<int>(nh, "publishers/"+name+"/queue_size", queue_size),
			param<bool>(nh, "publishers/"+name+"/latch", latch));
}

template<typename msg>
ThreadedPublisherPtr<msg> threadedAdvertise(ros::NodeHandle& nh, const std::string& name, const std::string& defaultTopic, uint32_t queue_size, bool latch = false, unsigned int maxMessageBufferSize = 10)
{
    return ThreadedPublisherPtr<msg>(new ThreadedPublisher<msg>(advertise<msg>(nh, name, defaultTopic, queue_size, latch), maxMessageBufferSize));
}

template<class M, class T>
ros::Subscriber subscribe(ros::NodeHandle& nh, const std::string& name, const std::string& defaultTopic,
		uint32_t queue_size, void(T::*fp)(const boost::shared_ptr<M const>&), T* obj, const ros::TransportHints& transport_hints = ros::TransportHints())
{
    if(nh.param<bool>("subscribers/"+name+"/deactivate", false)) {
        return ros::Subscriber(); // return empty subscriber
    }else{
        return nh.subscribe(
			param<std::string>(nh, "subscribers/"+name+"/topic", defaultTopic),
			param<int>(nh, "subscribers/"+name+"/queue_size", queue_size),
			fp, obj,
			transport_hints);
    }
}

template<class M, class T>
ThrottledSubscriberPtr<M,T> throttledSubscribe(double timeStep, ros::NodeHandle& nh, const std::string& name, const std::string& defaultTopic, uint32_t queue_size,
                                            void(T::*fp)(const boost::shared_ptr<M const>&), T* obj, const ros::TransportHints& transport_hints = ros::TransportHints())
{
  if(nh.param<bool>("subscribers/"+name+"/deactivate", false)) {
      return ThrottledSubscriberPtr<M,T>( new ThrottledSubscriber<M,T>() ); // return empty subscriber
  }else{
      return ThrottledSubscriberPtr<M,T>( new ThrottledSubscriber<M,T>(timeStep, nh, param<std::string>(nh, "subscribers/"+name+"/topic", defaultTopic),
    		  param<int>(nh, "subscribers/"+name+"/queue_size", queue_size), fp, obj, transport_hints));
  }
}

template<class T, class MReq, class MRes>
ros::ServiceServer advertiseService(ros::NodeHandle& nh, const std::string& name, const std::string& defaultService, bool(T::*srv_func)(MReq &, MRes &), T *obj)
{
	return nh.advertiseService(
			param<std::string>(nh, "servers/"+name+"/service", defaultService),
			srv_func, obj);

}

template<class MReq, class MRes>
ros::ServiceClient serviceClient(ros::NodeHandle& nh, const std::string& name, const std::string& defaultService, const ros::M_string& header_values = ros::M_string())
{
    return nh.serviceClient<MReq, MRes>(
            param<std::string>(nh, "clients/"+name+"/service", defaultService),
            param<bool>(nh, "clients/"+name+"/persistent", false), header_values);
}

template<class Service>
ros::ServiceClient serviceClient(ros::NodeHandle& nh, const std::string& name, const std::string& defaultService, const ros::M_string& header_values = ros::M_string())
{
    return nh.serviceClient<Service>(
            param<std::string>(nh, "clients/"+name+"/service", defaultService),
			param<bool>(nh, "clients/"+name+"/persistent", false), header_values);
}

} // namespace any_node
