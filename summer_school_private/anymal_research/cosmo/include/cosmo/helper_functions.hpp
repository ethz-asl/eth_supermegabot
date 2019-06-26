/*!
 * @file	helper_functions.hpp
 * @author	Philipp Leemann
 * @date	May, 2017
 */

#pragma once

#include "cosmo/Publisher.hpp"
#include "cosmo/Subscriber.hpp"

#include <string>


namespace cosmo {

/**
 * @brief      Creates a new cosmo publisher
 *
 * @param[in]  topic  The message topic
 * @tparam     Msg_   Shared memory message struct type
 * @return     A shared ptr to the new publisher
 */
template<typename Msg_>
inline PublisherPtr<Msg_> advertiseShm(const std::string& topic)
{
    return advertiseShm<Msg_>(std::make_shared<PublisherOptions>(topic));
}

/**
 * @brief      Creates a new cosmo publisher
 *
 * @param[in]  options  Shared pointer to the options
 *
 * @tparam     Msg_     Shared memory message struct type
 *
 * @return     A shared ptr to the new publisher
 */
template<class Msg_>
inline PublisherPtr<Msg_> advertiseShm(const PublisherOptionsPtr& options)
{
    return PublisherPtr<Msg_>(new Publisher<Msg_>( options ));
}

/**
 * @brief      Creates a new cosmo subscriber
 *
 * @param[in]  topic      The topic name
 * @param[in]  fp         callback function pointer
 * @param      obj        pointer to the object owning the callback function
 *
 * @tparam     Msg_       Shared memory message struct type
 * @tparam     Obj_       Type of the object which owns the callback function
 *
 * @return     Shared ptr to the new subscriber
 */
template<class Msg_, class Obj_>
inline SubscriberPtr<Msg_> subscribeShm(const std::string& topic, void(Obj_::*fp)(const Msg_&), Obj_* obj)
{
    return subscribeShm(std::make_shared<SubscriberOptions<Msg_>>(topic, std::bind(fp, obj, std::placeholders::_1)));
}

/**
 * @brief      Creates a new cosmo subscriber
 *
 * @param[in]  options  Shared pointer to the options
 *
 * @tparam     Msg_     Shared memory message struct type
 *
 * @return     Shared ptr to the new subscriber
 */
template<class Msg_>
inline SubscriberPtr<Msg_> subscribeShm(const SubscriberOptionsPtr<Msg_>& options)
{
    SubscriberPtr<Msg_> sub(new Subscriber<Msg_>( options ));
    if(!options->deferStart_) {
        sub->start();
    }
    return sub;
}


} // namespace 
