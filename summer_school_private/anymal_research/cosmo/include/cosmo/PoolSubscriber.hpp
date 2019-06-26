/*!
 * @file	PoolSubscriber.hpp
 * @author	Philipp Leemann
 * @date	May 24, 2017
 */

#pragma once

#include "cosmo/Subscriber.hpp"

namespace cosmo {

template<typename MessageType_, unsigned int Alignment_ = DefaultAlignment>
class PoolSubscriber : public Subscriber<MessageType_, Alignment_> {
public:
    using BaseType = Subscriber<MessageType_, Alignment_>;

    PoolSubscriber() = delete;
    PoolSubscriber(const std::string& topic, const bool printTiming=false):
        BaseType(topic, printTiming)
    {
    }

	~PoolSubscriber() override = default;

protected:

};


} // namespace 
