/*!
 * @file	Publisher.hpp
 * @author	Philipp Leemann, Christian Gehring
 * @date	May 24, 2017
 */

#pragma once

#include "cosmo/CommunicatorOptions.hpp"

#include <memory>

namespace cosmo {

class PublisherOptions : public CommunicatorOptions
{
public:
    PublisherOptions(const std::string topic):
            CommunicatorOptions(topic)
    {
    }

    ~PublisherOptions() override = default;

public:
};

using PublisherOptionsPtr = std::shared_ptr<PublisherOptions>;

} // namespace 
