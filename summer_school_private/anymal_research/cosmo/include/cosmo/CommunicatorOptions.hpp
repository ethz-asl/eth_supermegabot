/*!
 * @file	CommunicatorOptions.hpp
 * @author	Philipp Leemann
 * @date	Jul 24, 2017
 */

#pragma once

#include <boost/interprocess/permissions.hpp>

#include <string>
#include <chrono>

namespace cosmo {

constexpr char defaultMemoryPoolName[] = "COSMO_SHM";

class CommunicatorOptions
{
public:

    CommunicatorOptions() = delete;

    /**
     * @brief      Creates communicator options
     *
     * @param[in]  topic  The mesage topic
     */
    CommunicatorOptions(const std::string topic):
            topic_(topic)
    {
    }

    virtual ~CommunicatorOptions() = default;

public:
    //! name of the topic to publish to / subscribe from
    std::string topic_;

    //! name of the shared memory block. Connections can not be made across blocks with different names.
    std::string memoryPoolName_{defaultMemoryPoolName};

    //! size of the shared memory pool, in bytes (max limit can be found by running 'cat /proc/sys/kernel/shmmax' in the console)
    unsigned int memoryPoolSize_{1048576}; // =2^20

    //! permissions for accessing the memory block. Unix style (e.g. 0664)
    boost::interprocess::permissions memoryPermissions_{0664};

    //! Timeout [us] for the locking of the topic mutex during initialization. If this timeout is exceeded, it is assumed that a process has
    //! crashed while having the topic mutex locked.
    std::chrono::microseconds missingProcessTimeout_{5000000};
};

} // namespace 
