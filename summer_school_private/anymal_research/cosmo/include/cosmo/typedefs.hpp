/*!
 * @file	typedefs.hpp
 * @author	Philipp Leemann
 * @date	Mar, 2018
 */

#pragma once

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/allocators/allocator.hpp>


namespace cosmo {

constexpr unsigned int DefaultAlignment = 16;

using MemoryManager = boost::interprocess::managed_shared_memory;
using SegmentManager = MemoryManager::segment_manager;

using CharAllocator = boost::interprocess::allocator<char, SegmentManager>;
using ShmString = boost::interprocess::basic_string<char, std::char_traits<char>, CharAllocator>;

using StringAllocator = boost::interprocess::allocator<ShmString, SegmentManager>;
using ShmStringVector = boost::interprocess::vector<ShmString, StringAllocator>;


struct NodeInfo {
    NodeInfo(const int pid, const unsigned int startTime, const bool isPublisher)
            : pid_(pid),
              startTime_(startTime),
              isPublisher_(isPublisher)
    {
    }

    int pid_{0};
    unsigned int startTime_{0};
    bool isPublisher_{false};
};
using NodeInfoAllocator = boost::interprocess::allocator<NodeInfo, SegmentManager>;
using ShmNodeInfoVector = boost::interprocess::vector<NodeInfo, NodeInfoAllocator>;

} // end namespace