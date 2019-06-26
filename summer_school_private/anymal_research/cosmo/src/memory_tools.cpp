/*!
 * @file	memory_tools.cpp
 * @author	Philipp Leemann
 * @date	Sep, 2017
 */

#include "cosmo/memory_tools.hpp"
#include "cosmo/typedefs.hpp"

#include "message_logger/message_logger.hpp"

#include <boost/interprocess/sync/interprocess_sharable_mutex.hpp>
#include <boost/interprocess/sync/sharable_lock.hpp>
#include <boost/thread/thread_time.hpp> // for boost::get_system_time

#include <sstream>
#include <string>
#include <vector>

namespace cosmo {

template <class T>
T* find_check(boost::interprocess::managed_shared_memory& managedShm, const std::string& name) {
    auto tmpPair = managedShm.find<T>(name.c_str());
    if(tmpPair.second == 0 || tmpPair.first == 0) {
        MELO_FATAL_STREAM("Unable to find variable " << name << " in shared memory. Shared memory seems corrupted, you should clean/remove it!");
    }
    return tmpPair.first;
}

bool removeMemoryPool(const std::string& poolName) {
    return boost::interprocess::shared_memory_object::remove(poolName.c_str());
}

bool getMemoryInfo(const std::string& poolName, MemoryInfo* info) {

    boost::interprocess::managed_shared_memory managedShm;
    try {
        managedShm = {boost::interprocess::open_only, poolName.c_str()};
    }catch(...) {
        MELO_ERROR_STREAM("No shared memory pool named '" << poolName << "' found!");
        return false;
    }

    try{
        info->poolName = poolName;
        info->size = managedShm.get_size();
        info->freeMemory = managedShm.get_free_memory();
        info->allDeallocated = managedShm.all_memory_deallocated();
        info->sane = managedShm.check_sanity();
        info->numObjects = managedShm.get_num_named_objects();
        info->numUniques = managedShm.get_num_unique_objects();

        {
            auto poolMutex = find_check<boost::interprocess::interprocess_sharable_mutex>(managedShm, "__PoolMutex");
            const boost::posix_time::ptime abs_time = boost::get_system_time() + boost::posix_time::microseconds(2000000);
            boost::interprocess::sharable_lock<boost::interprocess::interprocess_sharable_mutex> poolLock(*(poolMutex), abs_time);

            if(poolLock.owns()) {
                const auto numNodes = find_check<unsigned int>(managedShm, "__NumNodes");
                const auto topicList = find_check<ShmStringVector>(managedShm, "__TopicList");

                info->numCommunicators = *(numNodes);
                info->numTopics = topicList->size();

                for (const auto &topic : *(topicList)) {

                    const NodeInfoAllocator alloc_inst(managedShm.get_segment_manager());
                    auto smMutex = find_check<boost::interprocess::interprocess_sharable_mutex>(managedShm, (topic + "__Mutex").c_str());
                    const auto smNumPublishers = find_check<unsigned int>(managedShm, (topic + "__NumSubscribers").c_str());
                    const auto smNumSubscribers = find_check<unsigned int>(managedShm, (topic + "__NumPublishers").c_str());
                    const auto smCounter = find_check<unsigned long int>(managedShm, (topic + "__Counter").c_str());
//            const auto smNodeInfos = managedShm.find<Communicator::ShmNodeInfoVector>((topic + "__NodeInfos").c_str());

                    boost::interprocess::sharable_lock<boost::interprocess::interprocess_sharable_mutex> lock(*(smMutex));

                    TopicInfo topicInfo;
                    topicInfo.topicName = topic.c_str();
                    topicInfo.numSubscribers = *(smNumSubscribers);
                    topicInfo.numPublishers = *(smNumPublishers);
                    topicInfo.messageCounter = *(smCounter);
                    info->topicInfos.push_back(std::move(topicInfo));
                }
            }else{
                MELO_ERROR("Unable to lock pool mutex! Shared memory seems corrupted, you should clean/remove it!");
                return false;
            }

        }
    }catch(...) {
        return false;
    }

    return true;
}

std::string memoryInfoToString(const MemoryInfo& info) {

    std::ostringstream outStream;
    outStream   << "Shared memory '" << info.poolName << "' details:\n"
                << "  Size: " << info.size << "\n"
                << "  Free memory: " << info.freeMemory << "\n"
                << "  All deallocated: " << (info.allDeallocated ? "True" : "False") << "\n"
                << "  Sanity check result: " << (info.sane ? "True" : "False") << "\n"
                << "  Number of named objects: " << info.numObjects << "\n"
                << "  Number of unique objects: " << info.numUniques << "\n\n"
                << "Memory content:\n" << "  Number of communicators: " << info.numCommunicators << "\n"
                << "  Number of topics: " << info.numTopics << "\n\n";

            for (const auto &topicInfo : info.topicInfos) {
                outStream << "  Topic " << topicInfo.topicName << "\n" << "    Number of Subscribers: " << topicInfo.numSubscribers << "\n"
                          << "    Number of Publishers: " << topicInfo.numPublishers << "\n" << "    Message counter: " << topicInfo.messageCounter
                          << "\n";
            }
    return outStream.str();
}
} // namespace 
