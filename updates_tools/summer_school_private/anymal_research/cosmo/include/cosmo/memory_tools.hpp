/*!
 * @file	memory_tools.hpp
 * @author	Philipp Leemann
 * @date	Sep, 2017
 */

#pragma once

#include <string>
#include <vector>

namespace cosmo {

struct TopicInfo {
    std::string topicName;
    unsigned int numSubscribers;
    unsigned int numPublishers;
    unsigned long int messageCounter;
};

struct MemoryInfo {
    std::string poolName;
    unsigned int size;
    unsigned int freeMemory;
    bool allDeallocated;
    bool sane;
    unsigned int numObjects;
    unsigned int numUniques;
    unsigned int numCommunicators;
    unsigned int numTopics;
    std::vector<TopicInfo> topicInfos;
};

/**
 * @brief      Removes the memory pool
 *
 * @param[in]  poolName  The name of the memory pool
 *
 * @return     Returns true of the memory pool has been removed
 */
bool removeMemoryPool(const std::string& poolName);

/**
 * @brief      Retrieve information about the shared memory management
 *
 * @param[in]  poolName  The name of the memory pool
 * @param[out] info      A string with information
 *
 * @return     Returns true if the memory pool has been opened
 */
bool getMemoryInfo(const std::string& poolName, MemoryInfo* info);

std::string memoryInfoToString(const MemoryInfo& info);

} // namespace 
