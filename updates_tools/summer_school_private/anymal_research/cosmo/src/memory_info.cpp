/*
 * memory_cleaner.cpp
 *
 *  Created on: Sep, 2017
 *      Author: Philipp Leemann
 */

#include "cosmo/CommunicatorOptions.hpp"
#include "cosmo/memory_tools.hpp"

#include "message_logger/message_logger.hpp"

#include <iostream>
#include <cstring>

void printUsage() {
    std::cout << "Arguments: " << std::endl
              << "  poolName:  Name of the memory pool to inspect. Default is " << cosmo::defaultMemoryPoolName << std::endl;
}


int main(int argc, char **argv) {

    /// Parse arguments
    std::string memoryPoolName;
    if(argc > 2) {
        printUsage();
        return -1;
    }else if(argc == 2) {
        if(std::strcmp(argv[1], "--help") == 0) {
            printUsage();
            return 0;
        }

        memoryPoolName = argv[1];
    }else{
        memoryPoolName = cosmo::defaultMemoryPoolName;
    }

    cosmo::MemoryInfo info;
    if(!cosmo::getMemoryInfo(memoryPoolName, &info)) {
        MELO_ERROR("Failed to get full memory info, printout below may be inaccurate!");
    }

    std::cout << cosmo::memoryInfoToString(info);

    return 0;
}