/*
 * memory_cleaner.cpp
 *
 *  Created on: Sep, 2017
 *      Author: Philipp Leemann
 */

#include "cosmo/CommunicatorOptions.hpp"
#include "cosmo/memory_tools.hpp"

#include <iostream>
#include <cstring>

void printUsage() {
    std::cout << "Arguments: " << std::endl
              << "  poolName:  Name of the memory pool to remove. Default is " << cosmo::defaultMemoryPoolName << std::endl;
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

    if(cosmo::removeMemoryPool(memoryPoolName)) {
        std::cout << "Sucessfully removed shared memory pool '" << memoryPoolName << "'" << std::endl;
    }else{
        std::cout << "Failed to remove shared memory pool '" << memoryPoolName << "'!" << std::endl;
    }

    return 0;
}