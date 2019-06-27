//
// Created by pleemann on 25.05.17.
//

#pragma once

#include <cstdint>

class TestMessage{
public:
    TestMessage() {}

    int64_t  timestamp;
    double   position[3];
    double   orientation[4];
    int32_t  num_ranges;
    bool  enabled;
};
