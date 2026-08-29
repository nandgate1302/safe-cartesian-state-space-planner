#pragma once
#include <cstdint>

// A directed edge between two states.
struct Transition {
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;       // per-edge safety score in [0, 1], 1 = safest
    double reliability;  // in (0, 1], 1 = fully reliable
    bool available;
};