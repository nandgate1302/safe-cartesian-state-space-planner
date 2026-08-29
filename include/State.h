#pragma once
#include <cstdint>
#include <vector>

// A point in the d-dimensional Cartesian state space.
struct State {
    uint64_t id;
    std::vector<double> embedding; // (x1, x2, ..., xd)
};