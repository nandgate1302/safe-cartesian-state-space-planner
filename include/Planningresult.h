#pragma once
#include <cstdint>
#include <vector>

struct PlanningResult {
    bool success = false;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost = 0.0;     // sum of raw transition costs along the path
    double safetyScore = 0.0;   // minimum distance from any visited state to the nearest bad state
};