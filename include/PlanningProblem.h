#pragma once
#include <cstdint>
#include <vector>
#include "State.h"
#include "Transition.h"

struct PlanningProblem {
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;
};