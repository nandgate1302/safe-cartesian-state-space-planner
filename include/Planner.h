#pragma once
#include "PlanningProblem.h"
#include "PlanningResult.h"

class Planner {
public:
    virtual ~Planner() = default;
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
};