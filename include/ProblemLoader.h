#pragma once
#include <istream>
#include <string>
#include "PlanningProblem.h"

// Parses a PlanningProblem out of a plain-text stream. See the format
// description in main.cpp's header comment (or README.md) for the exact
// layout expected. Returns true on success; on failure, returns false and
// fills `error` with a human-readable reason.
bool loadProblemFromStream(std::istream& in, PlanningProblem& outProblem, std::string& error);