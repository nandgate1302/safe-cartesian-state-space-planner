# Safe Semantic Planner (LPA*)

C++17 implementation of PCCST503 Assignment 1: a safe path planner over a
finite Cartesian state space that avoids bad states, balances cost against
safety, and replans efficiently when the environment changes.

## Build

With CMake:
```
mkdir build && cd build
cmake ..
make
./safe_planner
```

Or directly with g++:
```
g++ -std=c++17 -Iinclude src/main.cpp src/LPAStarPlanner.cpp -o safe_planner
./safe_planner
```

## Layout

```
include/
  State.h              - a point in R^d
  Transition.h         - a directed, weighted edge
  PlanningProblem.h     - the input struct (as given in the assignment)
  PlanningResult.h      - the output struct (as given in the assignment)
  Planner.h             - abstract base class (as given in the assignment)
  LPAStarPlanner.h       - the actual planner
src/
  LPAStarPlanner.cpp     - LPA* implementation
  main.cpp               - runs all 6 illustrative test cases from the spec
```

## Algorithm

LPA* (Lifelong Planning A*), run to full convergence rather than stopping
early at the goal (see the comment at the top of `LPAStarPlanner.h` for why -
short version: it makes goal changes free and edge changes stay local).

Combined edge weight (what the search actually minimizes):

```
weight(u -> v) = costWeight   * (cost / max(reliability, floor))
               + edgeSafetyWeight * (1 - safety)
               + distSafetyWeight * (1 / (epsilon + distToNearestBadState(v)))
```

This operationalizes the assignment's `Score(P) = aG - bC + gD + dR` as a
single scalar edge cost that standard shortest-path search can minimize.
Bad states are excluded from the graph entirely at load time, so "never
visit a bad state" is a hard structural guarantee, not just a soft penalty.

## Complexity

- Space: O(V + E)
- Initial planning: O(E log V)
- Replanning after a local change: proportional to the number of states
  that actually become inconsistent (see `lastReplanExpansions()`), typically
  far below O(V) for changes away from the currently active path.