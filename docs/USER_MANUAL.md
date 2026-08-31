# PCCST503 – Machine Learning | Assignment 1
# User Manual: Safe Cartesian State Space Planner

**Department of Computer Science and Engineering**  
**Course Code:** PCCST503 — Machine Learning  
**Assignment:** Design of a Safe Semantic Planner in a Finite Cartesian State Space  
**Author:** Nandana Sasikumar (TCR24CS050)  
**Implementation Language:** C++17  

---

## 1. Introduction

The **Safe Cartesian State Space Planner** is an incremental path planning system built in C++17 using the **Lifelong Planning A* (LPA\*)** algorithm. It computes optimal, collision-free paths for agents navigating a finite set of states embedded in a $d$-dimensional Cartesian space $\mathbb{R}^d$.

Key capabilities:
- **Zero Bad-State Guarantee:** Hard structural avoidance of hazardous obstacles.
- **Multi-Objective Optimization:** Balances distance cost, intrinsic edge safety, reliability, and obstacle clearance margins.
- **Dynamic Replanning:** Adapts to runtime edge failures, cost modifications, obstacle additions, and goal changes without rebuilding graph search trees.
- **Dual Interface:** Operates both as an interactive command-line application and as a reusable C++ library.

---

## 2. System Requirements & Installation

### 2.1 Prerequisites
- **Compiler:** Any modern C++17 compliant compiler (`g++ >= 7.0`, `clang++ >= 5.0`, or MSVC 2019+)
- **Build System:** CMake 3.10 or higher (optional, but recommended)
- **Platforms:** Linux, macOS, and Windows

### 2.2 Building with CMake (Recommended)

```bash
# Clone or navigate to the repository directory
cd safe-cartesian-state-space-planner

# Create and enter build directory
mkdir -p build && cd build

# Configure and compile all targets
cmake ..
make -j4
```

This compiles three executables:
1. `build/safe_planner`: Interactive CLI planner supporting file or stdin inputs.
2. `build/demo_test`: Regression test suite executing canonical Test Cases 1 through 6.
3. `build/benchmark`: Automated empirical benchmark harness measuring scaling, memory, and replan speedups.

### 2.3 Direct Compilation (Without CMake)

You can compile the binaries directly with `g++`:

```bash
# Build the interactive CLI planner
g++ -std=c++17 -O3 -Iinclude src/main.cpp src/ProblemLoader.cpp src/LPAStarPlanner.cpp -o safe_planner

# Build the regression test suite
g++ -std=c++17 -O3 -Iinclude src/demo_test.cpp src/LPAStarPlanner.cpp -o demo_test

# Build the benchmark suite
g++ -std=c++17 -O3 -Iinclude src/benchmark.cpp src/LPAStarPlanner.cpp -o benchmark
```

*(Note for Windows users: Add `-lpsapi` when compiling with MinGW).*

---

## 3. Input Graph File Format

The planner loads graphs using a clean, human-readable text format. Blank lines and comments (lines starting with `#`) are ignored.

### 3.1 File Structure Specification

```text
# 1. State Space Header: <number_of_states> <dimension_d>
<N> <d>

# 2. State Definitions: <state_id> <coord_1> <coord_2> ... <coord_d>
<id_1> <x_11> <x_12> ... <x_1d>
<id_2> <x_21> <x_22> ... <x_2d>
...

# 3. Bad State Count Header: <number_of_bad_states>
<K>

# 4. Bad State Identifiers (one per line)
<bad_id_1>
<bad_id_2>
...

# 5. Transitions Header: <number_of_transitions>
<M>

# 6. Transition Definitions: <id> <from> <to> <cost> <safety> <reliability> [available]
<t_1> <u_1> <v_1> <cost_1> <safety_1> <rel_1> [avail_1]
<t_2> <u_2> <v_2> <cost_2> <safety_2> <rel_2> [avail_2]
...

# 7. Query Header: <initial_state_id> <goal_state_id>
<initialState> <goalState>
```

### 3.2 Field Descriptions

| Field | Type | Description |
| :--- | :--- | :--- |
| `numStates` | Integer | Total number of states $N \ge 1$. |
| `dimension` | Integer | Dimension of Cartesian embedding space $d \ge 1$. |
| `state_id` | Integer (`uint64_t`) | Unique identifier for the state. |
| `coord_i` | Floating-point | $i$-th coordinate value in $\mathbb{R}^d$. |
| `badStates` | List of Integers | IDs of states that must never be visited. |
| `trans_id` | Integer (`uint64_t`) | Unique identifier for directed edge. |
| `from`, `to` | Integer (`uint64_t`) | Source and target state IDs. |
| `cost` | Floating-point | Traversal cost ($\ge 0.0$). |
| `safety` | Floating-point | Intrinsic edge safety score $\in [0.0, 1.0]$ ($1.0 = \text{safest}$). |
| `reliability` | Floating-point | Probability of transition success $\in (0.0, 1.0]$. |
| `available` | Integer ($0$ or $1$) | Optional flag: $1 = \text{active}$ (default), $0 = \text{disabled}$. |
| `initialState` | Integer | Starting state ID. |
| `goalState` | Integer | Target goal state ID. |

### 3.3 Sample Input File (`examples/sample_graph.txt`)

```text
# Sample graph: 6 states in 2D space (x, y)
6 2
1 0 0
2 1 1
3 2 1
4 4 0
5 1 -1
6 2 -1

# 1 bad state (pothole at state 3)
1
3

# 6 directed transitions: id from to cost safety reliability [available]
6
201 1 2 1.0 1.0 1.0 1
202 2 3 1.0 1.0 1.0 1
203 3 4 1.0 1.0 1.0 1
204 1 5 1.5 1.0 1.0 1
205 5 6 1.5 1.0 1.0 1
206 6 4 1.5 1.0 1.0 1

# Initial state (1) and Goal state (4)
1 4
```

---

## 4. Running the Interactive CLI Planner

### 4.1 Loading from a File

```bash
./build/safe_planner examples/sample_graph.txt
```

**Terminal Output:**
```text
Loaded 6 states, 6 transitions, 1 bad states.
[initial path] path: 1 -> 5 -> 6 -> 4 | cost=4.5 | minDistToBadState=2.00

Commands:
  path [goalId]           - print path to current goal, or to goalId
  goal <id>                - change the goal (instant, no replanning)
  avail <transId> <0|1>    - mark a transition unavailable/available
  cost <transId> <cost>    - update a transition's cost
  add <id> <from> <to> <cost> <safety> <reliability> [avail]
                            - add a new transition
  remove <transId>         - remove (disable) a transition
  badstate <id>            - mark a state bad (excludes its transitions)
  goodstate <id>           - clear a state's bad-state status
  stats                    - expansions, timing, memory for the last update
  help                     - show this list again
  quit                     - exit

> 
```

### 4.2 Interactive CLI Commands Reference

| Command Syntax | Parameters | Description |
| :--- | :--- | :--- |
| `path [goalId]` | Optional `goalId` | Prints optimal path from start to current goal (or specified `goalId`). |
| `goal <id>` | Target state `id` | Changes goal state instantly in $O(1)$ time with **zero** replanning. |
| `avail <id> <0\|1>` | `id`: transition ID, `0/1`: flag | Toggles transition availability and incrementally replans. |
| `cost <id> <newCost>` | `id`: transition ID, `newCost`: float | Updates edge cost and incrementally replans affected subpaths. |
| `add <id> <from> <to> <cost> <safety> <rel> [avail]` | Full edge tuple | Adds a new directed transition (e.g., shortcut) into the graph. |
| `remove <id>` | `id`: transition ID | Disables a transition and reroutes if necessary. |
| `badstate <id>` | `id`: state ID | Designates a state as bad; prunes incident edges and recalculates. |
| `goodstate <id>` | `id`: state ID | Clears bad status from a state, restoring valid connections. |
| `stats` | None | Displays expansions, replan time (ms), analytical memory, and RSS. |
| `help` | None | Displays the interactive command menu. |
| `quit` or `exit` | None | Terminates the application. |

---

## 5. C++ Developer API Reference

The planner can be embedded directly into robotics stacks or AI agents.

### 5.1 Minimal One-Shot Planning Example

```cpp
#include <iostream>
#include "LPAStarPlanner.h"

int main() {
    PlanningProblem problem;
    problem.initialState = 1;
    problem.goalState = 4;

    // 1. Define states in 2D Cartesian space
    problem.states = {
        State{1, {0.0, 0.0}},
        State{2, {1.0, 0.0}},
        State{3, {2.0, 0.0}},
        State{4, {3.0, 0.0}}
    };

    // 2. Define bad states (obstacles)
    problem.badStates = {};

    // 3. Define transitions
    problem.transitions = {
        Transition{101, 1, 2, 1.0, 1.0, 1.0, true},
        Transition{102, 2, 3, 1.0, 1.0, 1.0, true},
        Transition{103, 3, 4, 1.0, 1.0, 1.0, true}
    };

    // 4. Instantiate planner and solve
    LPAStarPlanner planner;
    PlanningResult result = planner.plan(problem);

    if (result.success) {
        std::cout << "Path found with total cost: " << result.totalCost << "\n";
        for (uint64_t s : result.statePath) {
            std::cout << s << " ";
        }
        std::cout << "\n";
    }

    return 0;
}
```

### 5.2 Customizing Multi-Objective Weights

```cpp
LPAStarPlanner::Weights customWeights;
customWeights.costWeight = 2.0;       // Weight on transition cost
customWeights.edgeSafetyWeight = 1.5; // Penalty for hazardous edges
customWeights.distSafetyWeight = 5.0; // Strong penalty for proximity to bad states
customWeights.distEpsilon = 1e-3;     // Numerical buffer

LPAStarPlanner customPlanner(customWeights);
PlanningResult result = customPlanner.plan(problem);
```

### 5.3 Incremental Dynamic Replanning API

```cpp
LPAStarPlanner planner;
planner.loadProblem(problem);

// Query initial path
PlanningResult path1 = planner.getPath(problem.goalState);

// Dynamic Event 1: Edge failure
planner.setTransitionAvailable(102, false);
PlanningResult pathAfterFailure = planner.getPath(problem.goalState);

// Dynamic Event 2: Add shortcut
planner.addTransition(Transition{104, 1, 4, 1.5, 1.0, 1.0, true});
PlanningResult pathWithShortcut = planner.getPath(problem.goalState);

// Dynamic Event 3: Instant goal retargeting
planner.setGoal(2);
PlanningResult pathToNewGoal = planner.getPath(2); // O(1) query time
```

---

## 6. Troubleshooting & Diagnostics

- **Error: "Failed to parse input: malformed state count line"**  
  *Cause:* The first non-comment line must contain `<numStates> <dimension>`. Check for missing spaces or invalid characters.
- **Error: "FAILED - no path found"**  
  *Cause:* All candidate paths to the goal either traverse a designated bad state or all available transitions are marked `available = 0`. Use the `badstate` or `avail` commands to check connectivity.
- **Memory Diagnostic Note:** On Linux systems, `stats` reports true Resident Set Size (RSS) via `/proc/self/status`. On Windows, it links against `psapi.lib`.
