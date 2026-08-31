# Safe Cartesian State Space Planner (LPA*)

**PCCST503 – Machine Learning | Assignment 1**  
**Department of Computer Science and Engineering**  
**Author:** Nandana Sasikumar (TCR24CS050)  
**Language:** C++17  

A high-performance C++17 implementation of a safe, multi-objective path planning algorithm in a finite $d$-dimensional Cartesian state space $\mathbb{R}^d$ using **Lifelong Planning A* (LPA\*)**. The planner enforces a hard zero bad-state invariant, balances path cost against obstacle proximity, and performs instant incremental replanning when graph conditions change dynamically.

---

## 📑 Assignment Deliverables

All required assignment deliverables are thoroughly documented and available in the [`docs/`](docs/) directory:

1. **C++ Source Code:** Implementation adhering strictly to the assignment interfaces (`include/` and `src/`).
2. **[Design Report](docs/DESIGN_REPORT.md):** Theoretical formulation, state embeddings, heuristic admissibility proof, dual-layer safety model, LPA* mechanics, and asymptotic complexity analysis ($O(E \log V)$ time, $O(V \cdot d + E)$ space).
3. **[Experimental Results](docs/EXPERIMENTAL_RESULTS.md):** Empirical evaluation measuring goal success rate, zero bad-state violations, path costs, safety margins, vertex expansions, initial planning vs. replanning runtimes, and memory footprint across canonical and synthetic scalability benchmarks.
4. **[User Manual](docs/USER_MANUAL.md):** System requirements, build guide (CMake & direct g++), input graph grammar/file format, interactive CLI command guide, and C++ programmatic API documentation.
5. **[Demonstration](docs/DEMONSTRATION.md):** Step-by-step walkthrough of all 6 canonical test cases from the specification with visual graph topology diagrams, expected vs. observed outputs, and an annotated CLI demonstration transcript.

---

## 🚀 Quick Start

### Build with CMake (Recommended)

```bash
mkdir -p build && cd build
cmake ..
make -j4
```

This compiles three executables:
- `./build/safe_planner`: Interactive CLI planner supporting file or stdin inputs.
- `./build/demo_test`: Regression test suite executing canonical Test Cases 1 through 6.
- `./build/benchmark`: Automated empirical benchmark suite measuring scaling, memory, and replanning speedups.

### Direct Compilation with g++

```bash
# Build the interactive CLI planner
g++ -std=c++17 -O3 -Iinclude src/main.cpp src/ProblemLoader.cpp src/LPAStarPlanner.cpp -o safe_planner

# Build the regression test suite
g++ -std=c++17 -O3 -Iinclude src/demo_test.cpp src/LPAStarPlanner.cpp -o demo_test

# Build the benchmark suite
g++ -std=c++17 -O3 -Iinclude src/benchmark.cpp src/LPAStarPlanner.cpp -o benchmark
```

---

## 💻 Usage

### 1. Interactive CLI Mode

```bash
./build/safe_planner examples/sample_graph.txt
```

Once loaded, use interactive commands to trigger dynamic events:
```text
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
help                     - show command list
quit                     - exit
```

### 2. Regression Test Suite

```bash
./build/demo_test
```

Executes and verifies all 6 canonical scenarios defined in the assignment specification.

### 3. Empirical Benchmarks

```bash
./build/benchmark
```

Runs comprehensive benchmarks on canonical test cases, scaling grid topologies ($N=16$ to $N=2500$ states), dynamic edge disruptions, and safety parameter sweeps.

---

## 📂 Project Structure

```text
.
├── CMakeLists.txt              # CMake build configuration
├── README.md                   # Project overview and quickstart
├── docs/                       # Assignment Deliverables
│   ├── DESIGN_REPORT.md        # Comprehensive Design Report
│   ├── EXPERIMENTAL_RESULTS.md # Quantitative Evaluation & Scaling
│   ├── USER_MANUAL.md          # Build & CLI/API Reference
│   └── DEMONSTRATION.md        # Test Cases Walkthrough & Transcript
├── examples/
│   └── sample_graph.txt        # Example input graph in 2D Cartesian space
├── include/
│   ├── LPAStarPlanner.h        # Lifelong Planning A* implementation header
│   ├── MemoryUtil.h            # Platform-agnostic resident memory (RSS) diagnostics
│   ├── Planner.h               # Abstract planner interface base class
│   ├── PlanningProblem.h       # Problem definition struct
│   ├── PlanningResult.h        # Output result struct
│   ├── ProblemLoader.h         # File and stream parsing utility
│   ├── State.h                 # State representation with R^d embedding vector
│   └── Transition.h            # Directed edge with cost, safety, and reliability
└── src/
    ├── LPAStarPlanner.cpp      # LPA* algorithm and dynamic graph logic
    ├── MemoryUtil.h            # Header utility
    ├── ProblemLoader.cpp       # Stream parser implementation
    ├── benchmark.cpp           # Automated empirical evaluation harness
    ├── demo_test.cpp           # Regression test executable (TC1 - TC6)
    └── main.cpp                # Interactive CLI application
```

---

## ⚙️ Algorithmic Formulation

### Operational Objective Function
The assignment specifies the multi-objective utility:
$$\text{Score}(P) = \alpha G - \beta C + \gamma D + \delta R$$

This is operationalized into a scalarized edge cost minimized by LPA*:
$$w(u \to v) = w_{\text{cost}} \cdot \left(\frac{\text{cost}(t)}{\max(\text{reliability}(t), \epsilon_R)}\right) + w_{\text{edge\_safety}} \cdot (1 - \text{safety}(t)) + w_{\text{dist\_safety}} \cdot \left(\frac{1}{\epsilon_D + \text{dist}(v, \mathcal{B})}\right)$$

### Dual-Layer Safety Guarantee
1. **Hard Topological Exclusion:** States in $\mathcal{B}$ and their incident transitions are strictly omitted from the searchable graph, ensuring that the number of bad states visited is identically **zero**.
2. **Soft Continuous Barrier:** The inverse Euclidean distance potential $\frac{1}{\epsilon_D + \text{dist}(v, \mathcal{B})}$ penalizes close proximity to obstacles, steering trajectories along wide, high-clearance corridors.