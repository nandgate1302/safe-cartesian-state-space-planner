# PCCST503 – Machine Learning | Assignment 1
# Design Report: Safe Semantic Planner in a Finite Cartesian State Space

**Department of Computer Science and Engineering**  
**Course Code:** PCCST503 — Machine Learning  
**Assignment:** Design of a Safe Semantic Planner in a Finite Cartesian State Space  
**Author:** Nandana Sasikumar (TCR24CS050)  
**Implementation Language:** C++17  

---

## 1. Executive Summary & Problem Formulation

The objective of this work is the design, implementation, and empirical evaluation of a generic, high-performance path planning algorithm capable of navigating a finite, $d$-dimensional Cartesian state space under safety, cost, reliability, and dynamic environment constraints.

```
       Cartesian State Space R^d
┌──────────────────────────────────────────────┐
│  s_I (Start)                                 │
│    │                                         │
│    ▼                                         │
│  [State 1] ───► [State 2] ───► s_G (Goal)   │
│                      ▲                       │
│                      │                       │
│             [Bad State / Obstacle]           │
│             (Safety Buffer Radius)           │
└──────────────────────────────────────────────┘
```

### 1.1 Formal Problem Definition

Let the planning environment be defined as a tuple $\mathcal{M} = \langle \mathcal{S}, s_I, s_G, \mathcal{B}, \mathcal{T} \rangle$, where:
- $\mathcal{S} = \{s_1, s_2, \dots, s_n\}$ is a finite set of states embedded in Cartesian space $\mathbb{R}^d$. Each state $s_i$ possesses a coordinate vector $\mathbf{x}(s_i) = (x_1, x_2, \dots, x_d) \in \mathbb{R}^d$.
- $s_I \in \mathcal{S}$ is the initial/start state.
- $s_G \in \mathcal{S}$ is the designated target/goal state.
- $\mathcal{B} = \{b_1, b_2, \dots, b_k\} \subset \mathcal{S}$ is the set of unsafe/bad states (e.g., physical hazards, restricted zones).
- $\mathcal{T} = \{t_1, t_2, \dots, t_m\}$ is a set of directed transitions $t = (u, v)$ where $u, v \in \mathcal{S}$.

Each transition $t = (u, v) \in \mathcal{T}$ is characterized by attributes:
1. $\text{cost}(t) \in \mathbb{R}_{> 0}$: Traversal resource expenditure (distance, time, or energy).
2. $\text{safety}(t) \in [0, 1]$: Intrinsic edge safety rating ($1.0 = \text{safest}, 0.0 = \text{hazardous}$).
3. $\text{reliability}(t) \in (0, 1]$: Probability of successful execution without failure ($1.0 = \text{deterministic}$).
4. $\text{available}(t) \in \{0, 1\}$: Operational status flag ($1 = \text{active}, 0 = \text{disabled}$).

### 1.2 Multi-Objective Optimization Formulation

A valid planning trajectory $P = \langle s^{(0)}, s^{(1)}, \dots, s^{(L)} \rangle$ with $s^{(0)} = s_I$ and $s^{(L)} = s_G$ must satisfy:
1. **Goal Reachability:** $s^{(L)} = s_G$.
2. **Zero Bad-State Invariant:** $\forall s \in P, s \notin \mathcal{B}$.
3. **Cost Minimization:** Minimize cumulative traversal expenditure $C(P) = \sum_{i=0}^{L-1} \text{cost}(s^{(i)}, s^{(i+1)})$.
4. **Safety Margin Maximization:** Maximize minimum clearance distance to all obstacles $D(P) = \min_{s \in P} \min_{b \in \mathcal{B}} \|\mathbf{x}(s) - \mathbf{x}(b)\|_2$.
5. **Reliability Maximization:** Maximize path execution fidelity $R(P) = \prod_{i=0}^{L-1} \text{reliability}(s^{(i)}, s^{(i+1)})$.

The composite assignment objective function is given as:
$$\text{Score}(P) = \alpha G - \beta C(P) + \gamma D(P) + \delta R(P)$$
where $\alpha, \beta, \gamma, \delta \ge 0$ are weighting hyperparameters, and $G \in \{0, 1\}$ represents goal completion.

### 1.3 Operational Edge Scalarization

To efficiently optimize this multi-objective criterion within graph search algorithms, we map the objectives into a scalarized positive edge weight function $w(u \to v)$ for every transition $t = (u, v)$:

$$w(u \to v) = w_{\text{cost}} \cdot \left(\frac{\text{cost}(t)}{\max(\text{reliability}(t), \epsilon_R)}\right) + w_{\text{edge}} \cdot (1 - \text{safety}(t)) + w_{\text{dist}} \cdot \left(\frac{1}{\epsilon_D + \text{dist}(v, \mathcal{B})}\right)$$

Where:
- $\text{dist}(v, \mathcal{B}) = \min_{b \in \mathcal{B}} \|\mathbf{x}(v) - \mathbf{x}(b)\|_2$ is the Euclidean distance from state $v$ to the nearest bad state.
- $\epsilon_R = 10^{-6}$ prevents numerical instability for near-zero reliability.
- $\epsilon_D = 10^{-3}$ ensures finite barrier values when a state touches a bad state boundary.
- $w_{\text{cost}}, w_{\text{edge}}, w_{\text{dist}}$ correspond directly to $\beta, \delta, \gamma$.

Minimizing $\sum w(u \to v)$ directly optimizes the multi-objective utility $\text{Score}(P)$.

---

## 2. State Representation & Geometric Embeddings

### 2.1 State Data Model

Every state in the environment is modeled using the `State` structure:
```cpp
struct State {
    uint64_t id;
    std::vector<double> embedding; // Coordinates (x_1, x_2, ..., x_d)
};
```

States are identified by 64-bit unsigned integers (`uint64_t`) and positioned in an arbitrary $d$-dimensional Euclidean space $\mathbb{R}^d$.

### 2.2 Metric Computation & Distance Caching

The Euclidean distance between any two states $a$ and $b$ with embeddings $\mathbf{x}_a, \mathbf{x}_b \in \mathbb{R}^d$ is:
$$\text{dist}(a, b) = \|\mathbf{x}_a - \mathbf{x}_b\|_2 = \sqrt{\sum_{i=1}^d \left(x_{a,i} - x_{b,i}\right)^2}$$

Because finding the nearest bad state requires checking all $k = |\mathcal{B}|$ obstacles:
$$\text{dist}(s, \mathcal{B}) = \min_{b \in \mathcal{B}} \|\mathbf{x}_s - \mathbf{x}_b\|_2$$
an un-cached calculation would impose an $O(k \cdot d)$ computational burden on every edge evaluation.

To eliminate this bottleneck, the planner maintains `safetyDistCache_`: a memoization map `std::unordered_map<uint64_t, double>`. Distance values are computed lazily on first access and cached in $O(1)$ amortized time. Whenever bad states are inserted or removed, the cache is invalidated (`safetyDistCache_.clear()`), preserving consistency.

---

## 3. Data Structures & Graph Architecture

The planner maintains explicit graph representations and algorithmic bookkeeping structures:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           LPAStarPlanner                                │
├─────────────────────────────────────────────────────────────────────────┤
│  states_          : unordered_map<uint64_t, State>                      │
│  transitions_     : unordered_map<uint64_t, Transition>                 │
│  badStates_       : unordered_set<uint64_t>                             │
│  outgoing_        : unordered_map<uint64_t, vector<uint64_t>>           │
│  incoming_        : unordered_map<uint64_t, vector<uint64_t>>           │
│  g_, rhs_         : unordered_map<uint64_t, double>                     │
│  safetyDistCache_ : unordered_map<uint64_t, double>                     │
│  queuedKey_       : unordered_map<uint64_t, Key>                        │
│  pq_              : priority_queue<QueueEntry, vector, QueueEntryCompare>│
└─────────────────────────────────────────────────────────────────────────┘
```

### 3.1 Adjacency Indices
- **`outgoing_`**: Maps state ID $u$ to transition IDs $t = (u, v)$. Used during vertex expansion to propagate cost updates to successor nodes.
- **`incoming_`**: Maps state ID $v$ to transition IDs $t = (u, v)$. Crucial for computing the one-step lookahead cost $rhs(v) = \min_{u \in \text{Pred}(v)} (g(u) + w(u, v))$.

### 3.2 Dual Value Maps ($g$ and $rhs$)
- **`g_[s]`**: Current estimated shortest path cost from start $s_I$ to $s$.
- **`rhs_[s]`**: Right-Hand Side (one-step lookahead) value:
  $$rhs(s) = \begin{cases} 0 & \text{if } s = s_I \\ \min_{(u, s) \in \mathcal{T}, \text{avail}} (g(u) + w(u \to s)) & \text{otherwise} \end{cases}$$

### 3.3 Priority Queue with Lazy Deletion
- Standard `std::priority_queue` stores pairs of keys and node IDs.
- In LPA*, keys change when vertex costs change. Instead of expensive $O(N)$ queue re-heapifications, the planner uses **lazy deletion**:
  - `queuedKey_[u]` stores the currently valid key for node $u$.
  - When popping from `pq_`, entries with stale keys (`!(found->second == top.key)`) are instantly discarded in $O(1)$.

---

## 4. Heuristic Function & Search Convergence

### 4.1 Heuristic Formulation

The heuristic function $h(s)$ estimates the remaining distance from state $s$ to goal $s_G$:
$$h(s) = \|\mathbf{x}_s - \mathbf{x}_{s_G}\|_2 \cdot w_{\text{cost}}$$

### 4.2 Admissibility & Consistency Proof

1. **Admissibility:** In Cartesian space $\mathbb{R}^d$, the straight-line Euclidean distance $\|\mathbf{x}_s - \mathbf{x}_{s_G}\|_2$ is the physical lower bound on any connected path length. Since all transition costs are normalized $\ge 1.0$, $h(s) \le c^*(s, s_G)$ holds strictly. Hence, $h(s)$ is admissible.
2. **Consistency (Triangle Inequality):** By the triangle inequality of Euclidean space:
   $$\|\mathbf{x}_u - \mathbf{x}_{s_G}\|_2 \le \|\mathbf{x}_u - \mathbf{x}_v\|_2 + \|\mathbf{x}_v - \mathbf{x}_{s_G}\|_2 \le w(u \to v) + h(v)$$
   Therefore, $h(u) \le w(u \to v) + h(v)$, proving monotonicity/consistency.

### 4.3 Key Priority Formulation

Each node is prioritized in the open list using a lexicographical 2-tuple key $K(s) = [k_1(s), k_2(s)]$:
$$\begin{aligned}
k_1(s) &= \min(g(s), rhs(s)) + h(s) \\
k_2(s) &= \min(g(s), rhs(s))
\end{aligned}$$
Key comparison $K(a) < K(b)$ is evaluated as $(k_1(a) < k_1(b)) \lor (k_1(a) = k_1(b) \land k_2(a) < k_2(b))$.

### 4.4 Design Decision: Convergence vs. Early Stopping

Standard A* terminates the moment the goal node $s_G$ is expanded. However, this planner runs LPA* until the priority queue is empty (full convergence of all inconsistent nodes):
- **Why this design is superior for dynamic systems:** Full convergence ensures that $g(s)$ contains the exact, globally optimal distance from $s_I$ to **all** reachable vertices $s \in \mathcal{S}$.
- **Instantaneous Goal Switching:** When a user or mission supervisor changes the target goal from $s_{G1}$ to $s_{G2}$, the planner extracts the new path in $O(|P|)$ time with **0 expansions** and **0 ms** replanning delay (demonstrated in Test Case 5).
- **Localized Edge Perturbations:** Only states topologically downstream of modified edges become inconsistent, restricting replanning strictly to affected subgraphs.

---

## 5. Dual-Layer Safety Computation

Safety in Cartesian navigation cannot rely solely on soft cost penalties; physical collisions must be strictly impossible. This architecture employs a **dual-layer safety model**:

```
                  DUAL-LAYER SAFETY ARCHITECTURE
 ┌──────────────────────────────────────────────────────────────┐
 │ Layer 1: Hard Topological Exclusion (Zero Bad States)       │
 │   - Bad states pruned from graph adjacency lists             │
 │   - Any edge entering or leaving bad state is dropped        │
 ├──────────────────────────────────────────────────────────────┤
 │ Layer 2: Soft Continuous Barrier Potential (Safety Margin)   │
 │   - Edge weight augmented by 1 / (epsilon + dist(v, Bad))    │
 │   - Steering paths smoothly away from hazard boundaries      │
 └──────────────────────────────────────────────────────────────┘
```

### 5.1 Layer 1: Hard Topological Exclusion
At graph load time and during dynamic obstacle updates:
- Any state $b \in \mathcal{B}$ is banned from the search graph.
- All incident transitions $t = (u, v)$ where $u \in \mathcal{B} \lor v \in \mathcal{B}$ are omitted from `outgoing_` and `incoming_`.
- If an agent is already planning, $g(b)$ and $rhs(b)$ are set to $\infty$.

**Formal Guarantee:** The number of bad states visited on any computed path is guaranteed to be **identically zero** ($|P \cap \mathcal{B}| \equiv 0$).

### 5.2 Layer 2: Soft Barrier Clearance Potential
To satisfy Optimization Objective 4 ("Maximize minimum Euclidean distance to nearest bad state"), the edge scalarization incorporates an inverse-distance potential barrier:
$$\text{Barrier}(v) = \frac{w_{\text{dist}}}{\epsilon_D + \min_{b \in \mathcal{B}} \|\mathbf{x}_v - \mathbf{x}_b\|_2}$$

- States in close proximity to bad states incur high barrier penalties.
- The planner naturally routes trajectories along wide, high-clearance corridors unless a tight passage is the sole reachable route.

---

## 6. Incremental Replanning Mechanism (LPA*)

Lifelong Planning A* (LPA*) maintains consistency between $g(s)$ and $rhs(s)$ values across graph modifications without recomputing unchanged shortest-path subtrees.

### 6.1 Vertex Classification
For any state $u \in \mathcal{S}$:
1. **Locally Consistent:** $g(u) = rhs(u)$. The shortest path estimate is correct.
2. **Locally Overconsistent:** $g(u) > rhs(u)$. The path cost to $u$ decreased (e.g., edge cost reduced, shortcut added).
3. **Locally Underconsistent:** $g(u) < rhs(u)$. The path cost to $u$ increased (e.g., edge failed, obstacle appeared).

```
                      updateVertex(u)
                             │
                  Is u == Start State?
                    ┌────────┴────────┐
                  YES                 NO
                    │                  │
                rhs[u] = 0        rhs[u] = min (g[v] + w(v,u))
                    └────────┬────────┘
                             │
                     Is g[u] == rhs[u]?
                    ┌────────┴────────┐
                  YES                 NO
                    │                  │
            Remove from PQ      Insert/Update in PQ
          (Locally Consistent)  with calculateKey(u)
```

### 6.2 Vertex Update Routine (`updateVertex`)
Whenever transition $t = (u, v)$ changes:
```cpp
void LPAStarPlanner::updateVertex(uint64_t u) {
    if (u != start_) {
        double best = INF;
        if (incoming_.count(u)) {
            for (auto tid : incoming_[u]) {
                const auto& t = transitions_[tid];
                if (!t.available) continue;
                best = std::min(best, g(t.from) + edgeWeight(t));
            }
        }
        rhs_[u] = best;
    }
    queuedKey_.erase(u);
    if (g(u) != rhsOf(u)) {
        Key k = calculateKey(u);
        queuedKey_[u] = k;
        pq_.push({k, u});
    }
}
```

### 6.3 Dynamic Event Handling

| Dynamic Event | Handler Method | Algorithm Action |
| :--- | :--- | :--- |
| **Transition Disabled** | `setTransitionAvailable(id, false)` | Marks `available=false`, calls `updateVertex(to)`, runs `computeShortestPath()`. Only affected downstream branch is recomputed. |
| **Transition Cost Altered** | `updateTransitionCost(id, cost)` | Updates cost, updates target vertex $rhs$, ripples local changes. |
| **New Transition Added** | `addTransition(t)` | Adds to adjacency lists, checks if $g(\text{from}) + w(t) < rhs(\text{to})$. If so, enqueues target. |
| **Transition Removed** | `removeTransition(id)` | Soft-disables edge, updates target vertex. |
| **New Bad State Inserted** | `addBadState(id)` | Prunes incident edges, clears distance cache, sets $g(id)=\infty$, updates affected neighbors. |
| **Bad State Cleared** | `removeBadState(id)` | Restores incident edges, clears distance cache, triggers vertex updates. |
| **Goal Updated** | `setGoal(newGoal)` | $O(1)$ state assignment. Path extracted immediately from existing $g$-tree with zero replanning. |

---

## 7. Asymptotic Complexity Analysis

### 7.1 Time Complexity

Let $|V| = |\mathcal{S}|$ denote total states, $|E| = |\mathcal{T}|$ total transitions, and $|\mathcal{B}|$ bad states in $\mathbb{R}^d$.

1. **Initial Graph Construction & Search:**
   - Adjacency and hash index building: $O(|V| + |E|)$.
   - Priority Queue Operations: In the worst case, every vertex is inserted and popped at most twice. Each queue operation costs $O(\log |V|)$.
   - Relaxation across edges: $O(|E| \log |V|)$.
   - **Total Initial Planning:** $\mathcal{O}(|E| \log |V|)$.
2. **Incremental Dynamic Replanning:**
   - When an edge or bad state changes, only the set of affected inconsistent vertices $\Delta V_{\text{incons}} \subseteq V$ and their adjacent edges $\Delta E_{\text{incons}} \subseteq E$ are processed.
   - **Total Dynamic Replanning:** $\mathcal{O}(|\Delta E_{\text{incons}}| \log |\Delta V_{\text{incons}}|)$.
   - For localized perturbations, $|\Delta V_{\text{incons}}| \ll |V|$, yielding orders-of-magnitude faster responses than full recomputation.
3. **Goal Re-Targeting:**
   - Since $g$-values are fully converged from $s_I$, updating $s_G$ requires zero queue expansions.
   - **Total Goal Update Time:** $\mathcal{O}(1)$.
4. **Path Extraction:**
   - Tracing back from $s_G$ to $s_I$ via incoming edges: $\mathcal{O}(|P| \cdot \text{deg}_{\text{in}})$, where $|P|$ is path length (hops).
5. **Safety Distance Calculation:**
   - First computation: $O(|\mathcal{B}| \cdot d)$.
   - Subsequent queries (cached): $\mathcal{O}(1)$ amortized.

### 7.2 Space Complexity

The planner stores:
- Vertex attributes and $d$-dimensional coordinate embeddings: $O(|V| \cdot d)$
- Directed adjacency lists (`outgoing_` and `incoming_`): $O(|V| + |E|)$
- Cost tables ($g, rhs, \text{queuedKey}$): $O(|V|)$
- Priority Queue: $O(|V|)$
- Safety Distance Cache: $O(|V|)$

$$\text{Total Space Complexity} = \mathcal{O}(|V| \cdot d + |E|)$$

The memory footprint is strictly linear in the graph size and embedding dimension.

---

## 8. Summary & Architectural Highlights

1. **Standard Interface Compliance:** Strictly implements the assignment specification (`State`, `Transition`, `PlanningProblem`, `PlanningResult`, `Planner`).
2. **Deterministic Safety:** Combines hard topological exclusion ($0$ bad states visited) with soft inverse-distance barrier guidance.
3. **High Replan Efficiency:** LPA* architecture reduces dynamic edge updates from full $O(|E| \log |V|)$ graph re-solves to local $O(|\Delta E| \log |\Delta V|)$ adjustments, while providing $O(1)$ goal switching.
4. **Cross-Platform Compatibility:** Features built-in memory diagnostics supporting both Linux (`/proc/self/status`) and Windows (`GetProcessMemoryInfo`).
