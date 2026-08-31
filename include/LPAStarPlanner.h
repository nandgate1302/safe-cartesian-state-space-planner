#pragma once
#include <cstdint>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <utility>
#include "Planner.h"

class LPAStarPlanner : public Planner {
public:
    // Tunable weights mirroring the assignment's alpha/beta/gamma/delta
    // style objective Score(P) = aG - bC + gD + dR.
    struct Weights {
        double costWeight = 1.0;        // weight on (cost / reliability)
        double reliabilityFloor = 1e-6; // avoids divide-by-zero for reliability -> 0
        double edgeSafetyWeight = 1.0;  // penalizes low per-edge safety score
        double distSafetyWeight = 1.0;  // penalizes proximity to bad states
        double distEpsilon = 1e-3;      // avoids divide-by-zero when a state sits on a bad state's boundary
    };

    LPAStarPlanner();
    explicit LPAStarPlanner(Weights weights);

    // One-shot interface required by the assignment's Planner base class.
    PlanningResult plan(const PlanningProblem& problem) override;

    // ---------------- Incremental / dynamic-environment API ----------------
    void loadProblem(const PlanningProblem& problem);   // full (re)build, O(V + E)
    PlanningResult getPath(uint64_t goal) const;         // O(path length), no replanning
    void setGoal(uint64_t goal);                         // O(1), see class comment above

    void setTransitionAvailable(uint64_t transitionId, bool available);
    void updateTransitionCost(uint64_t transitionId, double newCost);
    void addTransition(const Transition& t);
    void removeTransition(uint64_t transitionId);

    void addBadState(uint64_t id);
    void removeBadState(uint64_t id);

    // Diagnostics for the experiments section of the report.
    std::size_t lastReplanExpansions() const { return lastExpansions_; }
    double lastPlanningTimeMs() const { return lastPlanningTimeMs_; }
    std::size_t approximateMemoryBytes() const;

private:
    using Key = std::pair<double, double>;
    struct QueueEntry {
        Key key;
        uint64_t id;
    };
    struct QueueEntryCompare {
        bool operator()(const QueueEntry& a, const QueueEntry& b) const {
            return a.key > b.key; // std::priority_queue is a max-heap; flip for min-heap behavior
        }
    };

    Weights w_;
    std::unordered_map<uint64_t, State> states_;
    std::unordered_map<uint64_t, Transition> transitions_;
    std::unordered_set<uint64_t> badStates_;
    std::unordered_map<uint64_t, std::vector<uint64_t>> outgoing_; // state -> transition ids
    std::unordered_map<uint64_t, std::vector<uint64_t>> incoming_;

    uint64_t start_ = 0;
    uint64_t goal_ = 0;

    std::unordered_map<uint64_t, double> g_;
    std::unordered_map<uint64_t, double> rhs_;
    mutable std::unordered_map<uint64_t, double> safetyDistCache_;

    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryCompare> pq_;
    std::unordered_map<uint64_t, Key> queuedKey_; // node -> its currently-valid key (lazy deletion)

    mutable std::size_t lastExpansions_ = 0;
    double lastPlanningTimeMs_ = 0.0; // wall-clock time of the most recent computeShortestPath() call

    void removeAdjacencyEntry(std::unordered_map<uint64_t, std::vector<uint64_t>>& adj,
                               uint64_t key, uint64_t transitionId);

    double g(uint64_t s) const;
    double rhsOf(uint64_t s) const;
    double euclidean(uint64_t a, uint64_t b) const;
    double distToNearestBadState(uint64_t s) const;
    double edgeWeight(const Transition& t) const;
    Key calculateKey(uint64_t s) const;
    void updateVertex(uint64_t u);
    void computeShortestPath();
    bool isBad(uint64_t s) const { return badStates_.count(s) > 0; }
};