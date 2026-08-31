#include "LPAStarPlanner.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <chrono>

static constexpr double INF = std::numeric_limits<double>::infinity();

LPAStarPlanner::LPAStarPlanner() : w_(Weights()) {}
LPAStarPlanner::LPAStarPlanner(Weights weights) : w_(weights) {}

void LPAStarPlanner::loadProblem(const PlanningProblem& problem) {
    states_.clear();
    transitions_.clear();
    badStates_.clear();
    outgoing_.clear();
    incoming_.clear();
    g_.clear();
    rhs_.clear();
    safetyDistCache_.clear();
    while (!pq_.empty()) pq_.pop();
    queuedKey_.clear();

    for (const auto& s : problem.states) states_[s.id] = s;
    for (auto b : problem.badStates) badStates_.insert(b);
    start_ = problem.initialState;
    goal_ = problem.goalState;

    for (const auto& t : problem.transitions) {
        transitions_[t.id] = t;
        if (isBad(t.from) || isBad(t.to)) continue; // a bad state is never part of the graph we search
        outgoing_[t.from].push_back(t.id);
        incoming_[t.to].push_back(t.id);
    }

    for (const auto& kv : states_) {
        g_[kv.first] = INF;
        rhs_[kv.first] = INF;
    }
    rhs_[start_] = 0.0;
    Key k0 = calculateKey(start_);
    queuedKey_[start_] = k0;
    pq_.push({k0, start_});

    computeShortestPath();
}

PlanningResult LPAStarPlanner::plan(const PlanningProblem& problem) {
    loadProblem(problem);
    return getPath(problem.goalState);
}

double LPAStarPlanner::g(uint64_t s) const {
    auto it = g_.find(s);
    return it == g_.end() ? INF : it->second;
}

double LPAStarPlanner::rhsOf(uint64_t s) const {
    auto it = rhs_.find(s);
    return it == rhs_.end() ? INF : it->second;
}

double LPAStarPlanner::euclidean(uint64_t a, uint64_t b) const {
    const auto& ea = states_.at(a).embedding;
    const auto& eb = states_.at(b).embedding;
    double sum = 0.0;
    std::size_t n = std::min(ea.size(), eb.size());
    for (std::size_t i = 0; i < n; ++i) {
        double d = ea[i] - eb[i];
        sum += d * d;
    }
    return std::sqrt(sum);
}

double LPAStarPlanner::distToNearestBadState(uint64_t s) const {
    auto cached = safetyDistCache_.find(s);
    if (cached != safetyDistCache_.end()) return cached->second;
    double best = INF;
    for (auto b : badStates_) best = std::min(best, euclidean(s, b));
    if (badStates_.empty()) best = 1e6; // no bad states => treat as maximally safe
    safetyDistCache_[s] = best;
    return best;
}

double LPAStarPlanner::edgeWeight(const Transition& t) const {
    double reliab = std::max(t.reliability, w_.reliabilityFloor);
    double safety = std::min(1.0, std::max(0.0, t.safety));
    double dist = distToNearestBadState(t.to);
    return w_.costWeight * (t.cost / reliab)
         + w_.edgeSafetyWeight * (1.0 - safety)
         + w_.distSafetyWeight * (1.0 / (w_.distEpsilon + dist));
}

LPAStarPlanner::Key LPAStarPlanner::calculateKey(uint64_t s) const {
    double m = std::min(g(s), rhsOf(s));
    double h = euclidean(s, goal_) * w_.costWeight; // ordering heuristic only, see class comment
    return {m + h, m};
}

void LPAStarPlanner::updateVertex(uint64_t u) {
    if (u != start_) {
        double best = INF;
        auto it = incoming_.find(u);
        if (it != incoming_.end()) {
            for (auto tid : it->second) {
                const auto& t = transitions_.at(tid);
                if (!t.available) continue;
                double cand = g(t.from) + edgeWeight(t);
                best = std::min(best, cand);
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

void LPAStarPlanner::computeShortestPath() {
    auto startClock = std::chrono::steady_clock::now();
    lastExpansions_ = 0;
    while (!pq_.empty()) {
        QueueEntry top = pq_.top();
        auto found = queuedKey_.find(top.id);
        if (found == queuedKey_.end() || !(found->second == top.key)) {
            pq_.pop(); // stale, lazily-deleted entry
            continue;
        }
        pq_.pop();
        queuedKey_.erase(top.id);
        ++lastExpansions_;

        if (g(top.id) > rhsOf(top.id)) {
            g_[top.id] = rhsOf(top.id);
            auto it = outgoing_.find(top.id);
            if (it != outgoing_.end())
                for (auto tid : it->second) updateVertex(transitions_.at(tid).to);
        } else {
            g_[top.id] = INF;
            updateVertex(top.id);
            auto it = outgoing_.find(top.id);
            if (it != outgoing_.end())
                for (auto tid : it->second) updateVertex(transitions_.at(tid).to);
        }
    }
    auto endClock = std::chrono::steady_clock::now();
    lastPlanningTimeMs_ = std::chrono::duration<double, std::milli>(endClock - startClock).count();
}

PlanningResult LPAStarPlanner::getPath(uint64_t goal) const {
    PlanningResult result;
    if (!states_.count(goal) || g(goal) == INF) {
        result.success = false;
        return result;
    }

    std::vector<uint64_t> statePath, transitionPath;
    uint64_t cur = goal;
    statePath.push_back(cur);
    double totalRawCost = 0.0;
    double minDist = INF;

    while (cur != start_) {
        auto it = incoming_.find(cur);
        if (it == incoming_.end()) { result.success = false; return result; }

        uint64_t bestT = 0;
        double bestVal = INF;
        bool found = false;
        for (auto tid : it->second) {
            const auto& t = transitions_.at(tid);
            if (!t.available) continue;
            double cand = g(t.from) + edgeWeight(t);
            if (cand < bestVal) { bestVal = cand; bestT = tid; found = true; }
        }
        if (!found) { result.success = false; return result; }

        const auto& t = transitions_.at(bestT);
        transitionPath.push_back(bestT);
        totalRawCost += t.cost;
        minDist = std::min(minDist, distToNearestBadState(t.to));
        cur = t.from;
        statePath.push_back(cur);
    }
    minDist = std::min(minDist, distToNearestBadState(start_));

    std::reverse(statePath.begin(), statePath.end());
    std::reverse(transitionPath.begin(), transitionPath.end());

    result.success = true;
    result.statePath = statePath;
    result.transitionPath = transitionPath;
    result.totalCost = totalRawCost;
    result.safetyScore = (minDist == INF) ? 0.0 : minDist;
    return result;
}

void LPAStarPlanner::setGoal(uint64_t goal) {
    goal_ = goal; // g_ already holds true shortest distance from start to every reachable state
}

void LPAStarPlanner::setTransitionAvailable(uint64_t transitionId, bool available) {
    auto it = transitions_.find(transitionId);
    if (it == transitions_.end()) return;
    it->second.available = available;
    updateVertex(it->second.to);
    computeShortestPath();
}

void LPAStarPlanner::updateTransitionCost(uint64_t transitionId, double newCost) {
    auto it = transitions_.find(transitionId);
    if (it == transitions_.end()) return;
    it->second.cost = newCost;
    updateVertex(it->second.to);
    computeShortestPath();
}

void LPAStarPlanner::addTransition(const Transition& t) {
    transitions_[t.id] = t;
    if (isBad(t.from) || isBad(t.to) || !states_.count(t.to) || !states_.count(t.from)) return;
    outgoing_[t.from].push_back(t.id);
    incoming_[t.to].push_back(t.id);
    updateVertex(t.to);
    computeShortestPath();
}

void LPAStarPlanner::removeTransition(uint64_t transitionId) {
    auto it = transitions_.find(transitionId);
    if (it == transitions_.end()) return;
    uint64_t to = it->second.to;
    it->second.available = false; // soft removal keeps adjacency bookkeeping simple
    updateVertex(to);
    computeShortestPath();
}

void LPAStarPlanner::removeAdjacencyEntry(std::unordered_map<uint64_t, std::vector<uint64_t>>& adj,
                                           uint64_t key, uint64_t transitionId) {
    auto it = adj.find(key);
    if (it == adj.end()) return;
    auto& vec = it->second;
    vec.erase(std::remove(vec.begin(), vec.end(), transitionId), vec.end());
}

void LPAStarPlanner::addBadState(uint64_t id) {
    if (!states_.count(id) || badStates_.count(id)) return; // unknown state, or already bad
    badStates_.insert(id);

    for (const auto& kv : transitions_) {
        const Transition& t = kv.second;
        if (t.from != id && t.to != id) continue;
        removeAdjacencyEntry(outgoing_, t.from, t.id);
        removeAdjacencyEntry(incoming_, t.to, t.id);
    }
    g_[id] = INF;
    rhs_[id] = INF;

    safetyDistCache_.clear();
    for (const auto& kv : states_) updateVertex(kv.first);
    computeShortestPath();
}

void LPAStarPlanner::removeBadState(uint64_t id) {
    if (!badStates_.count(id)) return; // wasn't bad, nothing to do
    badStates_.erase(id);

    // Restore any transition touching this state, as long as its other
    // endpoint isn't ALSO a bad state (in which case it must stay excluded).
    for (const auto& kv : transitions_) {
        const Transition& t = kv.second;
        if (t.from != id && t.to != id) continue;
        if (isBad(t.from) || isBad(t.to)) continue;
        outgoing_[t.from].push_back(t.id);
        incoming_[t.to].push_back(t.id);
    }

    safetyDistCache_.clear();
    for (const auto& kv : states_) updateVertex(kv.first);
    computeShortestPath();
}

std::size_t LPAStarPlanner::approximateMemoryBytes() const {
    // Analytical estimate from container sizes - a portable stand-in for
    // true OS-level RSS, which is platform-specific (see MemoryUtil.h for
    // an actual OS measurement, used alongside this in main.cpp).
    std::size_t bytes = 0;
    bytes += states_.size() * (sizeof(uint64_t) + sizeof(State));
    for (const auto& kv : states_) bytes += kv.second.embedding.size() * sizeof(double);
    bytes += transitions_.size() * (sizeof(uint64_t) + sizeof(Transition));
    bytes += badStates_.size() * sizeof(uint64_t);
    for (const auto& kv : outgoing_) bytes += sizeof(uint64_t) + kv.second.size() * sizeof(uint64_t);
    for (const auto& kv : incoming_) bytes += sizeof(uint64_t) + kv.second.size() * sizeof(uint64_t);
    bytes += g_.size() * (sizeof(uint64_t) + sizeof(double));
    bytes += rhs_.size() * (sizeof(uint64_t) + sizeof(double));
    bytes += safetyDistCache_.size() * (sizeof(uint64_t) + sizeof(double));
    bytes += pq_.size() * sizeof(QueueEntry);
    bytes += queuedKey_.size() * (sizeof(uint64_t) + sizeof(Key));
    return bytes;
}