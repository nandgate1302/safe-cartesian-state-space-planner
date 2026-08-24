#include <iostream>
#include <iomanip>
#include "LPAStarPlanner.h"

static void printResult(const std::string& label, const PlanningResult& r) {
    std::cout << "[" << label << "] ";
    if (!r.success) {
        std::cout << "FAILED - no path found\n";
        return;
    }
    std::cout << "path: ";
    for (std::size_t i = 0; i < r.statePath.size(); ++i) {
        std::cout << r.statePath[i];
        if (i + 1 < r.statePath.size()) std::cout << " -> ";
    }
    std::cout << " | cost=" << r.totalCost
              << " | minDistToBadState=" << std::fixed << std::setprecision(2) << r.safetyScore
              << "\n";
}

static State st(uint64_t id, double x, double y) { return State{id, {x, y}}; }
static Transition tr(uint64_t id, uint64_t from, uint64_t to, double cost,
                      double safety = 1.0, double reliability = 1.0, bool available = true) {
    return Transition{id, from, to, cost, safety, reliability, available};
}

// Test Case 1: Basic reachability, S(1) -> A(2) -> B(3) -> G(4)
static void testCase1() {
    PlanningProblem p;
    p.initialState = 1; p.goalState = 4;
    p.states = {st(1,0,0), st(2,1,0), st(3,2,0), st(4,3,0)};
    p.transitions = {tr(101,1,2,1.0), tr(102,2,3,1.0), tr(103,3,4,1.0)};

    LPAStarPlanner planner;
    printResult("TC1 Basic reachability", planner.plan(p));
}

// Test Case 2: Bad state avoidance. S(1)->A(2)->X(3, bad)->G(4) vs S(1)->C(5)->D(6)->G(4)
static void testCase2() {
    PlanningProblem p;
    p.initialState = 1; p.goalState = 4;
    p.badStates = {3};
    p.states = {st(1,0,0), st(2,1,1), st(3,2,1), st(4,4,0), st(5,1,-1), st(6,2,-1)};
    p.transitions = {
        tr(201,1,2,1.0), tr(202,2,3,1.0), tr(203,3,4,1.0), // through bad state X
        tr(204,1,5,1.5), tr(205,5,6,1.5), tr(206,6,4,1.5)  // safe detour
    };

    LPAStarPlanner planner;
    printResult("TC2 Bad state avoidance", planner.plan(p));
}

// Test Case 3: Safety margin trade-off between a cheap-but-close path and a
// costlier-but-far path. Bad state sits right next to the cheap route.
static void testCase3() {
    PlanningProblem p;
    p.initialState = 1; p.goalState = 4;
    p.badStates = {99};
    p.states = {
        st(1,0,0), st(2,1,0.1), st(3,2,0.1), st(4,3,0), // cheap path, hugs the bad state
        st(5,1,3), st(6,2,3),                           // costlier path, far from it
        st(99,1.5,0)                                    // the bad state itself
    };
    p.transitions = {
        tr(301,1,2,1.0), tr(302,2,3,1.0), tr(303,3,4,1.0),   // cheap, unsafe-ish
        tr(304,1,5,2.0), tr(305,5,6,2.0), tr(306,6,4,2.0)    // costlier, safe
    };

    LPAStarPlanner::Weights w; // push distSafetyWeight up to see the planner favor the far path
    w.distSafetyWeight = 5.0;
    LPAStarPlanner planner(w);
    printResult("TC3 Safety margin (safety-weighted)", planner.plan(p));

    LPAStarPlanner::Weights wCheap; wCheap.distSafetyWeight = 0.0; // pure cost minimization
    LPAStarPlanner plannerCheap(wCheap);
    printResult("TC3 Safety margin (cost-only)", plannerCheap.plan(p));
}

// Test Case 4: Dynamic transition removal. S(1)->A(2)->G(3), then (A,G) drops out.
static void testCase4() {
    PlanningProblem p;
    p.initialState = 1; p.goalState = 3;
    p.states = {st(1,0,0), st(2,1,0), st(3,2,0), st(4,1,-2), st(5,2,-2)};
    p.transitions = {
        tr(401,1,2,1.0), tr(402,2,3,1.0),  // primary route
        tr(403,1,4,2.0), tr(404,4,5,2.0), tr(405,5,3,2.0) // fallback, costlier
    };

    LPAStarPlanner planner;
    planner.loadProblem(p);
    printResult("TC4 Before edge removal", planner.getPath(3));

    planner.setTransitionAvailable(402, false); // (A, G) becomes unavailable
    printResult("TC4 After edge removal (replanned)", planner.getPath(3));
    std::cout << "        LPA* expansions triggered by this single update: "
              << planner.lastReplanExpansions() << "\n";
}

// Test Case 5: Goal update mid-execution, with zero replanning.
static void testCase5() {
    PlanningProblem p;
    p.initialState = 1; p.goalState = 3;
    p.states = {st(1,0,0), st(2,1,0), st(3,2,0), st(4,1,2), st(5,2,2)};
    p.transitions = {
        tr(501,1,2,1.0), tr(502,2,3,1.0),
        tr(503,1,4,1.0), tr(504,4,5,1.0)
    };

    LPAStarPlanner planner;
    planner.loadProblem(p);
    printResult("TC5 Original goal (3)", planner.getPath(3));

    planner.setGoal(5); // goal moves - no recomputation happens
    printResult("TC5 Updated goal (5), zero replanning", planner.getPath(5));
}

// Test Case 6: New shortcut transition improves the solution.
static void testCase6() {
    PlanningProblem p;
    p.initialState = 1; p.goalState = 4;
    p.states = {st(1,0,0), st(2,1,0), st(3,2,0), st(4,3,0)};
    p.transitions = {tr(601,1,2,1.0), tr(602,2,3,1.0), tr(603,3,4,1.0)};

    LPAStarPlanner planner;
    planner.loadProblem(p);
    printResult("TC6 Before shortcut", planner.getPath(4));

    planner.addTransition(tr(604,1,4,1.2)); // direct shortcut, cheaper than the 3-hop route
    printResult("TC6 After shortcut added (replanned)", planner.getPath(4));
}

int main() {
    testCase1();
    testCase2();
    testCase3();
    testCase4();
    testCase5();
    testCase6();
    return 0;
}