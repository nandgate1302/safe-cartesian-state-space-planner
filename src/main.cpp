// General-purpose CLI: loads a PlanningProblem from a file (or stdin),
// solves it, then drops into an interactive prompt where you can query
// paths and trigger live updates (goal changes, edge changes) to see
// LPA*'s incremental replanning in action.
//
// ---------------------------------------------------------------------
// INPUT FILE FORMAT (blank lines and lines starting with '#' are ignored
// anywhere - use them freely to comment your graph):
//
//   <numStates> <dimension>
//   <id> <x1> <x2> ... <x_dimension>      (one line per state)
//   ...
//   <numBadStates>
//   <id>                                    (one line per bad state)
//   ...
//   <numTransitions>
//   <id> <from> <to> <cost> <safety> <reliability> [available]
//   ...                                     (one line per transition;
//                                             [available] is 0 or 1,
//                                             defaults to 1 if omitted)
//   <initialState> <goalState>
//
// See examples/sample_graph.txt for a working example.
//
// USAGE:
//   safe_planner path/to/graph.txt     (load from a file)
//   safe_planner                       (type/paste the graph directly, then
//                                        keep typing commands on the same input)
//
// Once loaded, interactive commands:
//   path [goalId]           print path to current goal, or to goalId
//   goal <id>               change the goal (instant, no replanning)
//   avail <transId> <0|1>   mark a transition unavailable/available
//   cost <transId> <cost>   update a transition's cost
//   add <id> <from> <to> <cost> <safety> <reliability> [avail]
//                            add a new transition
//   remove <transId>        remove (disable) a transition
//   stats                    expansions triggered by the last update
//   help                     show this list again
//   quit                     exit
// ---------------------------------------------------------------------

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include "LPAStarPlanner.h"
#include "ProblemLoader.h"

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

static void printHelp() {
    std::cout <<
        "Commands:\n"
        "  path [goalId]           - print path to current goal, or to goalId\n"
        "  goal <id>                - change the goal (instant, no replanning)\n"
        "  avail <transId> <0|1>    - mark a transition unavailable/available\n"
        "  cost <transId> <cost>    - update a transition's cost\n"
        "  add <id> <from> <to> <cost> <safety> <reliability> [avail]\n"
        "                            - add a new transition\n"
        "  remove <transId>         - remove (disable) a transition\n"
        "  stats                    - expansions triggered by the last update\n"
        "  help                     - show this list again\n"
        "  quit                     - exit\n";
}

int main(int argc, char** argv) {
    PlanningProblem problem;
    std::string error;
    bool ok;

    if (argc > 1) {
        std::ifstream file(argv[1]);
        if (!file) {
            std::cerr << "Could not open file: " << argv[1] << "\n";
            return 1;
        }
        ok = loadProblemFromStream(file, problem, error);
    } else {
        std::cout << "No input file given - type or paste the graph now "
                  << "(see the format in main.cpp's header comment).\n"
                  << "Tip: run 'safe_planner <file>' instead to load from a file.\n\n";
        ok = loadProblemFromStream(std::cin, problem, error);
    }

    if (!ok) {
        std::cerr << "Failed to parse input: " << error << "\n";
        return 1;
    }

    LPAStarPlanner planner;
    planner.loadProblem(problem);

    std::cout << "\nLoaded " << problem.states.size() << " states, "
              << problem.transitions.size() << " transitions, "
              << problem.badStates.size() << " bad states.\n";

    uint64_t currentGoal = problem.goalState;
    printResult("initial path", planner.getPath(currentGoal));
    std::cout << "\n";
    printHelp();

    std::string line;
    std::cout << "\n> ";
    while (std::getline(std::cin, line)) {
        std::istringstream iss(line);
        std::string cmd;
        if (!(iss >> cmd)) { std::cout << "> "; continue; }

        if (cmd == "quit" || cmd == "exit") {
            break;
        } else if (cmd == "help") {
            printHelp();
        } else if (cmd == "path") {
            uint64_t g = currentGoal;
            iss >> g; // if omitted, defaults to currentGoal
            printResult("path", planner.getPath(g));
        } else if (cmd == "goal") {
            uint64_t g;
            if (iss >> g) {
                planner.setGoal(g);
                currentGoal = g;
                printResult("goal updated", planner.getPath(g));
            } else {
                std::cout << "usage: goal <id>\n";
            }
        } else if (cmd == "avail") {
            uint64_t id; int a;
            if (iss >> id >> a) {
                planner.setTransitionAvailable(id, a != 0);
                printResult("replanned", planner.getPath(currentGoal));
                std::cout << "  expansions: " << planner.lastReplanExpansions() << "\n";
            } else {
                std::cout << "usage: avail <transitionId> <0|1>\n";
            }
        } else if (cmd == "cost") {
            uint64_t id; double c;
            if (iss >> id >> c) {
                planner.updateTransitionCost(id, c);
                printResult("replanned", planner.getPath(currentGoal));
                std::cout << "  expansions: " << planner.lastReplanExpansions() << "\n";
            } else {
                std::cout << "usage: cost <transitionId> <newCost>\n";
            }
        } else if (cmd == "add") {
            Transition t{};
            int avail = 1;
            if (iss >> t.id >> t.from >> t.to >> t.cost >> t.safety >> t.reliability) {
                iss >> avail;
                t.available = (avail != 0);
                planner.addTransition(t);
                printResult("replanned", planner.getPath(currentGoal));
                std::cout << "  expansions: " << planner.lastReplanExpansions() << "\n";
            } else {
                std::cout << "usage: add <id> <from> <to> <cost> <safety> <reliability> [avail]\n";
            }
        } else if (cmd == "remove") {
            uint64_t id;
            if (iss >> id) {
                planner.removeTransition(id);
                printResult("replanned", planner.getPath(currentGoal));
                std::cout << "  expansions: " << planner.lastReplanExpansions() << "\n";
            } else {
                std::cout << "usage: remove <transitionId>\n";
            }
        } else if (cmd == "stats") {
            std::cout << "Last update triggered " << planner.lastReplanExpansions() << " expansions.\n";
        } else {
            std::cout << "Unknown command. Type 'help' for a list.\n";
        }
        std::cout << "\n> ";
    }
    return 0;
}