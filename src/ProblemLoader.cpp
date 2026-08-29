#include "ProblemLoader.h"
#include <sstream>

// Reads the next line that has real content: skips blank lines and lines
// that are entirely comments (start with '#'). Returns false at end of
// stream. This lets the input file have blank spacing and explanatory
// comments anywhere without confusing the parser.
static bool nextMeaningfulLine(std::istream& in, std::string& line) {
    while (std::getline(in, line)) {
        std::size_t start = line.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) continue;      // blank line
        if (line[start] == '#') continue;               // comment line
        return true;
    }
    return false;
}

bool loadProblemFromStream(std::istream& in, PlanningProblem& outProblem, std::string& error) {
    std::string line;

    // Line: <numStates> <dimension>
    if (!nextMeaningfulLine(in, line)) { error = "expected state count line"; return false; }
    std::istringstream headerLine(line);
    std::size_t numStates = 0;
    int dimension = 0;
    if (!(headerLine >> numStates >> dimension) || dimension <= 0) {
        error = "malformed state count line - expected '<numStates> <dimension>'";
        return false;
    }

    outProblem.states.clear();
    outProblem.states.reserve(numStates);
    for (std::size_t i = 0; i < numStates; ++i) {
        if (!nextMeaningfulLine(in, line)) { error = "expected a state line"; return false; }
        std::istringstream stateLine(line);
        State s;
        if (!(stateLine >> s.id)) { error = "malformed state line (missing id)"; return false; }
        s.embedding.resize(dimension);
        for (int d = 0; d < dimension; ++d) {
            if (!(stateLine >> s.embedding[d])) {
                error = "malformed state line (expected " + std::to_string(dimension) + " coordinates)";
                return false;
            }
        }
        outProblem.states.push_back(s);
    }

    // Line: <numBadStates>
    if (!nextMeaningfulLine(in, line)) { error = "expected bad-state count line"; return false; }
    std::istringstream badHeader(line);
    std::size_t numBad = 0;
    if (!(badHeader >> numBad)) { error = "malformed bad-state count line"; return false; }

    outProblem.badStates.clear();
    for (std::size_t i = 0; i < numBad; ++i) {
        if (!nextMeaningfulLine(in, line)) { error = "expected a bad-state id line"; return false; }
        std::istringstream badLine(line);
        uint64_t id;
        if (!(badLine >> id)) { error = "malformed bad-state id line"; return false; }
        outProblem.badStates.push_back(id);
    }

    // Line: <numTransitions>
    if (!nextMeaningfulLine(in, line)) { error = "expected transition count line"; return false; }
    std::istringstream transHeader(line);
    std::size_t numTransitions = 0;
    if (!(transHeader >> numTransitions)) { error = "malformed transition count line"; return false; }

    outProblem.transitions.clear();
    outProblem.transitions.reserve(numTransitions);
    for (std::size_t i = 0; i < numTransitions; ++i) {
        if (!nextMeaningfulLine(in, line)) { error = "expected a transition line"; return false; }
        std::istringstream transLine(line);
        Transition t;
        int availableFlag = 1; // default: available, if the column is omitted
        if (!(transLine >> t.id >> t.from >> t.to >> t.cost >> t.safety >> t.reliability)) {
            error = "malformed transition line - expected "
                    "'<id> <from> <to> <cost> <safety> <reliability> [available]'";
            return false;
        }
        if (transLine >> availableFlag) {
            t.available = (availableFlag != 0);
        } else {
            t.available = true;
        }
        outProblem.transitions.push_back(t);
    }

    // Line: <initialState> <goalState>
    if (!nextMeaningfulLine(in, line)) { error = "expected initial/goal state line"; return false; }
    std::istringstream goalLine(line);
    if (!(goalLine >> outProblem.initialState >> outProblem.goalState)) {
        error = "malformed initial/goal state line - expected '<initialState> <goalState>'";
        return false;
    }

    return true;
}