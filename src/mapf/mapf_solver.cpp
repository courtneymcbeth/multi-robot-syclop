#include "mapf_solver.h"
#include "mapf_decoupled.h"

std::unique_ptr<MAPFSolver> createMAPFSolver(const std::string& method) {
    if (method == "decoupled" || method == "astar") {
        return std::make_unique<DecoupledMAPFSolver>();
    }
    // Future: CBS, ECBS, etc.
    return nullptr;
}
