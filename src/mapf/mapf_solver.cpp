#include "mapf_solver.h"
#include "mapf_decoupled.h"
#include "mapf_cbs.h"
#include <stdexcept>

std::unique_ptr<MAPFSolver> createMAPFSolver(
    const std::string& method,
    int region_capacity,
    double timeout)
{
    if (method == "decoupled" || method == "astar") {
        return std::make_unique<DecoupledMAPFSolver>();
    }
    else if (method == "cbs") {
        return std::make_unique<CBSMAPFSolver>(region_capacity, timeout);
    }

    throw std::runtime_error("Unknown MAPF method: " + method);
}
