#ifndef MAPF_SOLVER_H
#define MAPF_SOLVER_H

#include <ompl/control/planners/syclop/Decomposition.h>
#include <ompl/base/State.h>
#include <memory>
#include <vector>
#include <string>

namespace ob = ompl::base;
namespace oc = ompl::control;

class MAPFSolver {
public:
    virtual ~MAPFSolver() = default;

    virtual std::vector<std::vector<int>> solve(
        oc::DecompositionPtr decomp,
        const std::vector<ob::State*>& start_states,
        const std::vector<ob::State*>& goal_states) = 0;

    virtual std::string getName() const = 0;
};

std::unique_ptr<MAPFSolver> createMAPFSolver(const std::string& method);

#endif // MAPF_SOLVER_H
