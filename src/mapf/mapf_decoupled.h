#ifndef MAPF_DECOUPLED_H
#define MAPF_DECOUPLED_H

#include "mapf_solver.h"
#include <boost/graph/adjacency_list.hpp>

// ============================================================================
// Graph type definitions
// ============================================================================

struct EdgeProperty {
    double weight;
};

using RegionGraph = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
                                          boost::no_property, EdgeProperty>;
using Vertex = boost::graph_traits<RegionGraph>::vertex_descriptor;

// ============================================================================
// Decoupled MAPF Solver (A* per robot, independent planning)
// ============================================================================

class DecoupledMAPFSolver : public MAPFSolver {
public:
    DecoupledMAPFSolver() = default;

    std::vector<std::vector<int>> solve(
        oc::DecompositionPtr decomp,
        const std::vector<ob::State*>& start_states,
        const std::vector<ob::State*>& goal_states) override;

    std::string getName() const override { return "Decoupled A*"; }

private:
    RegionGraph buildRegionGraph(oc::DecompositionPtr decomp);

    std::vector<int> findPathAStar(
        const RegionGraph& graph,
        int start_region,
        int goal_region);
};

#endif // MAPF_DECOUPLED_H
