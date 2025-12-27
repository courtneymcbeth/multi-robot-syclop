#include "mapf_decoupled.h"
#include <boost/graph/astar_search.hpp>

// ============================================================================
// Helper structures for A* search
// ============================================================================

struct found_goal {};

class GoalVisitor : public boost::default_astar_visitor {
public:
    GoalVisitor(int goal) : goal_region(goal) {}

    void examine_vertex(Vertex v, const RegionGraph& /*g*/) {
        if (static_cast<int>(v) == goal_region) {
            throw found_goal();
        }
    }

private:
    int goal_region;
};

class SimpleHeuristic : public boost::astar_heuristic<RegionGraph, double> {
public:
    SimpleHeuristic() {}

    double operator()(Vertex /*v*/) {
        return 0.0; // Uniform heuristic (admissible, reduces to Dijkstra's)
    }
};

// ============================================================================
// DecoupledMAPFSolver Implementation
// ============================================================================

RegionGraph DecoupledMAPFSolver::buildRegionGraph(oc::DecompositionPtr decomp) {
    int num_regions = decomp->getNumRegions();
    RegionGraph graph(num_regions);

    // Add edges with uniform weight 1
    for (int i = 0; i < num_regions; ++i) {
        std::vector<int> neighbors;
        decomp->getNeighbors(i, neighbors);
        for (int neighbor : neighbors) {
            auto edge = boost::add_edge(i, neighbor, graph);
            graph[edge.first].weight = 1.0;
        }
    }

    return graph;
}

std::vector<int> DecoupledMAPFSolver::findPathAStar(
    const RegionGraph& graph,
    int start_region,
    int goal_region)
{
    std::vector<int> path;

    if (start_region == goal_region) {
        path.push_back(start_region);
        return path;
    }

    int num_regions = boost::num_vertices(graph);
    std::vector<Vertex> parents(num_regions);
    std::vector<double> distances(num_regions);

    try {
        boost::astar_search(graph, boost::vertex(start_region, graph),
                            SimpleHeuristic(),
                            boost::weight_map(boost::get(&EdgeProperty::weight, graph))
                                .distance_map(boost::make_iterator_property_map(distances.begin(),
                                                                                boost::get(boost::vertex_index, graph)))
                                .predecessor_map(boost::make_iterator_property_map(parents.begin(),
                                                                                    boost::get(boost::vertex_index, graph)))
                                .visitor(GoalVisitor(goal_region)));
    } catch (found_goal) {
        // Reconstruct the path from start to goal
        int region = goal_region;
        int path_length = 1;

        while (region != start_region) {
            region = parents[region];
            ++path_length;
        }

        path.resize(path_length);
        region = goal_region;
        for (int i = path_length - 1; i >= 0; --i) {
            path[i] = region;
            region = parents[region];
        }
    }

    return path;
}

std::vector<std::vector<int>> DecoupledMAPFSolver::solve(
    oc::DecompositionPtr decomp,
    const std::vector<ob::State*>& start_states,
    const std::vector<ob::State*>& goal_states)
{
    std::vector<std::vector<int>> high_level_paths;
    high_level_paths.resize(start_states.size());

    // Build the region graph from the decomposition
    RegionGraph graph = buildRegionGraph(decomp);

    // Iterate over each robot and find independent paths
    for (size_t robot_idx = 0; robot_idx < start_states.size(); ++robot_idx) {
        // Get the region IDs for the start and goal states
        int start_region = decomp->locateRegion(start_states[robot_idx]);
        int goal_region = decomp->locateRegion(goal_states[robot_idx]);

        // Find path using A*
        high_level_paths[robot_idx] = findPathAStar(graph, start_region, goal_region);
    }

    return high_level_paths;
}
