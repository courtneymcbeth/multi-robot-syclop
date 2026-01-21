#include "mapf_decoupled.h"
#include "../decomposition.h"
#include <boost/graph/astar_search.hpp>
#include <stdexcept>
#include <iostream>
#include <set>

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

RegionGraph DecoupledMAPFSolver::buildRegionGraph(
    oc::DecompositionPtr decomp,
    const std::vector<fcl::CollisionObjectf*>& obstacles,
    double max_obstacle_volume_percent)
{
    int num_regions = decomp->getNumRegions();
    RegionGraph graph(num_regions);

    // We need access to region bounds to filter by obstacle volume
    // This requires GridDecompositionImpl to access getRegionBounds
    auto grid_decomp = std::dynamic_pointer_cast<GridDecompositionImpl>(decomp);
    if (!grid_decomp) {
        throw std::runtime_error("DecoupledMAPFSolver: Decomposition must be GridDecompositionImpl");
    }

    // Build a set of valid regions (those with acceptable obstacle volume)
    std::set<int> valid_regions;
    for (int i = 0; i < num_regions; ++i) {
        const auto& region_bounds = grid_decomp->getRegionBoundsPublic(i);
        double obstacle_percent = computeObstacleVolumePercent(region_bounds, obstacles);

        if (obstacle_percent <= max_obstacle_volume_percent) {
            valid_regions.insert(i);
        }
    }

    // Add edges only between valid regions
    for (int i = 0; i < num_regions; ++i) {
        // Skip invalid regions
        if (valid_regions.find(i) == valid_regions.end()) {
            continue;
        }

        std::vector<int> neighbors;
        decomp->getNeighbors(i, neighbors);
        for (int neighbor : neighbors) {
            // Only add edge if neighbor is also valid
            if (valid_regions.find(neighbor) != valid_regions.end()) {
                auto edge = boost::add_edge(i, neighbor, graph);
                graph[edge.first].weight = 1.0;
            }
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

    bool found = false;
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
        found = true;
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

    if (!found) {
        throw std::runtime_error("Decoupled A*: No path found from region " +
                                 std::to_string(start_region) + " to region " +
                                 std::to_string(goal_region));
    }

    return path;
}

std::vector<std::vector<int>> DecoupledMAPFSolver::solve(
    oc::DecompositionPtr decomp,
    const std::vector<ob::State*>& start_states,
    const std::vector<ob::State*>& goal_states,
    const std::vector<fcl::CollisionObjectf*>& obstacles,
    double max_obstacle_volume_percent)
{
    std::vector<std::vector<int>> high_level_paths;
    high_level_paths.resize(start_states.size());

    // We need access to region bounds to validate start/goal regions
    auto grid_decomp = std::dynamic_pointer_cast<GridDecompositionImpl>(decomp);
    if (!grid_decomp) {
        throw std::runtime_error("DecoupledMAPFSolver: Decomposition must be GridDecompositionImpl");
    }

    // Build the region graph from the decomposition (filters invalid regions)
    RegionGraph graph = buildRegionGraph(decomp, obstacles, max_obstacle_volume_percent);
    int num_regions = decomp->getNumRegions();

    // Iterate over each robot and find independent paths
    for (size_t robot_idx = 0; robot_idx < start_states.size(); ++robot_idx) {
        // Get the region IDs for the start and goal states
        int start_region = decomp->locateRegion(start_states[robot_idx]);
        int goal_region = decomp->locateRegion(goal_states[robot_idx]);

        // Validate that regions are valid
        if (start_region < 0 || start_region >= num_regions) {
            throw std::runtime_error("Decoupled A*: Start state for robot " +
                                     std::to_string(robot_idx) +
                                     " is outside decomposition bounds (region=" +
                                     std::to_string(start_region) + ", num_regions=" +
                                     std::to_string(num_regions) + ")");
        }
        if (goal_region < 0 || goal_region >= num_regions) {
            throw std::runtime_error("Decoupled A*: Goal state for robot " +
                                     std::to_string(robot_idx) +
                                     " is outside decomposition bounds (region=" +
                                     std::to_string(goal_region) + ", num_regions=" +
                                     std::to_string(num_regions) + ")");
        }

        // Validate start region
        const auto& start_bounds = grid_decomp->getRegionBoundsPublic(start_region);
        double start_obstacle_percent = computeObstacleVolumePercent(start_bounds, obstacles);
        if (start_obstacle_percent > max_obstacle_volume_percent) {
            throw std::runtime_error(
                "DecoupledMAPFSolver: Start region " + std::to_string(start_region) +
                " for robot " + std::to_string(robot_idx) +
                " has " + std::to_string(start_obstacle_percent * 100.0) +
                "% obstacle volume, exceeding threshold of " +
                std::to_string(max_obstacle_volume_percent * 100.0) + "%");
        }

        // Validate goal region
        const auto& goal_bounds = grid_decomp->getRegionBoundsPublic(goal_region);
        double goal_obstacle_percent = computeObstacleVolumePercent(goal_bounds, obstacles);
        if (goal_obstacle_percent > max_obstacle_volume_percent) {
            throw std::runtime_error(
                "DecoupledMAPFSolver: Goal region " + std::to_string(goal_region) +
                " for robot " + std::to_string(robot_idx) +
                " has " + std::to_string(goal_obstacle_percent * 100.0) +
                "% obstacle volume, exceeding threshold of " +
                std::to_string(max_obstacle_volume_percent * 100.0) + "%");
        }

        // Find path using A*
        high_level_paths[robot_idx] = findPathAStar(graph, start_region, goal_region);
    }

    return high_level_paths;
}
