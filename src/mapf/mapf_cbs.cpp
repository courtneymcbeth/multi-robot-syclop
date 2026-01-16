#include "mapf_cbs.h"
#include "../decomposition.h"
#include <boost/graph/astar_search.hpp>
#include <queue>
#include <iostream>
#include <stdexcept>
#include <tuple>
#include <set>

// Uncomment for debug output
// #define CBS_DEBUG

// ============================================================================
// Constructor
// ============================================================================

CBSMAPFSolver::CBSMAPFSolver(int region_capacity, double timeout)
    : region_capacity_(region_capacity), timeout_(timeout)
{
#ifdef CBS_DEBUG
    std::cout << "CBS Solver initialized with capacity=" << region_capacity_
              << ", timeout=" << timeout_ << "s" << std::endl;
#endif
}

// ============================================================================
// Main Solve Method
// ============================================================================

std::vector<std::vector<int>> CBSMAPFSolver::solve(
    oc::DecompositionPtr decomp,
    const std::vector<ob::State*>& start_states,
    const std::vector<ob::State*>& goal_states,
    const std::vector<fcl::CollisionObjectf*>& obstacles,
    double max_obstacle_volume_percent)
{
    auto start_time = std::chrono::high_resolution_clock::now();

#ifdef CBS_DEBUG
    std::cout << "CBS: Starting solve for " << start_states.size() << " robots" << std::endl;
#endif

    // We need access to region bounds to validate start/goal regions
    auto grid_decomp = std::dynamic_pointer_cast<GridDecompositionImpl>(decomp);
    if (!grid_decomp) {
        throw std::runtime_error("CBSMAPFSolver: Decomposition must be GridDecompositionImpl");
    }

    // Build region graph (filters invalid regions)
    RegionGraph graph = buildRegionGraph(decomp, obstacles, max_obstacle_volume_percent);

    // Initialize root CT node
    CTNode root;
    root.constraints_per_robot.resize(start_states.size());
    root.paths.resize(start_states.size());
    root.cost = 0;

    // Find initial paths for all robots (no constraints)
    for (size_t i = 0; i < start_states.size(); ++i) {
        int start_region = decomp->locateRegion(start_states[i]);
        int goal_region = decomp->locateRegion(goal_states[i]);

#ifdef CBS_DEBUG
        std::cout << "CBS: Planning initial path for robot " << i
                  << " from region " << start_region << " to " << goal_region << std::endl;
#endif

        // Validate that regions are valid
        if (start_region < 0) {
            throw std::runtime_error("CBS: Start state for robot " + std::to_string(i) +
                                     " is outside decomposition bounds (region=" + std::to_string(start_region) + ")");
        }
        if (goal_region < 0) {
            throw std::runtime_error("CBS: Goal state for robot " + std::to_string(i) +
                                     " is outside decomposition bounds (region=" + std::to_string(goal_region) + ")");
        }

        // Validate start region obstacle volume
        const auto& start_bounds = grid_decomp->getRegionBoundsPublic(start_region);
        double start_obstacle_percent = computeObstacleVolumePercent(start_bounds, obstacles);
        if (start_obstacle_percent > max_obstacle_volume_percent) {
            throw std::runtime_error(
                "CBSMAPFSolver: Start region " + std::to_string(start_region) +
                " for robot " + std::to_string(i) +
                " has " + std::to_string(start_obstacle_percent * 100.0) +
                "% obstacle volume, exceeding threshold of " +
                std::to_string(max_obstacle_volume_percent * 100.0) + "%");
        }

        // Validate goal region obstacle volume
        const auto& goal_bounds = grid_decomp->getRegionBoundsPublic(goal_region);
        double goal_obstacle_percent = computeObstacleVolumePercent(goal_bounds, obstacles);
        if (goal_obstacle_percent > max_obstacle_volume_percent) {
            throw std::runtime_error(
                "CBSMAPFSolver: Goal region " + std::to_string(goal_region) +
                " for robot " + std::to_string(i) +
                " has " + std::to_string(goal_obstacle_percent * 100.0) +
                "% obstacle volume, exceeding threshold of " +
                std::to_string(max_obstacle_volume_percent * 100.0) + "%");
        }

        try {
            root.paths[i] = findPathWithConstraints(
                graph, start_region, goal_region,
                root.constraints_per_robot[i]);
            root.cost += root.paths[i].cost;

#ifdef CBS_DEBUG
            std::cout << "  Robot " << i << " initial path: ";
            for (size_t j = 0; j < root.paths[i].regions.size(); ++j) {
                std::cout << "(" << root.paths[i].regions[j] << ",t="
                          << root.paths[i].times[j] << ") ";
            }
            std::cout << "  cost=" << root.paths[i].cost << std::endl;
#endif
        } catch (const std::exception& e) {
            throw std::runtime_error("CBS: No initial path found for robot " +
                                     std::to_string(i) + ": " + e.what());
        }
    }

#ifdef CBS_DEBUG
    std::cout << "CBS: Root node cost = " << root.cost << std::endl;
#endif

    // Priority queue for CT nodes (min-heap by cost)
    std::priority_queue<CTNode, std::vector<CTNode>, std::greater<CTNode>> open_list;
    open_list.push(root);

    int iterations = 0;
    const int max_iterations = 100000;  // Safety limit

    // Main CBS loop
    while (!open_list.empty()) {
        // Check timeout
        auto current_time = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(current_time - start_time).count();
        if (elapsed > timeout_) {
            throw std::runtime_error("CBS: Timeout after " + std::to_string(elapsed) + " seconds");
        }

        // Check iteration limit
        if (++iterations > max_iterations) {
            throw std::runtime_error("CBS: Exceeded maximum iterations (" +
                                     std::to_string(max_iterations) + ")");
        }

        CTNode current = open_list.top();
        open_list.pop();

#ifdef CBS_DEBUG
        std::cout << "CBS: Iteration " << iterations << ", cost = " << current.cost << std::endl;
#endif

        // Detect conflicts in current solution
        current.conflict = detectFirstConflict(current.paths);

        // If no conflicts, return solution
        if (current.conflict.type == Conflict::NONE) {
#ifdef CBS_DEBUG
            std::cout << "CBS: Solution found after " << iterations << " iterations" << std::endl;
            std::cout << "Final paths with timing:" << std::endl;
            for (size_t i = 0; i < current.paths.size(); ++i) {
                std::cout << "  Robot " << i << ": ";
                for (size_t j = 0; j < current.paths[i].regions.size(); ++j) {
                    std::cout << "(" << current.paths[i].regions[j] << ",t="
                              << current.paths[i].times[j] << ") ";
                }
                std::cout << std::endl;
            }
#endif
            return extractRegionSequences(current.paths);
        }

#ifdef CBS_DEBUG
        if (current.conflict.type == Conflict::VERTEX) {
            std::cout << "CBS: Vertex conflict at region " << current.conflict.vertex_conflict.region
                      << " at time " << current.conflict.vertex_conflict.time
                      << " involving " << current.conflict.vertex_conflict.robot_ids.size()
                      << " robots" << std::endl;
        } else {
            std::cout << "CBS: Edge conflict on edge (" << current.conflict.edge_conflict.from_region
                      << "->" << current.conflict.edge_conflict.to_region << ") at time "
                      << current.conflict.edge_conflict.time << " involving "
                      << current.conflict.edge_conflict.robot_ids.size() << " robots" << std::endl;
        }
#endif

        // Generate child nodes by adding constraints
        std::vector<CTNode> children = generateChildNodes(
            current, graph, decomp, start_states, goal_states);

        // Add children to open list
        for (auto& child : children) {
            open_list.push(child);
        }
    }

    // No solution found
    throw std::runtime_error("CBS: No solution found (open list exhausted)");
}

// ============================================================================
// Graph Construction
// ============================================================================

RegionGraph CBSMAPFSolver::buildRegionGraph(
    oc::DecompositionPtr decomp,
    const std::vector<fcl::CollisionObjectf*>& obstacles,
    double max_obstacle_volume_percent)
{
    int num_regions = decomp->getNumRegions();
    RegionGraph graph(num_regions);

    // We need access to region bounds to filter by obstacle volume
    auto grid_decomp = std::dynamic_pointer_cast<GridDecompositionImpl>(decomp);
    if (!grid_decomp) {
        throw std::runtime_error("CBSMAPFSolver: Decomposition must be GridDecompositionImpl");
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
                EdgeProperty ep;
                ep.weight = 1.0;  // Uniform cost
                boost::add_edge(i, neighbor, ep, graph);
            }
        }
    }

    return graph;
}

void CBSMAPFSolver::getNeighbors(const RegionGraph& graph, int region,
                                 std::vector<int>& neighbors)
{
    neighbors.clear();
    auto out_edges = boost::out_edges(region, graph);
    for (auto it = out_edges.first; it != out_edges.second; ++it) {
        neighbors.push_back(boost::target(*it, graph));
    }
}

// ============================================================================
// Conflict Detection
// ============================================================================

Conflict CBSMAPFSolver::detectFirstConflict(const std::vector<TimedPath>& paths)
{
    // Build occupancy tables
    std::map<std::pair<int,int>, std::vector<int>> vertex_occupancy;  // [region][time] -> robot IDs
    std::map<std::tuple<int,int,int>, std::vector<int>> edge_occupancy;  // [from][to][time] -> robot IDs

    // Populate occupancy tables
    for (size_t robot_id = 0; robot_id < paths.size(); ++robot_id) {
        const TimedPath& path = paths[robot_id];

        // Vertex occupancy
        for (size_t i = 0; i < path.regions.size(); ++i) {
            vertex_occupancy[{path.regions[i], path.times[i]}].push_back(robot_id);
        }

        // Edge occupancy
        for (size_t i = 0; i + 1 < path.regions.size(); ++i) {
            edge_occupancy[{path.regions[i], path.regions[i+1], path.times[i]}].push_back(robot_id);
        }
    }

    // Check for vertex capacity violations (prioritize earlier times)
    int earliest_conflict_time = std::numeric_limits<int>::max();
    Conflict earliest_conflict;
    earliest_conflict.type = Conflict::NONE;

    for (const auto& [key, robot_ids] : vertex_occupancy) {
        if (static_cast<int>(robot_ids.size()) > region_capacity_) {
            int time = key.second;
            if (time < earliest_conflict_time) {
                earliest_conflict_time = time;
                earliest_conflict.type = Conflict::VERTEX;
                earliest_conflict.vertex_conflict.robot_ids = robot_ids;
                earliest_conflict.vertex_conflict.region = key.first;
                earliest_conflict.vertex_conflict.time = time;
            }
        }
    }

    // Check for edge capacity violations (same direction)
    for (const auto& [key, robot_ids] : edge_occupancy) {
        if (static_cast<int>(robot_ids.size()) > region_capacity_) {
            int time = std::get<2>(key);
            if (time < earliest_conflict_time ||
                (time == earliest_conflict_time && earliest_conflict.type != Conflict::VERTEX)) {
                earliest_conflict_time = time;
                earliest_conflict.type = Conflict::EDGE;
                earliest_conflict.edge_conflict.robot_ids = robot_ids;
                earliest_conflict.edge_conflict.from_region = std::get<0>(key);
                earliest_conflict.edge_conflict.to_region = std::get<1>(key);
                earliest_conflict.edge_conflict.time = time;
            }
        }
    }

    // Check for swapping conflicts (opposite direction edge conflicts)
    // For each edge (A->B) at time t, check if there's an edge (B->A) at time t
    for (const auto& [key1, robot_ids1] : edge_occupancy) {
        int from1 = std::get<0>(key1);
        int to1 = std::get<1>(key1);
        int time1 = std::get<2>(key1);

        // Look for opposite direction edge at same time
        auto key2 = std::make_tuple(to1, from1, time1);
        auto it = edge_occupancy.find(key2);
        if (it != edge_occupancy.end()) {
            const auto& robot_ids2 = it->second;

            // Swapping conflict: robots going opposite directions
            // Total robots using this edge (both directions) exceeds capacity
            std::vector<int> all_robots = robot_ids1;
            all_robots.insert(all_robots.end(), robot_ids2.begin(), robot_ids2.end());

            if (static_cast<int>(all_robots.size()) > region_capacity_) {
                if (time1 < earliest_conflict_time ||
                    (time1 == earliest_conflict_time && earliest_conflict.type != Conflict::VERTEX)) {
                    earliest_conflict_time = time1;
                    earliest_conflict.type = Conflict::EDGE;
                    earliest_conflict.edge_conflict.robot_ids = all_robots;
                    earliest_conflict.edge_conflict.from_region = from1;
                    earliest_conflict.edge_conflict.to_region = to1;
                    earliest_conflict.edge_conflict.time = time1;
                }
            }
        }
    }

    return earliest_conflict;
}

// ============================================================================
// Child Node Generation
// ============================================================================

std::vector<CTNode> CBSMAPFSolver::generateChildNodes(
    const CTNode& parent,
    const RegionGraph& graph,
    oc::DecompositionPtr decomp,
    const std::vector<ob::State*>& start_states,
    const std::vector<ob::State*>& goal_states)
{
    std::vector<CTNode> children;

    // Get conflicting robots from parent's conflict
    std::vector<int> conflicting_robots;
    if (parent.conflict.type == Conflict::VERTEX) {
        conflicting_robots = parent.conflict.vertex_conflict.robot_ids;
    } else {
        conflicting_robots = parent.conflict.edge_conflict.robot_ids;
    }

    // Create one child node per conflicting robot
    for (int robot_id : conflicting_robots) {
        CTNode child = parent;  // Copy parent

        // Add constraint to this robot
        if (parent.conflict.type == Conflict::VERTEX) {
            VertexConstraint vc;
            vc.robot_id = robot_id;
            vc.region = parent.conflict.vertex_conflict.region;
            vc.time = parent.conflict.vertex_conflict.time;
            child.constraints_per_robot[robot_id].vertex_constraints.push_back(vc);

#ifdef CBS_DEBUG
            std::cout << "  Adding vertex constraint: robot " << robot_id
                      << " cannot be at region " << vc.region
                      << " at time " << vc.time << std::endl;
#endif
        } else {
            EdgeConstraint ec;
            ec.robot_id = robot_id;
            ec.from_region = parent.conflict.edge_conflict.from_region;
            ec.to_region = parent.conflict.edge_conflict.to_region;
            ec.time = parent.conflict.edge_conflict.time;
            child.constraints_per_robot[robot_id].edge_constraints.push_back(ec);

#ifdef CBS_DEBUG
            std::cout << "  Adding edge constraint: robot " << robot_id
                      << " cannot traverse edge (" << ec.from_region
                      << "->" << ec.to_region << ") at time " << ec.time << std::endl;
#endif
        }

        // Replan path for this robot with new constraint
        int start_region = decomp->locateRegion(start_states[robot_id]);
        int goal_region = decomp->locateRegion(goal_states[robot_id]);

        try {
            child.paths[robot_id] = findPathWithConstraints(
                graph, start_region, goal_region,
                child.constraints_per_robot[robot_id]);

            // Update cost
            child.cost = 0;
            for (const auto& path : child.paths) {
                child.cost += path.cost;
            }

            children.push_back(child);
        } catch (const std::exception& e) {
            // No path found with this constraint, skip this child
#ifdef CBS_DEBUG
            std::cout << "CBS: Cannot generate child for robot " << robot_id
                      << " (no feasible path)" << std::endl;
#endif
        }
    }

    return children;
}

// ============================================================================
// Low-Level Space-Time A* Search
// ============================================================================

struct STNode {
    int region;
    int time;
    int g_cost;      // Cost from start
    int f_cost;      // g + h
    int parent_idx;  // For path reconstruction (-1 for start)

    bool operator>(const STNode& other) const {
        return f_cost > other.f_cost;
    }
};

TimedPath CBSMAPFSolver::findPathWithConstraints(
    const RegionGraph& graph,
    int start_region,
    int goal_region,
    const Constraints& constraints)
{
    // Storage for all nodes (closed list + current expansions)
    std::vector<STNode> nodes;

    // Priority queue (min-heap by f_cost)
    std::priority_queue<int, std::vector<int>, std::function<bool(int, int)>> open_list(
        [&nodes](int a, int b) {
            return nodes[a] > nodes[b];
        });

    // Visited set: (region, time)
    std::set<std::pair<int,int>> visited;

    // Initialize with start state
    STNode start_node;
    start_node.region = start_region;
    start_node.time = 0;
    start_node.g_cost = 0;
    start_node.f_cost = 0;  // Using zero heuristic (admissible)
    start_node.parent_idx = -1;

    nodes.push_back(start_node);
    open_list.push(0);

    const int max_time = 1000;  // Maximum time steps to consider
    const int max_expansions = 100000;  // Safety limit
    int expansions = 0;

    while (!open_list.empty()) {
        if (++expansions > max_expansions) {
            throw std::runtime_error("Low-level A*: Exceeded maximum expansions");
        }

        int current_idx = open_list.top();
        open_list.pop();

        const STNode& current = nodes[current_idx];

        // Skip if already visited
        if (visited.count({current.region, current.time})) {
            continue;
        }
        visited.insert({current.region, current.time});

        // Goal check
        if (current.region == goal_region) {
            // Reconstruct path
            TimedPath path;
            int idx = current_idx;
            while (idx != -1) {
                path.regions.push_back(nodes[idx].region);
                path.times.push_back(nodes[idx].time);
                idx = nodes[idx].parent_idx;
            }
            std::reverse(path.regions.begin(), path.regions.end());
            std::reverse(path.times.begin(), path.times.end());
            path.cost = current.time;  // Makespan
            return path;
        }

        // Safety: don't expand beyond max time
        if (current.time >= max_time) {
            continue;
        }

        // Expand neighbors (including wait action)
        std::vector<int> neighbors;
        getNeighbors(graph, current.region, neighbors);
        neighbors.push_back(current.region);  // Wait action

        for (int next_region : neighbors) {
            int next_time = current.time + 1;

            // Check vertex constraint
            if (violatesVertexConstraint(next_region, next_time, constraints)) {
                continue;
            }

            // Check edge constraint (only for actual moves, not wait)
            if (current.region != next_region &&
                violatesEdgeConstraint(current.region, next_region, next_time, constraints)) {
                continue;
            }

            // Skip if already visited
            if (visited.count({next_region, next_time})) {
                continue;
            }

            // Create successor node
            STNode successor;
            successor.region = next_region;
            successor.time = next_time;
            successor.g_cost = current.g_cost + 1;
            successor.f_cost = successor.g_cost;  // h = 0
            successor.parent_idx = current_idx;

            int successor_idx = nodes.size();
            nodes.push_back(successor);
            open_list.push(successor_idx);
        }
    }

    // No path found
    throw std::runtime_error("Low-level A*: No path found with constraints");
}

// ============================================================================
// Constraint Checking
// ============================================================================

bool CBSMAPFSolver::violatesVertexConstraint(int region, int time,
                                             const Constraints& constraints)
{
    for (const auto& vc : constraints.vertex_constraints) {
        if (vc.region == region && vc.time == time) {
            return true;
        }
    }
    return false;
}

bool CBSMAPFSolver::violatesEdgeConstraint(int from_region, int to_region, int time,
                                           const Constraints& constraints)
{
    for (const auto& ec : constraints.edge_constraints) {
        if (ec.from_region == from_region &&
            ec.to_region == to_region &&
            ec.time == time) {
            return true;
        }
    }
    return false;
}

// ============================================================================
// Path Extraction
// ============================================================================

std::vector<std::vector<int>> CBSMAPFSolver::extractRegionSequences(
    const std::vector<TimedPath>& paths)
{
    std::vector<std::vector<int>> result;
    result.reserve(paths.size());

    for (const auto& path : paths) {
        result.push_back(path.regions);
    }

    return result;
}
