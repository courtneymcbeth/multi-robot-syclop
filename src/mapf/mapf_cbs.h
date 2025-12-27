#ifndef MAPF_CBS_H
#define MAPF_CBS_H

#include "mapf_solver.h"
#include "mapf_decoupled.h"  // For EdgeProperty, RegionGraph, and Vertex types
#include <vector>
#include <map>
#include <set>
#include <chrono>

// ============================================================================
// CBS Data Structures
// ============================================================================

// Vertex constraint: robot cannot be at region at given time
struct VertexConstraint {
    int robot_id;
    int region;
    int time;

    bool operator==(const VertexConstraint& other) const {
        return robot_id == other.robot_id &&
               region == other.region &&
               time == other.time;
    }
};

// Edge constraint: robot cannot traverse edge at given time
struct EdgeConstraint {
    int robot_id;
    int from_region;
    int to_region;
    int time;

    bool operator==(const EdgeConstraint& other) const {
        return robot_id == other.robot_id &&
               from_region == other.from_region &&
               to_region == other.to_region &&
               time == other.time;
    }
};

// Container for all constraints on a single robot
struct Constraints {
    std::vector<VertexConstraint> vertex_constraints;
    std::vector<EdgeConstraint> edge_constraints;
};

// Path with explicit timing information
struct TimedPath {
    std::vector<int> regions;  // Region sequence
    std::vector<int> times;    // Time at each region
    int cost;                  // Path cost (makespan = final time)
};

// Vertex conflict: multiple robots in same region at same time
struct VertexConflict {
    std::vector<int> robot_ids;  // All robots involved (size > capacity)
    int region;
    int time;
};

// Edge conflict: multiple robots on same edge at same time
struct EdgeConflict {
    std::vector<int> robot_ids;  // All robots involved (size > capacity)
    int from_region;
    int to_region;
    int time;
};

// Union type for conflicts
struct Conflict {
    enum Type { VERTEX, EDGE, NONE };
    Type type;
    VertexConflict vertex_conflict;
    EdgeConflict edge_conflict;
};

// Constraint Tree (CT) node
struct CTNode {
    std::vector<Constraints> constraints_per_robot;  // One per robot
    std::vector<TimedPath> paths;                    // Planned paths
    int cost;                                        // Sum of path costs (sum of makespans)
    Conflict conflict;                               // First detected conflict

    // For priority queue ordering (lower cost = higher priority)
    bool operator>(const CTNode& other) const {
        return cost > other.cost;
    }
};

// ============================================================================
// CBS MAPF Solver with Capacity Constraints
// ============================================================================

class CBSMAPFSolver : public MAPFSolver {
public:
    CBSMAPFSolver(int region_capacity, double timeout);

    std::vector<std::vector<int>> solve(
        oc::DecompositionPtr decomp,
        const std::vector<ob::State*>& start_states,
        const std::vector<ob::State*>& goal_states) override;

    std::string getName() const override { return "CBS"; }

private:
    int region_capacity_;  // Maximum robots per vertex/edge
    double timeout_;       // Maximum planning time in seconds

    // Graph construction
    RegionGraph buildRegionGraph(oc::DecompositionPtr decomp);
    void getNeighbors(const RegionGraph& graph, int region, std::vector<int>& neighbors);

    // High-level CBS
    Conflict detectFirstConflict(const std::vector<TimedPath>& paths);
    std::vector<CTNode> generateChildNodes(
        const CTNode& parent,
        const RegionGraph& graph,
        oc::DecompositionPtr decomp,
        const std::vector<ob::State*>& start_states,
        const std::vector<ob::State*>& goal_states);

    // Low-level space-time A*
    TimedPath findPathWithConstraints(
        const RegionGraph& graph,
        int start_region,
        int goal_region,
        const Constraints& constraints);

    // Constraint checking
    bool violatesVertexConstraint(int region, int time, const Constraints& constraints);
    bool violatesEdgeConstraint(int from_region, int to_region, int time,
                                const Constraints& constraints);

    // Path extraction
    std::vector<std::vector<int>> extractRegionSequences(const std::vector<TimedPath>& paths);
};

#endif // MAPF_CBS_H
