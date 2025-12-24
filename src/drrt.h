#pragma once

#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <random>

#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/SE2StateSpace.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Forward declarations
class CompositeVertex;
class IndividualRoadmap;
class DiscreteRRTTree;
class DiscreteRRTPlanner;
class Robot;

// Represents a vertex in the tensor product graph
// Stores indices into each individual robot's roadmap
class CompositeVertex {
public:
    std::vector<unsigned int> vertex_indices;  // One index per robot
    CompositeVertex* parent;

    CompositeVertex(const std::vector<unsigned int>& indices)
        : vertex_indices(indices), parent(nullptr) {}

    // Hash function for use in unordered_set/map
    struct Hash {
        std::size_t operator()(const CompositeVertex& v) const;
    };

    // Equality comparison
    bool operator==(const CompositeVertex& other) const {
        return vertex_indices == other.vertex_indices;
    }

    // Check if this vertex differs from another in exactly one coordinate
    bool isNeighbor(const CompositeVertex& other) const;

    // Get the coordinate that differs (returns -1 if not a neighbor)
    int getDifferingCoordinate(const CompositeVertex& other) const;
};

// Stores a sparse roadmap for a single robot
class IndividualRoadmap {
public:
    struct SparsParams {
        bool has_max_failures = false;
        unsigned int max_failures = 0;
        bool has_sparse_delta_fraction = false;
        double sparse_delta_fraction = 0.0;
        bool has_dense_delta_fraction = false;
        double dense_delta_fraction = 0.0;
        bool has_stretch_factor = false;
        double stretch_factor = 0.0;
    };

    IndividualRoadmap(
        const std::shared_ptr<ob::SpaceInformation>& si,
        unsigned int num_samples = 500,
        double connection_radius = 1.0);

    IndividualRoadmap(
        const std::shared_ptr<ob::SpaceInformation>& si,
        unsigned int num_samples,
        double connection_radius,
        const SparsParams& spars_params);

    ~IndividualRoadmap() = default;

    // Build the roadmap with actual start and goal states
    void build(const ob::State* start, const ob::State* goal, double max_time = 10.0);

    // Get the number of vertices in the roadmap
    unsigned int getNumVertices() const;

    // Get the state associated with a vertex index
    const ob::State* getState(unsigned int index) const;

    // Get neighbors of a vertex
    std::vector<unsigned int> getNeighbors(unsigned int index) const;

    // Find the nearest vertex in the roadmap to a given state
    unsigned int getNearestVertex(const ob::State* state) const;

    // Find the neighbor of a vertex that moves closest to a target state
    // Returns the vertex index if found, or the same vertex if no valid neighbor
    unsigned int getNeighborToward(unsigned int vertex_idx, const ob::State* target_state) const;

    // Check if there's an edge between two vertices
    bool hasEdge(unsigned int v1, unsigned int v2) const;

private:
    void ensureVertexCache() const;

    std::shared_ptr<ob::SpaceInformation> si_;
    std::shared_ptr<og::SPARStwo> spars_;
    og::SimpleSetup ss_;

    unsigned int num_samples_;
    double connection_radius_;

    // Cache for neighbor lookups
    mutable std::unordered_map<unsigned int, std::vector<unsigned int>> neighbor_cache_;
    mutable bool vertex_cache_valid_ = false;
    mutable std::vector<og::SPARStwo::Vertex> vertex_cache_;
    mutable std::unordered_map<og::SPARStwo::Vertex, unsigned int> vertex_index_map_;
};

// Tree structure for discrete RRT
class DiscreteRRTTree {
public:
    DiscreteRRTTree(const std::vector<unsigned int>& root_indices);

    ~DiscreteRRTTree() = default;

    // Add a vertex to the tree with a parent
    void addVertex(const std::vector<unsigned int>& indices, CompositeVertex* parent);

    // Check if a vertex is in the tree
    bool contains(const std::vector<unsigned int>& indices) const;

    // Get a vertex from the tree (returns nullptr if not found)
    CompositeVertex* getVertex(const std::vector<unsigned int>& indices) const;

    // Get the root vertex
    CompositeVertex* getRoot() const { return root_.get(); }

    // Get all vertices in the tree
    const std::vector<std::unique_ptr<CompositeVertex>>& getVertices() const {
        return vertices_;
    }

    // Get size of tree
    size_t size() const { return vertex_map_.size(); }

private:
    std::unique_ptr<CompositeVertex> root_;
    std::vector<std::unique_ptr<CompositeVertex>> vertices_;

    // Hash table for fast lookup
    struct VectorHash {
        std::size_t operator()(const std::vector<unsigned int>& v) const {
            std::size_t seed = v.size();
            for (auto& i : v) {
                seed ^= i + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };

    std::unordered_map<std::vector<unsigned int>, CompositeVertex*, VectorHash> vertex_map_;
};

// Main discrete RRT planner
class DiscreteRRTPlanner {
public:
    DiscreteRRTPlanner(
        const std::vector<std::shared_ptr<IndividualRoadmap>>& roadmaps,
        const std::vector<std::shared_ptr<ob::SpaceInformation>>& space_info_list,
        const std::vector<std::shared_ptr<Robot>>& robots = {});

    ~DiscreteRRTPlanner() = default;

    // Set the start and goal configurations (as vertex indices)
    void setStartGoal(
        const std::vector<unsigned int>& start_indices,
        const std::vector<unsigned int>& goal_indices);

    // Set algorithm parameters
    void setExpansionsPerIteration(unsigned int n_it) { n_it_ = n_it; }

    // Run the planner
    bool solve(double max_time);

    // Get the solution path (if found)
    std::vector<std::vector<unsigned int>> getSolutionPath() const;

    // Get the solution as actual states
    std::vector<std::vector<const ob::State*>> getSolutionStates() const;

private:
    // Core algorithm functions
    void expandTree();

    // Sample a random continuous configuration
    std::vector<ob::ScopedState<>> sampleRandomConfiguration();

    // Find the nearest vertex in the tree to a continuous configuration
    CompositeVertex* nearestNeighborInTree(const std::vector<ob::ScopedState<>>& config);

    // Neighbor oracle: find a neighbor of v_near in direction of q_rand
    std::vector<unsigned int> neighborOracle(
        const CompositeVertex* v_near,
        const std::vector<ob::ScopedState<>>& q_rand);

    // Check if a composite vertex is collision-free
    bool isCollisionFree(const std::vector<unsigned int>& indices) const;

    // Check if the motion between two composite vertices is collision-free
    bool isCompositeMotionValid(
        const std::vector<unsigned int>& from_indices,
        const std::vector<unsigned int>& to_indices) const;

    // Check robot-robot collisions for a set of robot states
    bool isRobotRobotCollisionFree(const std::vector<const ob::State*>& states) const;

    // Check robot-robot collisions along the moving robot's edge
    bool isRobotRobotMotionValid(
        const std::vector<const ob::State*>& from_states,
        const std::vector<const ob::State*>& to_states) const;

    // Distance between two states
    double distance(const ob::State* s1, const ob::State* s2, size_t robot_idx) const;

    // Distance between composite configurations
    double compositeDistance(
        const std::vector<unsigned int>& indices,
        const std::vector<ob::ScopedState<>>& continuous_config) const;

    // Extract path from tree
    std::vector<std::vector<unsigned int>> extractPath(CompositeVertex* goal_vertex) const;

    // Member variables
    std::vector<std::shared_ptr<IndividualRoadmap>> roadmaps_;
    std::vector<std::shared_ptr<ob::SpaceInformation>> space_info_list_;
    std::vector<std::shared_ptr<Robot>> robots_;

    unsigned int num_robots_;

    std::vector<unsigned int> start_indices_;
    std::vector<unsigned int> goal_indices_;

    std::unique_ptr<DiscreteRRTTree> tree_;

    // Algorithm parameters
    unsigned int n_it_ = 5;  // Expansions per iteration

    // Random number generation
    std::mt19937 rng_;

    // Fallback collision radius (used when robots_ is not provided)
    double collision_radius_ = 0.5;

    // Statistics
    unsigned int num_expansions_ = 0;
    bool solution_found_ = false;
};
