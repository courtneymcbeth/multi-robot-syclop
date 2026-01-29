/**
 * @file composite_dbrrt.h
 * @brief Composite DB-RRT Planner for Multi-Robot Motion Planning
 *
 * This planner combines:
 * - Individual DB-RRT trees per robot (using motion primitives from dynoplan)
 * - Tensor product coordination in composite configuration space (from drrt.cpp)
 *
 * Each robot maintains its own dynamically growing DB-RRT tree, and the planner
 * coordinates their expansion through the tensor product of their state spaces.
 */

#pragma once

// Pinocchio/Crocoddyl must be included before Boost/OMPL headers
#include <dynoplan/optimization/ocp.hpp>

// Standard library
#include <memory>
#include <vector>
#include <unordered_map>
#include <random>
#include <atomic>
#include <string>
#include <map>

// Eigen
#include <Eigen/Core>

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/PathControl.h>
#include <ompl/datastructures/NearestNeighbors.h>

// FCL
#include <fcl/fcl.h>

// dynobench/dynoplan
#include "dynobench/robot_models.hpp"
#include "dynobench/motions.hpp"
#include "dynoplan/dbastar/dbastar.hpp"
#include "dynoplan/dbrrt/dbrrt.hpp"

// Local
#include "../db-CBS/src/robots.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

// Forward declarations
struct DBRRTNode;
struct CompositeDBRRTVertex;
class CompositeDBRRTTree;
class IndividualDBRRT;
class CompositeDBRRTPlanner;

// =============================================================================
// Configuration
// =============================================================================

/**
 * @brief Configuration for the Composite DB-RRT planner
 */
struct CompositeDBRRTConfig {
    // Time limit for planning (seconds)
    double time_limit = 60.0;

    // Per-robot DB-RRT parameters
    double delta = 0.3;              // Motion primitive search radius
    double goal_region = 0.3;        // Goal threshold for individual robots
    size_t max_motions = 1000;       // Max primitives to load per robot

    // Composite search parameters
    unsigned int expansions_per_iter = 5;  // Expansions per iteration
    double goal_threshold = 0.5;           // Per-robot goal check threshold
    bool require_all_move = false;         // If true, all robots must move in each expansion
                                           // If false, robots can stay in place (more flexible)

    // Goal bias for sampling
    double goal_bias = 0.1;

    // Motion primitive files (keyed by robot type)
    std::map<std::string, std::string> motion_files;

    // Dynobench models base path
    std::string models_base_path;

    // Random seed (-1 for random)
    int seed = -1;

    // Debug output
    bool debug = false;
};

// =============================================================================
// Input/Output Structures (compatible with coupled_rrt.h)
// =============================================================================

/**
 * @brief Specification for a single robot
 */
struct CompositeRobotSpec {
    std::string type;              // Robot type string (e.g., "unicycle1_v0")
    std::vector<double> start;     // Start state
    std::vector<double> goal;      // Goal state
};

/**
 * @brief Planning problem definition
 */
struct CompositeDBRRTProblem {
    std::vector<double> env_min;   // Environment bounds minimum [x_min, y_min]
    std::vector<double> env_max;   // Environment bounds maximum [x_max, y_max]
    std::vector<CompositeRobotSpec> robots;  // Robot specifications
    std::vector<fcl::CollisionObjectf*> obstacles;  // FCL collision objects
    std::vector<dynobench::Obstacle> dynobench_obstacles;  // For dynobench
};

/**
 * @brief Planning result
 */
struct CompositeDBRRTResult {
    bool solved = false;                   // Whether solution was found
    double planning_time = 0.0;            // Time spent planning
    std::vector<dynobench::Trajectory> trajectories;  // Per-robot trajectories
};

// =============================================================================
// DBRRTNode - Node in an individual robot's DB-RRT tree
// =============================================================================

/**
 * @brief Node in an individual robot's DB-RRT tree
 *
 * Similar to AStarNode from dynoplan but adapted for composite search.
 */
struct DBRRTNode {
    Eigen::VectorXd state;           // Robot state
    DBRRTNode* parent;               // Parent node in tree
    int used_motion;                 // Motion primitive index used to reach this node
    double cost;                     // Cost from root (time or distance)
    size_t id;                       // Unique ID for composite vertex hashing

    // Cached applicable motions (computed lazily)
    std::vector<int> cached_motions;
    bool motions_computed = false;

    // For trajectory reconstruction
    int intermediate_state = -1;

    DBRRTNode()
        : parent(nullptr)
        , used_motion(-1)
        , cost(0.0)
        , id(0)
    {}

    // Get unique ID for this node (thread-safe)
    static size_t getNextId() {
        static std::atomic<size_t> next_id{0};
        return next_id++;
    }
};

// =============================================================================
// CompositeDBRRTVertex - Vertex in the tensor product space
// =============================================================================

/**
 * @brief Vertex in the composite (tensor product) configuration space
 *
 * Instead of storing indices into static roadmaps (like drrt.cpp),
 * stores pointers to nodes in each robot's DB-RRT tree.
 */
struct CompositeDBRRTVertex {
    std::vector<DBRRTNode*> robot_nodes;  // One node per robot
    CompositeDBRRTVertex* parent;          // Parent in composite tree
    double g_score;                        // Cost from start

    CompositeDBRRTVertex(const std::vector<DBRRTNode*>& nodes)
        : robot_nodes(nodes)
        , parent(nullptr)
        , g_score(0.0)
    {}

    // Hash function for unordered_map lookup
    struct Hash {
        std::size_t operator()(const CompositeDBRRTVertex& v) const {
            std::size_t seed = v.robot_nodes.size();
            for (auto* node : v.robot_nodes) {
                seed ^= node->id + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };

    bool operator==(const CompositeDBRRTVertex& other) const {
        if (robot_nodes.size() != other.robot_nodes.size()) return false;
        for (size_t i = 0; i < robot_nodes.size(); ++i) {
            if (robot_nodes[i]->id != other.robot_nodes[i]->id) return false;
        }
        return true;
    }
};

// =============================================================================
// CompositeDBRRTTree - Tree structure for composite search
// =============================================================================

/**
 * @brief Tree of composite vertices for multi-robot search
 */
class CompositeDBRRTTree {
public:
    CompositeDBRRTTree(const std::vector<DBRRTNode*>& root_nodes);
    ~CompositeDBRRTTree() = default;

    // Add a vertex to the tree with a parent
    void addVertex(const std::vector<DBRRTNode*>& nodes, CompositeDBRRTVertex* parent);

    // Check if a composite vertex is in the tree
    bool contains(const std::vector<DBRRTNode*>& nodes) const;

    // Get a vertex from the tree (returns nullptr if not found)
    CompositeDBRRTVertex* getVertex(const std::vector<DBRRTNode*>& nodes) const;

    // Get the root vertex
    CompositeDBRRTVertex* getRoot() const { return root_.get(); }

    // Get all vertices in the tree
    const std::vector<std::unique_ptr<CompositeDBRRTVertex>>& getVertices() const {
        return vertices_;
    }

    // Get size of tree
    size_t size() const { return vertex_map_.size(); }

private:
    std::unique_ptr<CompositeDBRRTVertex> root_;
    std::vector<std::unique_ptr<CompositeDBRRTVertex>> vertices_;

    // Hash table for fast lookup using node IDs
    struct NodeVectorHash {
        std::size_t operator()(const std::vector<DBRRTNode*>& v) const {
            std::size_t seed = v.size();
            for (auto* node : v) {
                seed ^= node->id + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };

    struct NodeVectorEqual {
        bool operator()(const std::vector<DBRRTNode*>& a,
                       const std::vector<DBRRTNode*>& b) const {
            if (a.size() != b.size()) return false;
            for (size_t i = 0; i < a.size(); ++i) {
                if (a[i]->id != b[i]->id) return false;
            }
            return true;
        }
    };

    std::unordered_map<std::vector<DBRRTNode*>, CompositeDBRRTVertex*,
                       NodeVectorHash, NodeVectorEqual> vertex_map_;
};

// =============================================================================
// IndividualDBRRT - Per-robot DB-RRT tree wrapper
// =============================================================================

/**
 * @brief Manages a single robot's DB-RRT tree using motion primitives
 *
 * This replaces IndividualRoadmap from drrt.cpp with a dynamic tree
 * that grows using motion primitives from the dynoplan database.
 */
class IndividualDBRRT {
public:
    struct Config {
        double delta = 0.3;           // NN search radius for motions
        double goal_region = 0.3;     // Goal threshold
        double max_step_size = 3.0;   // Max expansion step
        size_t max_motions = 1000;    // Max primitives to load
        bool use_nigh_nn = true;      // Use nigh for NN
        int seed = -1;
    };

    IndividualDBRRT(
        std::shared_ptr<Robot> robot,
        std::shared_ptr<dynobench::Model_robot> dynobench_robot,
        const Config& config);

    ~IndividualDBRRT() = default;

    // Initialize the tree with a start state
    void initialize(const Eigen::VectorXd& start_state);

    // Load motion primitives from file
    bool loadMotions(const std::string& motions_file);

    // Tree operations
    DBRRTNode* addNode(const Eigen::VectorXd& state, DBRRTNode* parent, int motion_idx);
    DBRRTNode* nearest(const Eigen::VectorXd& state);

    // Expansion toward target - returns new node or nullptr if expansion failed
    DBRRTNode* expandToward(DBRRTNode* near_node, const Eigen::VectorXd& target);

    // Accessors
    DBRRTNode* getRoot() const { return root_; }
    size_t size() const { return nodes_.size(); }
    std::shared_ptr<Robot> getRobot() const { return robot_; }
    std::shared_ptr<dynobench::Model_robot> getDynobenchRobot() const { return dynobench_robot_; }
    const std::vector<dynoplan::Motion>& getMotions() const { return motions_; }

    // Check if a state is at goal
    bool isAtGoal(const Eigen::VectorXd& state, const Eigen::VectorXd& goal) const;

private:
    // Get applicable motions from a node
    std::vector<dynoplan::LazyTraj> getApplicableMotions(DBRRTNode* from_node);

    // Validate a motion (obstacle collision checking)
    bool validateMotion(dynoplan::LazyTraj& lazy_traj, Eigen::VectorXd& end_state);

    std::shared_ptr<Robot> robot_;
    std::shared_ptr<dynobench::Model_robot> dynobench_robot_;
    Config config_;

    // Tree storage
    DBRRTNode* root_;
    std::vector<std::unique_ptr<DBRRTNode>> nodes_;

    // NN structure for tree nodes
    std::unique_ptr<ompl::NearestNeighbors<DBRRTNode*>> tree_nn_;

    // Motion primitives
    std::vector<dynoplan::Motion> motions_;
    std::unique_ptr<ompl::NearestNeighbors<dynoplan::Motion*>> motion_nn_;

    // Expander for lazy trajectory generation
    std::unique_ptr<dynoplan::Expander> expander_;

    // Trajectory wrapper for validation
    dynobench::TrajWrapper traj_wrapper_;
    Eigen::VectorXd offset_;
    Eigen::VectorXd aux_last_state_;

    // Random number generator
    std::mt19937 rng_;
};

// =============================================================================
// CompositeDBRRTPlanner - Main multi-robot planner
// =============================================================================

/**
 * @brief Main Composite DB-RRT planner for multi-robot motion planning
 *
 * Combines individual DB-RRT trees with tensor product coordination.
 */
class CompositeDBRRTPlanner {
public:
    CompositeDBRRTPlanner(const CompositeDBRRTConfig& config);
    ~CompositeDBRRTPlanner();

    // Main planning function
    CompositeDBRRTResult plan(const CompositeDBRRTProblem& problem);

private:
    CompositeDBRRTConfig config_;

    // Individual trees for each robot
    std::vector<std::unique_ptr<IndividualDBRRT>> individual_trees_;

    // Composite tree
    std::unique_ptr<CompositeDBRRTTree> composite_tree_;

    // Robot instances
    std::vector<std::shared_ptr<Robot>> robots_;
    std::vector<std::shared_ptr<dynobench::Model_robot>> dynobench_robots_;

    // Environment collision manager
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> env_collision_manager_;
    ob::RealVectorBounds position_bounds_;

    // Goal states for each robot
    std::vector<Eigen::VectorXd> start_states_;
    std::vector<Eigen::VectorXd> goal_states_;

    // Random number generation
    std::mt19937 rng_;

    // Statistics
    unsigned int num_expansions_ = 0;
    bool solution_found_ = false;
    CompositeDBRRTVertex* goal_vertex_ = nullptr;

    // Setup methods
    void setupEnvironment(const CompositeDBRRTProblem& problem);
    void setupRobots(const CompositeDBRRTProblem& problem);
    void setupIndividualTrees(const CompositeDBRRTProblem& problem);
    void setupCompositeTree();
    void cleanup();

    // Core algorithm methods
    bool solve(double max_time);
    void expandCompositeTree();
    std::vector<Eigen::VectorXd> sampleRandomConfiguration();
    CompositeDBRRTVertex* nearestCompositeVertex(
        const std::vector<Eigen::VectorXd>& config);
    std::vector<DBRRTNode*> tensorProductExpansion(
        CompositeDBRRTVertex* v_near,
        const std::vector<Eigen::VectorXd>& q_rand);

    // Collision checking
    bool isCompositeMotionValid(
        const std::vector<DBRRTNode*>& from_nodes,
        const std::vector<DBRRTNode*>& to_nodes);
    bool isRobotRobotCollisionFree(
        const std::vector<Eigen::VectorXd>& states) const;
    bool isRobotRobotMotionValid(
        const std::vector<Eigen::VectorXd>& from_states,
        const std::vector<Eigen::VectorXd>& to_states) const;

    // Goal checking
    bool checkGoalReached();
    double compositeDistance(
        const std::vector<DBRRTNode*>& nodes,
        const std::vector<Eigen::VectorXd>& config);

    // Solution extraction
    std::vector<dynobench::Trajectory> extractSolution(
        CompositeDBRRTVertex* goal_vertex);
};

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * @brief Load configuration from YAML node
 */
CompositeDBRRTConfig loadCompositeDBRRTConfig(const YAML::Node& node);

