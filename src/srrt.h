#ifndef SRRT_H
#define SRRT_H

#include <vector>
#include <set>
#include <map>
#include <memory>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <fcl/fcl.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Forward declaration
class Robot;

/**
 * Motion node for forward search tree in composite space
 * Similar to OMPL's Motion but with collision set
 */
struct ForwardMotion {
    ob::State* state;                   // Joint configuration (all robots)
    ForwardMotion* parent;              // Parent motion in forward tree
    std::set<int> collision_set;        // Set of robots requiring coordination

    ForwardMotion() : state(nullptr), parent(nullptr) {}

    ForwardMotion(const ob::SpaceInformationPtr& si)
        : state(si->allocState()), parent(nullptr) {}
};

/**
 * Node in the policy tree (extracted from RRT)
 */
struct PolicyNode {
    ob::State* state;
    ob::State* parent_state;  // Parent's state (toward goal)

    PolicyNode() : state(nullptr), parent_state(nullptr) {}
};

/**
 * Individual Policy Tree - backward RRT from goal for a single robot
 * Uses OMPL's RRT planner internally
 */
class IndividualPolicyTree {
public:
    IndividualPolicyTree(const ob::SpaceInformationPtr& si, int robot_id);
    ~IndividualPolicyTree();

    // Build the backward tree from the goal state toward the start state
    void buildTree(ob::State* start_state, ob::State* goal_state, double timeout);

    // Get policy: returns next state toward goal
    // Uses nearest neighbor query on the RRT tree
    ob::State* getPolicy(ob::State* query_state);

    // Get the underlying RRT planner (for advanced queries)
    std::shared_ptr<og::RRT> getRRTPlanner() { return rrt_planner_; }

    int getRobotId() const { return robot_id_; }

private:
    // Extract tree structure from RRT planner data
    void extractTreeStructure();

    // Distance function for nearest neighbors
    double distanceFunction(const PolicyNode* a, const PolicyNode* b) const;

    ob::SpaceInformationPtr si_;
    int robot_id_;
    std::shared_ptr<og::RRT> rrt_planner_;
    ob::ProblemDefinitionPtr pdef_;
    ob::State* goal_state_;  // Store goal for lazy expansion

    // Extracted tree structure for efficient policy queries
    std::shared_ptr<ompl::NearestNeighbors<PolicyNode*>> nn_;
    std::vector<PolicyNode*> policy_nodes_;
};

/**
 * Forward Search Tree - tree in composite space with collision sets
 * Uses OMPL's NearestNeighbors but custom Motion nodes
 */
class ForwardSearchTree {
public:
    ForwardSearchTree(const ob::SpaceInformationPtr& si, int num_robots);
    ~ForwardSearchTree();

    // Initialize with start state
    void initialize(ob::State* start_state);

    // Find nearest motion to a state
    ForwardMotion* findNearest(ob::State* state);

    // Add a new motion to the tree
    void addMotion(ForwardMotion* motion);

    // Get all motions
    const std::vector<ForwardMotion*>& getMotions() const { return motions_; }

    ForwardMotion* getRoot() const { return root_; }

private:
    // Distance function for nearest neighbors
    double distanceFunction(const ForwardMotion* a, const ForwardMotion* b) const;

    ob::SpaceInformationPtr si_;
    int num_robots_;
    std::shared_ptr<ompl::NearestNeighbors<ForwardMotion*>> nn_;
    std::vector<ForwardMotion*> motions_;
    ForwardMotion* root_;
};

/**
 * sRRT Planner - Subdimensional RRT for multi-robot planning
 */
class sRRTPlanner {
public:
    sRRTPlanner(const ob::SpaceInformationPtr& si,
                int num_robots,
                const std::vector<ob::SpaceInformationPtr>& individual_si,
                const std::vector<std::shared_ptr<Robot>>& robots = {});
    ~sRRTPlanner();

    // Main planning function
    bool solve(const std::vector<ob::State*>& start_states,
               const std::vector<ob::State*>& goal_states,
               double timeout);

    // Get solution path if found
    std::vector<ob::State*> getSolutionPath() const;

    // Configuration parameters
    void setMaxDistance(double distance) { max_distance_ = distance; }
    void setGoalThreshold(double threshold) { goal_threshold_ = threshold; }
    void setEdgeCheckStep(double step) { edge_check_step_ = step; }
    double getMaxDistance() const { return max_distance_; }
    double getGoalThreshold() const { return goal_threshold_; }

private:
    // === Phase 1: Initialization ===
    void buildIndividualPolicies(const std::vector<ob::State*>& start_states,
                                  const std::vector<ob::State*>& goal_states,
                                  double timeout);
    void initializeForwardTree(const std::vector<ob::State*>& start_states);

    // === Phase 2: Main Loop ===
    bool mainSearchLoop(const std::vector<ob::State*>& goal_states, double timeout);

    // Sample random configuration in composite space
    ob::State* sampleCompositeState();

    // Find nearest motion in forward tree
    ForwardMotion* findNearestMotion(ob::State* state);

    // Project sample onto subdimensional search space
    ob::State* projectState(ob::State* sample, ForwardMotion* nearest);

    // Extend from nearest toward projected state
    ob::State* extend(ForwardMotion* from, ob::State* toward);

    // Collision checking
    bool checkRobotObstacleCollision(ob::State* state);
    std::set<int> checkRobotRobotCollisions(ob::State* from, ob::State* to);
    std::set<int> checkRobotRobotCollisionsAlongEdge(ob::State* from, ob::State* to);

    // Update collision sets
    void updateCollisionSet(ForwardMotion* motion, const std::set<int>& new_collisions);
    void propagateCollisionSet(ForwardMotion* motion, int robot_id);

    // Goal checking and path extraction
    bool isGoalReached(ob::State* state, const std::vector<ob::State*>& goal_states);
    void extractPath(ForwardMotion* goal_motion);

    // Helper functions for composite/individual state conversion
    ob::State* combineIndividualStates(const std::vector<ob::State*>& individual_states);
    void extractIndividualState(ob::State* composite_state, int robot_id, ob::State* individual_state);

private:
    ob::SpaceInformationPtr composite_si_;  // Composite state space
    std::vector<ob::SpaceInformationPtr> individual_si_;  // Individual state spaces
    int num_robots_;

    // Individual policy trees (OMPL RRT planners, backward from goals)
    std::vector<std::unique_ptr<IndividualPolicyTree>> individual_policies_;

    // Forward search tree (composite space with collision sets)
    std::unique_ptr<ForwardSearchTree> forward_tree_;

    // Solution path
    std::vector<ob::State*> solution_path_;

    // Samplers
    ob::StateSamplerPtr composite_sampler_;

    // Robot objects for collision checking
    std::vector<std::shared_ptr<Robot>> robots_;

    // Parameters
    double max_distance_;
    double goal_threshold_;
    double edge_check_step_;
    double collision_radius_;  // Fallback distance-based collision radius
};

#endif // SRRT_H
