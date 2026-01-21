#ifndef GUIDED_PLANNER_H
#define GUIDED_PLANNER_H

// Include dynobench before OMPL/Boost to avoid header conflicts
#include <dynobench/robot_models_base.hpp>  // For dynobench::Obstacle

#include <ompl/control/PathControl.h>
#include <ompl/control/planners/syclop/Decomposition.h>
#include <ompl/base/State.h>
#include <fcl/broadphase/broadphase_collision_manager.h>
#include <memory>
#include <vector>
#include <string>

namespace ob = ompl::base;
namespace oc = ompl::control;

// Forward declare Robot class from global namespace
class Robot;

namespace mr_syclop {

// ============================================================================
// DB-RRT Configuration Structure
// ============================================================================

struct DBRRTConfig {
    // Path to motion primitives file (msgpack format)
    std::string motions_file;

    // Path to dynobench models directory
    std::string models_base_path;

    // DB-RRT parameters
    double timelimit = 60.0;              // Planning time limit in seconds
    int max_expands = 10000;              // Maximum tree expansions
    double goal_region = 0.3;             // Goal region radius
    double delta = 1.0;                   // NN search radius in canonical space (was 0.3, increased for better angular coverage)
    double goal_bias = 0.1;               // Probability of sampling goal
    int max_motions = 1000;               // Max motion primitives to load
    int seed = -1;                        // Random seed (-1 for random)
    bool do_optimization = false;         // Enable trajectory optimization
    bool use_nigh_nn = true;              // Use nigh nearest neighbor
    bool debug = false;                   // Debug output

    // Trajectory optimization parameters
    int solver_id = 0;                    // Trajectory optimizer solver ID

    // Region guidance parameters (SYCLOP-style)
    bool use_region_guidance = true;      // Use region path for guidance
    double region_bias = 0.3;             // Bias towards regions in path

    // SYCLOP-style lead following parameters
    int num_region_expansions = 20;       // Expansions per region before moving to next
    double prob_follow_lead = 0.8;        // Probability of following the lead path
    double prob_abandon_lead_early = 0.25; // Probability of abandoning lead for shortcut
    bool use_waypoint_planning = true;    // Plan through intermediate waypoints along lead
    double waypoint_tolerance = 0.5;      // Tolerance for reaching waypoint before advancing
};

// ============================================================================
// Configuration Structure for Guided Planners
// ============================================================================

struct GuidedPlannerConfig {
    // Time budget per robot (seconds)
    double time_per_robot = 10.0;

    // SyclopRRT-specific parameters
    int num_free_volume_samples = 100000;
    int num_region_expansions = 100;
    int num_tree_expansions = 1;

    // Probability parameters
    double prob_abandon_lead_early = 0.25;
    double prob_shortest_path = 0.95;

    // Whether to use regional nearest neighbors
    bool use_regional_nn = true;

    // Debug output
    bool debug = false;
};

// ============================================================================
// Result Structure for Individual Robot Planning
// ============================================================================

struct GuidedPlanningResult {
    bool success;
    double planning_time;
    std::shared_ptr<oc::PathControl> path;
    size_t robot_index;
};

// ============================================================================
// GuidedPlanner Abstract Base Class
// ============================================================================

class GuidedPlanner {
public:
    GuidedPlanner(const GuidedPlannerConfig& config) : config_(config) {}
    virtual ~GuidedPlanner() = default;

    /**
     * @brief Solve guided planning problem for a single robot
     *
     * @param robot - Robot object with dynamics and collision geometry
     * @param decomp - Decomposition shared with MAPF
     * @param start_state - Start state for this robot
     * @param goal_state - Goal state for this robot
     * @param region_path - High-level path (sequence of region IDs) from MAPF
     * @param robot_index - Index of this robot (for debugging/logging)
     * @return GuidedPlanningResult containing path or failure indication
     */
    virtual GuidedPlanningResult solve(
        std::shared_ptr<::Robot> robot,
        oc::DecompositionPtr decomp,
        ob::State* start_state,
        ob::State* goal_state,
        const std::vector<int>& region_path,
        size_t robot_index) = 0;

    /**
     * @brief Get the name of this guided planner
     */
    virtual std::string getName() const = 0;

protected:
    GuidedPlannerConfig config_;
};

// ============================================================================
// Factory Function
// ============================================================================

/**
 * @brief Create a guided planner instance
 *
 * @param method - Planner method name ("syclop_rrt", "db_rrt", future: "sst", etc.)
 * @param config - Configuration parameters
 * @param collision_manager - FCL collision manager for environment obstacles
 * @return Unique pointer to GuidedPlanner instance
 */
std::unique_ptr<GuidedPlanner> createGuidedPlanner(
    const std::string& method,
    const GuidedPlannerConfig& config,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager);

/**
 * @brief Create a guided planner instance with DB-RRT specific config
 *
 * @param method - Planner method name
 * @param config - General guided planner configuration
 * @param db_config - DB-RRT specific configuration
 * @param collision_manager - FCL collision manager for environment obstacles
 * @param dynobench_obstacles - Precomputed obstacles in dynobench format (optional)
 * @return Unique pointer to GuidedPlanner instance
 */
std::unique_ptr<GuidedPlanner> createGuidedPlannerWithDBRRT(
    const std::string& method,
    const GuidedPlannerConfig& config,
    const DBRRTConfig& db_config,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager,
    const std::vector<dynobench::Obstacle>& dynobench_obstacles = {});

} // namespace mr_syclop

#endif // GUIDED_PLANNER_H
