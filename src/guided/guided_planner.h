#ifndef GUIDED_PLANNER_H
#define GUIDED_PLANNER_H

// Base types (no dynobench dependency)
#include "guided_planner_base.h"

// Include dynobench before OMPL/Boost to avoid header conflicts
#include <dynobench/robot_models_base.hpp>  // For dynobench::Obstacle

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
// Factory Function with DB-RRT support
// ============================================================================

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
