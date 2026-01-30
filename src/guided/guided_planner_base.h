#ifndef GUIDED_PLANNER_BASE_H
#define GUIDED_PLANNER_BASE_H

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
// Configuration Structure for Guided Planners
// ============================================================================

struct GuidedPlannerConfig {
    // Time budget per robot (seconds)
    double time_per_robot = 10.0;

    // Goal tolerance
    double goal_threshold = 0.5;

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
// Factory Function (geometric planners only, no DB-RRT dependency)
// ============================================================================

/**
 * @brief Create a guided planner instance
 *
 * @param method - Planner method name ("syclop_rrt")
 * @param config - Configuration parameters
 * @param collision_manager - FCL collision manager for environment obstacles
 * @return Unique pointer to GuidedPlanner instance
 */
std::unique_ptr<GuidedPlanner> createGuidedPlanner(
    const std::string& method,
    const GuidedPlannerConfig& config,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager);

} // namespace mr_syclop

#endif // GUIDED_PLANNER_BASE_H
