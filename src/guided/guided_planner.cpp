// Pinocchio/Crocoddyl must be included before Boost/OMPL headers
#include <dynoplan/optimization/ocp.hpp>

#include "guided_planner.h"
#include "syclop_rrt_solver.h"
#include "db_rrt_solver.h"
#include <stdexcept>

namespace mr_syclop {

std::unique_ptr<GuidedPlanner> createGuidedPlanner(
    const std::string& method,
    const GuidedPlannerConfig& config,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager)
{
    if (method == "syclop_rrt") {
        return std::make_unique<SyclopRRTSolver>(config, collision_manager);
    }
    else if (method == "db_rrt") {
        // Use default DB-RRT config
        // Note: For custom DB-RRT configuration, use createGuidedPlannerWithDBRRT()
        // or enhance MRSyCLoPConfig to include DBRRTConfig
        DBRRTConfig db_config;
        db_config.models_base_path = "db-CBS/dynoplan/dynobench/models";
        db_config.motions_file = ""; // Will auto-detect
        db_config.timelimit = config.time_per_robot;
        db_config.debug = config.debug;
        return std::make_unique<DBRRTSolver>(config, db_config, collision_manager);
    }

    // Future planners can be added here:
    // else if (method == "sst") { ... }
    // else if (method == "kpiece") { ... }

    throw std::runtime_error("Unknown guided planner method: " + method);
}

// Factory function with DB-RRT config
std::unique_ptr<GuidedPlanner> createGuidedPlannerWithDBRRT(
    const std::string& method,
    const GuidedPlannerConfig& config,
    const DBRRTConfig& db_config,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager,
    const std::vector<dynobench::Obstacle>& dynobench_obstacles)
{
    if (method == "db_rrt") {
        return std::make_unique<DBRRTSolver>(config, db_config, collision_manager, dynobench_obstacles);
    }

    // Fall back to regular factory
    return createGuidedPlanner(method, config, collision_manager);
}

} // namespace mr_syclop
