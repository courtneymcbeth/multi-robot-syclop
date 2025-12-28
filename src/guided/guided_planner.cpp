#include "guided_planner.h"
#include "syclop_rrt_solver.h"
#include <stdexcept>

std::unique_ptr<GuidedPlanner> createGuidedPlanner(
    const std::string& method,
    const GuidedPlannerConfig& config,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager)
{
    if (method == "syclop_rrt") {
        return std::make_unique<SyclopRRTSolver>(config, collision_manager);
    }

    // Future planners can be added here:
    // else if (method == "sst") { ... }
    // else if (method == "kpiece") { ... }

    throw std::runtime_error("Unknown guided planner method: " + method);
}
