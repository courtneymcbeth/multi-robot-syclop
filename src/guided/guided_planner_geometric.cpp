#include "guided_planner_base.h"
#include "syclop_rrt_solver.h"
#include "rrt_connect_solver.h"
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
    else if (method == "rrt_connect") {
        return std::make_unique<RRTConnectSolver>(config, collision_manager);
    }

    throw std::runtime_error("Unknown geometric guided planner method: " + method);
}

} // namespace mr_syclop
