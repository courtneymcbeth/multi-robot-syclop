#ifndef SYCLOP_RRT_SOLVER_H
#define SYCLOP_RRT_SOLVER_H

#include "guided_planner.h"
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <fcl/fcl.h>
#include <memory>

namespace mr_syclop {

/**
 * @brief SyclopRRT-based guided planner implementation
 *
 * This class wraps OMPL's SyclopRRT planner to work with the multi-robot
 * framework. It uses the high-level path from MAPF as guidance by setting
 * a custom lead computation function.
 */
class SyclopRRTSolver : public GuidedPlanner {
public:
    /**
     * @brief Constructor
     *
     * @param config - Configuration parameters for SyclopRRT
     * @param collision_manager - FCL collision manager for obstacles
     */
    SyclopRRTSolver(
        const GuidedPlannerConfig& config,
        std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager);

    ~SyclopRRTSolver() override = default;

    /**
     * @brief Solve guided planning for one robot
     */
    GuidedPlanningResult solve(
        std::shared_ptr<::Robot> robot,
        oc::DecompositionPtr decomp,
        ob::State* start_state,
        ob::State* goal_state,
        const std::vector<int>& region_path,
        size_t robot_index) override;

    std::string getName() const override { return "SyclopRRT"; }

private:

    // Collision manager for obstacles
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager_;

    /**
     * @brief Setup OMPL components for a specific robot
     *
     * Creates SpaceInformation with state validity checker and propagator,
     * and sets up the ProblemDefinition with start and goal states.
     */
    void setupOMPLComponents(
        std::shared_ptr<::Robot> robot,
        ob::State* start_state,
        ob::State* goal_state,
        oc::SpaceInformationPtr& si_out,
        ob::ProblemDefinitionPtr& pdef_out);

    /**
     * @brief Create lead computation function that returns the MAPF path
     *
     * This function is passed to SyclopRRT via setLeadComputeFn().
     * Instead of computing a lead via A*, it directly returns the
     * precomputed region path from MAPF.
     */
    oc::Syclop::LeadComputeFn createLeadFunction(
        const std::vector<int>& region_path);

    /**
     * @brief Configure SyclopRRT planner with parameters
     */
    void configurePlanner(oc::SyclopRRT& planner);
};

} // namespace mr_syclop

#endif // SYCLOP_RRT_SOLVER_H
