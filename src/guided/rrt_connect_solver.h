#ifndef RRT_CONNECT_SOLVER_H
#define RRT_CONNECT_SOLVER_H

#include "guided_planner_base.h"
#include <fcl/fcl.h>
#include <memory>

namespace mr_syclop {

/**
 * @brief RRTConnect-based guided planner implementation
 *
 * This class wraps OMPL's RRTConnect geometric planner to work with the
 * multi-robot framework. It plans in the robot's state space using
 * bidirectional RRT, then converts the geometric path into a PathControl
 * with trivial controls. Best suited for geometric (non-kinodynamic) planning.
 */
class RRTConnectSolver : public GuidedPlanner {
public:
    RRTConnectSolver(
        const GuidedPlannerConfig& config,
        std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager);

    ~RRTConnectSolver() override = default;

    GuidedPlanningResult solve(
        std::shared_ptr<::Robot> robot,
        oc::DecompositionPtr decomp,
        ob::State* start_state,
        ob::State* goal_state,
        const std::vector<int>& region_path,
        size_t robot_index) override;

    std::string getName() const override { return "RRTConnect"; }

private:
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager_;

    /**
     * @brief Convert a geometric path to a PathControl with trivial controls
     */
    std::shared_ptr<oc::PathControl> geometricToControl(
        const ob::PathPtr& geometric_path,
        const oc::SpaceInformationPtr& control_si);
};

} // namespace mr_syclop

#endif // RRT_CONNECT_SOLVER_H
