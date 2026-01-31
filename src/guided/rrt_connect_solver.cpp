#include "rrt_connect_solver.h"
#include "robots.h"
#include "fclStateValidityChecker.hpp"
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <iostream>
#include <chrono>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

namespace mr_syclop {

RRTConnectSolver::RRTConnectSolver(
    const GuidedPlannerConfig& config,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager)
    : GuidedPlanner(config)
    , collision_manager_(collision_manager)
{
}

GuidedPlanningResult RRTConnectSolver::solve(
    std::shared_ptr<::Robot> robot,
    oc::DecompositionPtr /*decomp*/,
    ob::State* start_state,
    ob::State* goal_state,
    const std::vector<int>& /*region_path*/,
    size_t robot_index)
{
    GuidedPlanningResult result;
    result.robot_index = robot_index;
    result.success = false;

    auto start_time = std::chrono::steady_clock::now();

    try {
        // Get the control-space SI from the robot
        auto control_si = robot->getSpaceInformation();
        auto state_space = control_si->getStateSpace();

        // Create a geometric SpaceInformation over the same state space
        auto geom_si = std::make_shared<ob::SpaceInformation>(state_space);

        // Set state validity checker (robot-obstacle collisions)
        auto validity_checker = std::make_shared<fclStateValidityChecker>(
            geom_si, collision_manager_, robot, false);
        geom_si->setStateValidityChecker(validity_checker);
        geom_si->setup();

        // Create problem definition
        auto pdef = std::make_shared<ob::ProblemDefinition>(geom_si);
        pdef->setStartAndGoalStates(start_state, goal_state, config_.goal_threshold);

        // Create and configure RRTConnect planner
        auto planner = std::make_shared<og::RRTConnect>(geom_si);
        planner->setProblemDefinition(pdef);
        planner->setup();

        // Solve with time limit
        ob::PlannerTerminationCondition ptc =
            ob::timedPlannerTerminationCondition(config_.time_per_robot);
        ob::PlannerStatus status = planner->solve(ptc);

        if (status == ob::PlannerStatus::EXACT_SOLUTION ||
            status == ob::PlannerStatus::APPROXIMATE_SOLUTION)
        {
            result.success = (status == ob::PlannerStatus::EXACT_SOLUTION);

            if (pdef->hasSolution() || pdef->hasApproximateSolution()) {
                ob::PathPtr path = pdef->getSolutionPath();
                result.path = geometricToControl(path, control_si);

#ifdef DBG_PRINTS
                std::cout << "Robot " << robot_index << " RRTConnect planning "
                          << (result.success ? "succeeded" : "found approximate solution")
                          << std::endl;
                if (result.path) {
                    std::cout << "  Path has " << result.path->getStateCount()
                              << " states" << std::endl;
                }
#endif
            }
        } else {
#ifdef DBG_PRINTS
            std::cout << "Robot " << robot_index << " RRTConnect planning failed"
                      << std::endl;
#endif
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception during RRTConnect planning for robot "
                  << robot_index << ": " << e.what() << std::endl;
        result.success = false;
    }

    auto end_time = std::chrono::steady_clock::now();
    result.planning_time = std::chrono::duration<double>(
        end_time - start_time).count();

    return result;
}

std::shared_ptr<oc::PathControl> RRTConnectSolver::geometricToControl(
    const ob::PathPtr& geometric_path,
    const oc::SpaceInformationPtr& control_si)
{
    auto geom_path = std::dynamic_pointer_cast<og::PathGeometric>(geometric_path);
    if (!geom_path || geom_path->getStateCount() == 0) {
        return nullptr;
    }

    auto control_path = std::make_shared<oc::PathControl>(control_si);

    // Create a zero control to use between waypoints
    auto control_space = control_si->getControlSpace();
    size_t control_dim = control_space->getDimension();

    for (size_t i = 0; i < geom_path->getStateCount(); ++i) {
        if (i < geom_path->getStateCount() - 1) {
            // Allocate a zero control
            oc::Control* ctrl = control_si->allocControl();
            auto* rv_ctrl = ctrl->as<oc::RealVectorControlSpace::ControlType>();
            for (size_t j = 0; j < control_dim; ++j) {
                rv_ctrl->values[j] = 0.0;
            }
            // Use unit duration per waypoint transition
            control_path->append(geom_path->getState(i), ctrl, 1.0);
        } else {
            // Last state, no control
            control_path->append(geom_path->getState(i));
        }
    }

    return control_path;
}

} // namespace mr_syclop
