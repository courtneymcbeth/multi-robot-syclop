#include "syclop_rrt_solver.h"
#include "robots.h"
#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <iostream>
#include <chrono>

namespace mr_syclop {

SyclopRRTSolver::SyclopRRTSolver(
    const GuidedPlannerConfig& config,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager)
    : GuidedPlanner(config)
    , collision_manager_(collision_manager)
{
}

GuidedPlanningResult SyclopRRTSolver::solve(
    std::shared_ptr<::Robot> robot,
    oc::DecompositionPtr decomp,
    ob::State* start_state,
    ob::State* goal_state,
    const std::vector<int>& region_path,
    size_t robot_index)
{
    GuidedPlanningResult result;
    result.robot_index = robot_index;
    result.success = false;

#ifdef DBG_PRINTS
    std::cout << "Planning for robot " << robot_index
              << " with region path: ";
    for (int r : region_path) {
        std::cout << r << " ";
    }
    std::cout << std::endl;
#endif

    auto start_time = std::chrono::steady_clock::now();

    try {
        // 1. Setup OMPL components (SpaceInformation, ProblemDefinition)
        oc::SpaceInformationPtr si;
        ob::ProblemDefinitionPtr pdef;
        setupOMPLComponents(robot, start_state, goal_state, si, pdef);

        // 2. Create SyclopRRT planner
        auto planner = std::make_shared<oc::SyclopRRT>(si, decomp);

        // 3. Configure planner parameters
        configurePlanner(*planner);

        // 4. Set custom lead computation function using MAPF path
        auto leadFn = createLeadFunction(region_path);
        planner->setLeadComputeFn(leadFn);

        // 5. Setup and solve
        planner->setProblemDefinition(pdef);
        planner->setup();

        // Use exactSolnPlannerTerminationCondition to stop after finding first solution
        // Falls back to time limit if no solution found
        ob::PlannerTerminationCondition exactSolnPtc =
            ob::exactSolnPlannerTerminationCondition(pdef);
        ob::PlannerTerminationCondition timedPtc =
            ob::timedPlannerTerminationCondition(config_.time_per_robot);
        ob::PlannerTerminationCondition ptc(
            [&exactSolnPtc, &timedPtc] { return exactSolnPtc() || timedPtc(); });
        ob::PlannerStatus status = planner->solve(ptc);

        // 6. Extract solution if found
        if (status == ob::PlannerStatus::EXACT_SOLUTION ||
            status == ob::PlannerStatus::APPROXIMATE_SOLUTION)
        {
            result.success = (status == ob::PlannerStatus::EXACT_SOLUTION);

            // Extract path from problem definition
            if (pdef->hasApproximateSolution() || pdef->hasSolution()) {
                ob::PathPtr path = pdef->getSolutionPath();
                result.path = std::dynamic_pointer_cast<oc::PathControl>(path);

#ifdef DBG_PRINTS
                std::cout << "Robot " << robot_index << " planning "
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
            std::cout << "Robot " << robot_index << " planning failed"
                      << std::endl;
#endif
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception during SyclopRRT planning for robot "
                  << robot_index << ": " << e.what() << std::endl;
        result.success = false;
    }

    auto end_time = std::chrono::steady_clock::now();
    result.planning_time = std::chrono::duration<double>(
        end_time - start_time).count();

    return result;
}

void SyclopRRTSolver::setupOMPLComponents(
    std::shared_ptr<::Robot> robot,
    ob::State* start_state,
    ob::State* goal_state,
    oc::SpaceInformationPtr& si_out,
    ob::ProblemDefinitionPtr& pdef_out)
{
    // Get robot's SpaceInformation (already configured in MRSyCLoPPlanner)
    si_out = robot->getSpaceInformation();

    // Set state validity checker (robot-obstacle collisions only)
    // Cast control::SpaceInformationPtr to base::SpaceInformationPtr
    ob::SpaceInformationPtr base_si = std::static_pointer_cast<ob::SpaceInformation>(si_out);
    auto validity_checker = std::make_shared<fclStateValidityChecker>(
        base_si, collision_manager_, robot, false);
    si_out->setStateValidityChecker(validity_checker);

    // State propagator should already be set by MRSyCLoPPlanner
    // But if not, set it here:
    if (!si_out->getStatePropagator()) {
        si_out->setStatePropagator(
            std::make_shared<RobotStatePropagator>(si_out, robot));
    }

    si_out->setup();

    // Create problem definition
    pdef_out = std::make_shared<ob::ProblemDefinition>(si_out);
    pdef_out->setStartAndGoalStates(start_state, goal_state, config_.goal_threshold);
}

oc::Syclop::LeadComputeFn SyclopRRTSolver::createLeadFunction(
    const std::vector<int>& region_path)
{
    // Capture region_path by value in the lambda
    return [region_path](int startRegion, int goalRegion,
                         std::vector<int>& lead) {
        // The MAPF algorithm has already computed the high-level path
        // Simply return it as the lead
        lead = region_path;

#ifdef DBG_PRINTS
        // Validate that the path starts and ends correctly
        if (!lead.empty()) {
            if (lead.front() != startRegion || lead.back() != goalRegion) {
                std::cerr << "Warning: MAPF path does not match start/goal regions"
                          << std::endl;
                std::cerr << "  Expected start: " << startRegion
                          << ", got: " << lead.front() << std::endl;
                std::cerr << "  Expected goal: " << goalRegion
                          << ", got: " << lead.back() << std::endl;
            }
        }
#endif
    };
}

void SyclopRRTSolver::configurePlanner(oc::SyclopRRT& planner)
{
    // Set SyclopRRT parameters from config
    planner.setNumFreeVolumeSamples(config_.num_free_volume_samples);
    planner.setNumRegionExpansions(config_.num_region_expansions);
    planner.setNumTreeExpansions(config_.num_tree_expansions);
    planner.setProbAbandonLeadEarly(config_.prob_abandon_lead_early);
    planner.setProbShortestPathLead(config_.prob_shortest_path);
    planner.setRegionalNearestNeighbors(config_.use_regional_nn);
}

} // namespace mr_syclop
