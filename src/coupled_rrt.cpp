/*********************************************************************
* Coupled Kinodynamic RRT for Multi-Robot Planning
*
* This planner operates in the joint/compound planning space of multiple
* robots, using kinodynamic RRT with control sampling and propagation.
*********************************************************************/

#include "coupled_rrt.h"

// OMPL headers - Control (Kinodynamic)
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/StatePropagator.h>

// OMPL headers - Geometric
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/PathGeometric.h>

// OMPL headers - Base
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/PlannerTerminationCondition.h>

// FCL
#include <fcl/fcl.h>

// Standard library
#include <iostream>
#include <vector>
#include <memory>
#include <chrono>

// db-CBS robot dynamics
#include "../db-CBS/src/robots.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

// ============================================================================
// CompoundStatePropagator - propagates each robot independently
// ============================================================================

class CompoundStatePropagator : public oc::StatePropagator
{
public:
    CompoundStatePropagator(
        const oc::SpaceInformationPtr& si,
        const std::vector<std::shared_ptr<Robot>>& robots)
        : oc::StatePropagator(si), robots_(robots)
    {
    }

    void propagate(
        const ob::State* state,
        const oc::Control* control,
        double duration,
        ob::State* result) const override
    {
        auto compound_state = state->as<ob::CompoundState>();
        auto compound_control = control->as<oc::CompoundControl>();
        auto compound_result = result->as<ob::CompoundState>();

        // Propagate each robot independently using its dynamics
        for (size_t i = 0; i < robots_.size(); ++i) {
            robots_[i]->propagate(
                compound_state->components[i],
                compound_control->components[i],
                duration,
                compound_result->components[i]
            );
        }
    }

private:
    std::vector<std::shared_ptr<Robot>> robots_;
};

// ============================================================================
// CompoundStateValidityChecker - checks robot-obstacle and robot-robot collisions
// ============================================================================

class CompoundStateValidityChecker : public ob::StateValidityChecker
{
public:
    CompoundStateValidityChecker(
        const ob::SpaceInformationPtr& si,
        const std::shared_ptr<fcl::BroadPhaseCollisionManagerf>& col_mng_environment,
        const std::vector<std::shared_ptr<Robot>>& robots)
        : ob::StateValidityChecker(si),
          col_mng_environment_(col_mng_environment),
          robots_(robots)
    {
    }

    bool isValid(const ob::State* state) const override
    {
        // Check bounds
        if (!si_->satisfiesBounds(state)) {
            return false;
        }

        auto compound = state->as<ob::CompoundState>();

        // Check each robot against obstacles
        for (size_t i = 0; i < robots_.size(); ++i) {
            for (size_t part = 0; part < robots_[i]->numParts(); ++part) {
                const auto& transform = robots_[i]->getTransform(
                    compound->components[i], part);

                fcl::CollisionObjectf robot_co(robots_[i]->getCollisionGeometry(part));
                robot_co.setTranslation(transform.translation());
                robot_co.setRotation(transform.rotation());
                robot_co.computeAABB();

                fcl::DefaultCollisionData<float> collision_data;
                col_mng_environment_->collide(&robot_co, &collision_data,
                    fcl::DefaultCollisionFunction<float>);

                if (collision_data.result.isCollision()) {
                    return false;
                }
            }
        }

        // Check robot-robot collisions
        for (size_t i = 0; i < robots_.size(); ++i) {
            for (size_t j = i + 1; j < robots_.size(); ++j) {
                if (checkRobotRobotCollision(
                    compound->components[i],
                    compound->components[j],
                    robots_[i],
                    robots_[j])) {
                    return false;
                }
            }
        }

        return true;
    }

private:
    bool checkRobotRobotCollision(
        const ob::State* state_i,
        const ob::State* state_j,
        const std::shared_ptr<Robot>& robot_i,
        const std::shared_ptr<Robot>& robot_j) const
    {
        // Check all parts of robot i against all parts of robot j
        for (size_t part_i = 0; part_i < robot_i->numParts(); ++part_i) {
            for (size_t part_j = 0; part_j < robot_j->numParts(); ++part_j) {
                const auto& transform_i = robot_i->getTransform(state_i, part_i);
                const auto& transform_j = robot_j->getTransform(state_j, part_j);

                fcl::CollisionObjectf co_i(robot_i->getCollisionGeometry(part_i));
                co_i.setTranslation(transform_i.translation());
                co_i.setRotation(transform_i.rotation());
                co_i.computeAABB();

                fcl::CollisionObjectf co_j(robot_j->getCollisionGeometry(part_j));
                co_j.setTranslation(transform_j.translation());
                co_j.setRotation(transform_j.rotation());
                co_j.computeAABB();

                fcl::CollisionRequestf request;
                fcl::CollisionResultf result;
                fcl::collide(&co_i, &co_j, request, result);

                if (result.isCollision()) {
                    return true;
                }
            }
        }
        return false;
    }

    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_environment_;
    std::vector<std::shared_ptr<Robot>> robots_;
};

// ============================================================================
// CoupledRRTPlanner Implementation
// ============================================================================

CoupledRRTPlanner::CoupledRRTPlanner(const CoupledRRTConfig& config)
    : config_(config), position_bounds_(2)
{
}

CoupledRRTPlanner::~CoupledRRTPlanner()
{
    cleanup();
}

void CoupledRRTPlanner::cleanup()
{
    // Note: We don't delete obstacle objects since they're owned by the caller
    obstacles_.clear();

    // Clean up start/goal states
    if (!start_states_.empty() && !robots_.empty()) {
        for (size_t i = 0; i < start_states_.size(); ++i) {
            if (start_states_[i] && i < robots_.size()) {
                robots_[i]->getSpaceInformation()->getStateSpace()->freeState(start_states_[i]);
            }
        }
        start_states_.clear();
    }

    if (!goal_states_.empty() && !robots_.empty()) {
        for (size_t i = 0; i < goal_states_.size(); ++i) {
            if (goal_states_[i] && i < robots_.size()) {
                robots_[i]->getSpaceInformation()->getStateSpace()->freeState(goal_states_[i]);
            }
        }
        goal_states_.clear();
    }

    // Clear other state
    robots_.clear();
    col_mng_environment_.reset();
    compound_state_space_.reset();
    compound_control_space_.reset();
    compound_si_.reset();
    pdef_.reset();
}

void CoupledRRTPlanner::setupEnvironment(const PlanningProblem& problem)
{
    // Set up position bounds
    position_bounds_.setLow(0, problem.env_min[0]);
    position_bounds_.setLow(1, problem.env_min[1]);
    position_bounds_.setHigh(0, problem.env_max[0]);
    position_bounds_.setHigh(1, problem.env_max[1]);

    // Create FCL collision manager and register existing obstacle objects
    col_mng_environment_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();

    for (auto* obstacle : problem.obstacles) {
        col_mng_environment_->registerObject(obstacle);
    }

    if (!problem.obstacles.empty()) {
        col_mng_environment_->setup();
    }
}

void CoupledRRTPlanner::setupRobots(const PlanningProblem& problem)
{
    for (const auto& robot_spec : problem.robots) {
        // Create robot
        auto robot = create_robot(robot_spec.type, position_bounds_);
        robots_.push_back(robot);

        // Get robot's space information
        auto robot_si = robot->getSpaceInformation();
        auto robot_state_space = robot_si->getStateSpace();

        // Parse start state
        auto start_state = robot_state_space->allocState();
        auto start_se2 = start_state->as<ob::SE2StateSpace::StateType>();
        start_se2->setX(robot_spec.start[0]);
        start_se2->setY(robot_spec.start[1]);
        start_se2->setYaw(robot_spec.start.size() > 2 ? robot_spec.start[2] : 0.0);
        start_states_.push_back(start_state);

        // Parse goal state
        auto goal_state = robot_state_space->allocState();
        auto goal_se2 = goal_state->as<ob::SE2StateSpace::StateType>();
        goal_se2->setX(robot_spec.goal[0]);
        goal_se2->setY(robot_spec.goal[1]);
        goal_se2->setYaw(robot_spec.goal.size() > 2 ? robot_spec.goal[2] : 0.0);
        goal_states_.push_back(goal_state);
    }
}

void CoupledRRTPlanner::setupCompoundSpaces()
{
    // Create compound state space (used in both geometric and kinodynamic modes)
    compound_state_space_ = std::make_shared<ob::CompoundStateSpace>();
    for (auto& robot : robots_) {
        compound_state_space_->addSubspace(
            robot->getSpaceInformation()->getStateSpace(), 1.0);
    }

    if (config_.use_geometric) {
        // ===== GEOMETRIC MODE =====
        // Create geometric SpaceInformation (state-only, no control space)
        compound_si_ = std::make_shared<ob::SpaceInformation>(compound_state_space_);

        // Set state validity checker
        auto validity_checker = std::make_shared<CompoundStateValidityChecker>(
            compound_si_, col_mng_environment_, robots_);
        compound_si_->setStateValidityChecker(validity_checker);

        // Setup SpaceInformation
        compound_si_->setup();

    } else {
        // ===== KINODYNAMIC MODE (default) =====
        // Create compound control space
        compound_control_space_ = std::make_shared<oc::CompoundControlSpace>(compound_state_space_);
        for (auto& robot : robots_) {
            compound_control_space_->addSubspace(
                robot->getSpaceInformation()->getControlSpace());
        }

        // Create kinodynamic SpaceInformation (state + control)
        auto compound_si_control = std::make_shared<oc::SpaceInformation>(
            compound_state_space_, compound_control_space_);
        compound_si_ = compound_si_control;  // Store as base type

        // Set state propagator
        auto propagator = std::make_shared<CompoundStatePropagator>(compound_si_control, robots_);
        compound_si_control->setStatePropagator(propagator);

        // Set state validity checker
        auto validity_checker = std::make_shared<CompoundStateValidityChecker>(
            compound_si_, col_mng_environment_, robots_);
        compound_si_->setStateValidityChecker(validity_checker);

        // Set propagation step size (use minimum dt from all robots)
        double min_dt = robots_[0]->dt();
        for (auto& robot : robots_) {
            min_dt = std::min(static_cast<double>(min_dt), static_cast<double>(robot->dt()));
        }
        compound_si_control->setPropagationStepSize(min_dt);
        compound_si_control->setMinMaxControlDuration(
            config_.min_control_duration,
            config_.max_control_duration);

        // Setup SpaceInformation
        compound_si_->setup();
    }
}

void CoupledRRTPlanner::setupProblemDefinition()
{
    // Create problem definition
    pdef_ = std::make_shared<ob::ProblemDefinition>(compound_si_);

    // Combine individual start states into compound start
    auto compound_start = compound_si_->allocState();
    auto cs = compound_start->as<ob::CompoundState>();
    for (size_t i = 0; i < robots_.size(); ++i) {
        auto individual_space = robots_[i]->getSpaceInformation()->getStateSpace();
        individual_space->copyState(cs->components[i], start_states_[i]);
    }
    pdef_->addStartState(compound_start);

    // Combine individual goal states into compound goal
    auto compound_goal = compound_si_->allocState();
    auto cg = compound_goal->as<ob::CompoundState>();
    for (size_t i = 0; i < robots_.size(); ++i) {
        auto individual_space = robots_[i]->getSpaceInformation()->getStateSpace();
        individual_space->copyState(cg->components[i], goal_states_[i]);
    }
    auto goal = std::make_shared<MultiRobotGoalState>(compound_si_);
    goal->setState(compound_goal);
    goal->setThreshold(config_.goal_threshold);
    pdef_->setGoal(goal);
}

PlanningResult CoupledRRTPlanner::plan(const PlanningProblem& problem)
{
    // Clean up any previous planning state
    cleanup();

    PlanningResult result;
    result.solved = false;
    result.planning_time = 0.0;

    try {
        // Setup environment
        setupEnvironment(problem);

        // Setup robots
        setupRobots(problem);

        // Setup compound spaces
        setupCompoundSpaces();

        // Setup problem definition
        setupProblemDefinition();

        // Create planner based on mode
        ob::PlannerPtr planner;
        if (config_.use_geometric) {
            // Geometric planner
            planner = std::make_shared<og::RRTConnect>(compound_si_);
        } else {
            // Kinodynamic planner
            auto compound_si_control = std::static_pointer_cast<oc::SpaceInformation>(compound_si_);
            planner = std::make_shared<oc::RRT>(compound_si_control);
        }
        planner->setProblemDefinition(pdef_);
        planner->setup();

        // Solve
        auto start_time = std::chrono::steady_clock::now();
        ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(config_.time_limit);
        ob::PlannerStatus solved = planner->solve(ptc);
        auto end_time = std::chrono::steady_clock::now();

        result.planning_time = std::chrono::duration<double>(end_time - start_time).count();
        result.solved = (solved.asString() == "Exact solution");

        if (result.solved) {
            if (config_.use_geometric) {
                // Get geometric solution path
                auto path = pdef_->getSolutionPath()->as<og::PathGeometric>();

                // Interpolate to uniform steps
                path->interpolate();

                result.geometric_path = std::make_shared<og::PathGeometric>(*path);
            } else {
                // Get kinodynamic solution path
                auto path = pdef_->getSolutionPath()->as<oc::PathControl>();

                // Interpolate to uniform time steps
                path->interpolate();

                result.path = std::make_shared<oc::PathControl>(*path);
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "Planning failed with exception: " << e.what() << std::endl;
        result.solved = false;
    }

    return result;
}
