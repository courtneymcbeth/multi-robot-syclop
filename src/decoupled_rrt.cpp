/*********************************************************************
* Decoupled RRT (Prioritized Planning) for Multi-Robot Planning
*
* This planner uses OMPL's Multi-Robot Prioritized Planning (PP) framework
* where robots are assigned priorities and planned sequentially. Lower
* priority robots treat higher priority robots as dynamic obstacles.
*
* Supports both geometric and kinodynamic planning modes.
*********************************************************************/

// Multi-Robot OMPL headers
#include <ompl/multirobot/control/SpaceInformation.h>
#include <ompl/multirobot/base/ProblemDefinition.h>
#include <ompl/multirobot/control/planners/pp/PP.h>
#include <ompl/multirobot/geometric/planners/pp/PP.h>
#include <ompl/multirobot/geometric/PlanGeometric.h>

// OMPL headers - Control (Kinodynamic)
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/StatePropagator.h>

// OMPL headers - Geometric
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/PathGeometric.h>

// OMPL headers - Base
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/util/RandomNumbers.h>

// FCL
#include <fcl/fcl.h>

// Standard library
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <memory>
#include <chrono>

// YAML
#include <yaml-cpp/yaml.h>

// Boost
#include <boost/program_options.hpp>

// db-CBS robot dynamics
#include "../db-CBS/src/robots.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;
namespace omrb = ompl::multirobot::base;
namespace omrc = ompl::multirobot::control;
namespace omrg = ompl::multirobot::geometric;
namespace po = boost::program_options;

// ============================================================================
// Global registry to map SpaceInformation to Robot objects
// This is needed because areStatesValid() receives another robot's SpaceInformation
// but we need the Robot object to get its collision geometry
// ============================================================================

std::map<const ob::SpaceInformation*, std::shared_ptr<Robot>> g_robot_registry;

// ============================================================================
// IndividualStatePropagator - propagates a single robot using its dynamics
// ============================================================================

class IndividualStatePropagator : public oc::StatePropagator
{
public:
    IndividualStatePropagator(
        const oc::SpaceInformationPtr& si,
        const std::shared_ptr<Robot>& robot)
        : oc::StatePropagator(si), robot_(robot)
    {
    }

    void propagate(
        const ob::State* state,
        const oc::Control* control,
        double duration,
        ob::State* result) const override
    {
        robot_->propagate(state, control, duration, result);
    }

private:
    std::shared_ptr<Robot> robot_;
};

// ============================================================================
// IndividualStateValidityChecker - checks robot-obstacle and robot-robot collisions
// ============================================================================

class IndividualStateValidityChecker : public ob::StateValidityChecker
{
public:
    IndividualStateValidityChecker(
        const ob::SpaceInformationPtr& si,
        const std::shared_ptr<fcl::BroadPhaseCollisionManagerf>& col_mng_environment,
        const std::shared_ptr<Robot>& robot)
        : ob::StateValidityChecker(si),
          col_mng_environment_(col_mng_environment),
          robot_(robot)
    {
    }

    // Check if the robot at this state collides with static obstacles
    bool isValid(const ob::State* state) const override
    {
        // Check bounds
        if (!si_->satisfiesBounds(state)) {
            return false;
        }

        // Check each part of the robot against obstacles
        for (size_t part = 0; part < robot_->numParts(); ++part) {
            const auto& transform = robot_->getTransform(state, part);

            fcl::CollisionObjectf robot_co(robot_->getCollisionGeometry(part));
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

        return true;
    }

    // Check if this robot at state1 collides with another robot at state2
    // This is used for dynamic obstacle checking in prioritized planning
    bool areStatesValid(
        const ob::State* state1,
        const std::pair<const ob::SpaceInformationPtr, const ob::State*> state2) const override
    {
        // Look up the other robot from the global registry using its SpaceInformation
        auto it = g_robot_registry.find(state2.first.get());
        if (it == g_robot_registry.end()) {
            // Fallback: if we can't find the robot, log a warning and return true
            // This should not happen if the registry is properly populated
            std::cerr << "Warning: Could not find robot in registry for collision check" << std::endl;
            return true;
        }
        const std::shared_ptr<Robot>& other_robot = it->second;

        static int call_count = 0;
        call_count++;
        if (call_count % 100000 == 0) {
            std::cout << "areStatesValid called " << call_count << " times" << std::endl;
        }

        // Check all parts of this robot against all parts of the other robot using FCL
        for (size_t part_i = 0; part_i < robot_->numParts(); ++part_i) {
            const auto& transform_i = robot_->getTransform(state1, part_i);

            fcl::CollisionObjectf co_i(robot_->getCollisionGeometry(part_i));
            co_i.setTranslation(transform_i.translation());
            co_i.setRotation(transform_i.rotation());
            co_i.computeAABB();

            for (size_t part_j = 0; part_j < other_robot->numParts(); ++part_j) {
                const auto& transform_j = other_robot->getTransform(state2.second, part_j);

                fcl::CollisionObjectf co_j(other_robot->getCollisionGeometry(part_j));
                co_j.setTranslation(transform_j.translation());
                co_j.setRotation(transform_j.rotation());
                co_j.computeAABB();

                // Perform FCL collision check between the two collision objects
                fcl::CollisionRequestf request;
                fcl::CollisionResultf result;
                fcl::collide(&co_i, &co_j, request, result);

                if (result.isCollision()) {
                    return false; // Collision detected
                }
            }
        }

        return true; // No collision
    }

private:
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_environment_;
    std::shared_ptr<Robot> robot_;
};

// ============================================================================
// GoalCondition - goal region for each robot
// ============================================================================

class IndividualGoalCondition : public ob::GoalRegion
{
public:
    IndividualGoalCondition(
        const ob::SpaceInformationPtr& si,
        const ob::State* goal_state,
        double threshold)
        : ob::GoalRegion(si)
    {
        goal_state_ = si->allocState();
        si->copyState(goal_state_, goal_state);
        threshold_ = threshold;
    }

    ~IndividualGoalCondition()
    {
        si_->freeState(goal_state_);
    }

    double distanceGoal(const ob::State* st) const override
    {
        return si_->distance(st, goal_state_);
    }

private:
    ob::State* goal_state_;
};

// ============================================================================
// Planner allocator for PP
// ============================================================================

ob::PlannerPtr plannerAllocator(const ob::SpaceInformationPtr& si)
{
    const oc::SpaceInformationPtr siC = std::static_pointer_cast<oc::SpaceInformation>(si);
    auto planner = std::make_shared<oc::RRT>(siC);
    return planner;
}

// ============================================================================
// Main function
// ============================================================================

int main(int argc, char** argv)
{
    // Parse command line arguments
    std::string inputFile;
    std::string outputFile;
    std::string configFile;
    double timelimit = 60.0;
    double goal_threshold = 0.5;
    int min_control_duration = 1;
    int max_control_duration = 10;
    double propagation_step_size = 0.1;
    int seed = -1;
    bool use_geometric = false;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Show help message")
        ("input,i", po::value<std::string>(&inputFile)->required(), "Input YAML file")
        ("output,o", po::value<std::string>(&outputFile)->required(), "Output YAML file")
        ("cfg,c", po::value<std::string>(&configFile), "Configuration YAML file")
        ("timelimit,t", po::value<double>(&timelimit)->default_value(60.0), "Time limit in seconds")
        ("geometric,g", po::bool_switch(&use_geometric), "Use geometric planning (default: kinodynamic)");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }

        po::notify(vm);
    } catch (const po::error& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        std::cout << desc << std::endl;
        return 1;
    }

    // Load configuration file if provided
    if (vm.count("cfg")) {
        try {
            YAML::Node cfg = YAML::LoadFile(configFile);
            if (cfg["goal_threshold"]) {
                goal_threshold = cfg["goal_threshold"].as<double>();
            }
            if (cfg["min_control_duration"]) {
                min_control_duration = cfg["min_control_duration"].as<int>();
            }
            if (cfg["max_control_duration"]) {
                max_control_duration = cfg["max_control_duration"].as<int>();
            }
            if (cfg["propagation_step_size"]) {
                propagation_step_size = cfg["propagation_step_size"].as<double>();
            }
            if (cfg["seed"]) {
                seed = cfg["seed"].as<int>();
            }
            if (cfg["use_geometric"]) {
                use_geometric = cfg["use_geometric"].as<bool>();
            }
        } catch (const YAML::Exception& e) {
            std::cerr << "ERROR loading config file: " << e.what() << std::endl;
            return 1;
        }
    }

    std::cout << "Decoupled RRT (Prioritized Planning) for Multi-Robot Systems" << std::endl;
    std::cout << "Mode: " << (use_geometric ? "GEOMETRIC" : "KINODYNAMIC") << std::endl;
    std::cout << "=============================================================" << std::endl;

    // Set the random seed
    if (seed >= 0) {
        std::cout << "Setting random seed to: " << seed << std::endl;
        ompl::RNG::setSeed(seed);
    } else {
        std::cout << "Using random seed" << std::endl;
    }

    // Load YAML configuration
    std::cout << "Loading YAML file: " << inputFile << std::endl;
    YAML::Node env;
    try {
        env = YAML::LoadFile(inputFile);
    } catch (const YAML::Exception& e) {
        std::cerr << "ERROR loading YAML file: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "YAML loaded successfully" << std::endl;

    // Parse environment bounds
    const auto& env_min = env["environment"]["min"];
    const auto& env_max = env["environment"]["max"];

    ob::RealVectorBounds position_bounds(2);
    position_bounds.setLow(0, env_min[0].as<double>());
    position_bounds.setLow(1, env_min[1].as<double>());
    position_bounds.setHigh(0, env_max[0].as<double>());
    position_bounds.setHigh(1, env_max[1].as<double>());

    // Create FCL collision manager for obstacles
    auto col_mng_environment = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
    std::vector<fcl::CollisionObjectf*> obstacles;

    if (env["environment"]["obstacles"]) {
        for (const auto& obs : env["environment"]["obstacles"]) {
            if (obs["type"].as<std::string>() == "box") {
                const auto& size = obs["size"];
                const auto& center = obs["center"];

                auto box = std::make_shared<fcl::Boxf>(
                    size[0].as<float>(),
                    size[1].as<float>(),
                    1.0f);

                auto* co = new fcl::CollisionObjectf(box);
                co->setTranslation(fcl::Vector3f(
                    center[0].as<float>(),
                    center[1].as<float>(),
                    0.0f));
                co->computeAABB();

                obstacles.push_back(co);
                col_mng_environment->registerObject(co);
            }
        }
        col_mng_environment->setup();
    }

    std::cout << "Loaded " << obstacles.size() << " obstacles" << std::endl;

    // Create multi-robot space information and problem definition
    // The base pointer is used for the problem definition and geometric PP;
    // the control pointer is additionally needed for kinodynamic PP.
    std::shared_ptr<omrb::SpaceInformation> ma_si;
    std::shared_ptr<omrc::SpaceInformation> ma_si_control;
    if (use_geometric) {
        ma_si = std::make_shared<omrb::SpaceInformation>();
    } else {
        ma_si_control = std::make_shared<omrc::SpaceInformation>();
        ma_si = ma_si_control;
    }
    auto ma_pdef = std::make_shared<omrb::ProblemDefinition>(ma_si);

    // Create robots from YAML
    std::cout << "Creating robots..." << std::endl;
    std::vector<std::shared_ptr<Robot>> robots;
    std::vector<ob::State*> start_states;
    std::vector<ob::State*> goal_states;
    // Track the per-robot SI used for planning (geometric or kinodynamic)
    std::vector<ob::SpaceInformationPtr> robot_sis;
    int robot_idx = 0;

    for (const auto& robot_node : env["robots"]) {
        auto robotType = robot_node["type"].as<std::string>();
        std::cout << "  Robot " << robot_idx << " (" << robotType << ")" << std::endl;

        // Create robot
        auto robot = create_robot(robotType, position_bounds);
        robots.push_back(robot);

        // Get robot's state space (shared between both modes)
        auto robot_state_space = robot->getSpaceInformation()->getStateSpace();

        // Set robot name
        std::string robot_name = "Robot " + std::to_string(robot_idx);
        robot_state_space->setName(robot_name);

        // Create the per-robot SpaceInformation based on mode
        ob::SpaceInformationPtr individual_si;

        if (use_geometric) {
            // ===== GEOMETRIC MODE =====
            // Create geometric SpaceInformation (state-only, no control space)
            individual_si = std::make_shared<ob::SpaceInformation>(robot_state_space);

            // Set state validity checker with dynamic obstacle support
            auto validity_checker = std::make_shared<IndividualStateValidityChecker>(
                individual_si, col_mng_environment, robot);
            individual_si->setStateValidityChecker(validity_checker);

            // Setup the space information
            individual_si->setup();
        } else {
            // ===== KINODYNAMIC MODE (default) =====
            auto robot_si = robot->getSpaceInformation();
            individual_si = robot_si;

            // Set state propagator using the robot's dynamics
            auto propagator = std::make_shared<IndividualStatePropagator>(robot_si, robot);
            robot_si->setStatePropagator(propagator);

            // Set state validity checker with dynamic obstacle support
            auto validity_checker = std::make_shared<IndividualStateValidityChecker>(
                robot_si, col_mng_environment, robot);
            robot_si->setStateValidityChecker(validity_checker);

            // Set propagation parameters
            robot_si->setPropagationStepSize(propagation_step_size);
            robot_si->setMinMaxControlDuration(min_control_duration, max_control_duration);

            // Setup the space information
            robot_si->setup();
        }

        robot_sis.push_back(individual_si);

        // Register this robot in the global registry so areStatesValid() can look it up
        g_robot_registry[individual_si.get()] = robot;

        // Parse start state
        const auto& start_vec = robot_node["start"];
        auto start_state = robot_state_space->allocState();
        auto start_se2 = start_state->as<ob::SE2StateSpace::StateType>();
        start_se2->setX(start_vec[0].as<double>());
        start_se2->setY(start_vec[1].as<double>());
        start_se2->setYaw(start_vec.size() > 2 ? start_vec[2].as<double>() : 0.0);
        start_states.push_back(start_state);

        // Parse goal state
        const auto& goal_vec = robot_node["goal"];
        auto goal_state = robot_state_space->allocState();
        auto goal_se2 = goal_state->as<ob::SE2StateSpace::StateType>();
        goal_se2->setX(goal_vec[0].as<double>());
        goal_se2->setY(goal_vec[1].as<double>());
        goal_se2->setYaw(goal_vec.size() > 2 ? goal_vec[2].as<double>() : 0.0);
        goal_states.push_back(goal_state);

        // Create problem definition for this robot
        auto pdef = std::make_shared<ob::ProblemDefinition>(individual_si);
        pdef->addStartState(start_state);
        pdef->setGoal(std::make_shared<IndividualGoalCondition>(
            individual_si, goal_state, goal_threshold));

        // Add to multi-robot space information and problem definition
        if (use_geometric) {
            ma_si->addIndividual(individual_si);
        } else {
            ma_si_control->addIndividual(
                std::static_pointer_cast<oc::SpaceInformation>(individual_si));
        }
        ma_pdef->addIndividual(pdef);

        std::cout << "    Start: (" << start_se2->getX() << ", " << start_se2->getY() << ")" << std::endl;
        std::cout << "    Goal:  (" << goal_se2->getX() << ", " << goal_se2->getY() << ")" << std::endl;

        robot_idx++;
    }

    const int num_robots = robots.size();
    std::cout << "Planning for " << num_robots << " robots" << std::endl;

    // Lock the multi-robot structures
    ma_si->lock();
    ma_pdef->lock();

    std::cout << "Planner configured. Starting search..." << std::endl;
    std::cout << "  Goal threshold: " << goal_threshold << std::endl;
    if (!use_geometric) {
        std::cout << "  Propagation step size: " << propagation_step_size << std::endl;
        std::cout << "  Control duration: [" << min_control_duration << ", " << max_control_duration << "]" << std::endl;
    }
    std::cout << "  Total time limit: " << timelimit << " seconds" << std::endl;

    // Solve
    auto start_time = std::chrono::steady_clock::now();
    bool solved = false;

    // Paths produced by geometric PP (kept alive so dynamic obstacle pointers remain valid)
    std::vector<og::PathGeometricPtr> geometric_paths;

    if (use_geometric) {
        // ===== GEOMETRIC MODE: Manual PP loop =====
        // We implement the prioritized planning loop directly instead of using
        // omrg::PP because the library has a use-after-free bug: it stores raw
        // state pointers as dynamic obstacles from a path that is then destroyed.
        solved = true;
        auto ptc = ob::timedPlannerTerminationCondition(timelimit);
        for (int r = 0; r < num_robots; ++r) {
            auto rrt = std::make_shared<og::RRT>(robot_sis[r], true);
            rrt->setProblemDefinition(ma_pdef->getIndividual(r));
            bool robot_solved = rrt->solve(ptc);
            if (robot_solved) {
                auto path = std::make_shared<og::PathGeometric>(
                    *rrt->getProblemDefinition()->getSolutionPath()->as<og::PathGeometric>());
                geometric_paths.push_back(path);

                // Add path states as dynamic obstacles for subsequent robots,
                // cloning each state so pointers remain valid independently
                for (int r2 = r + 1; r2 < num_robots; ++r2) {
                    for (size_t t = 0; t < path->getStateCount(); ++t) {
                        auto cloned = robot_sis[r]->cloneState(path->getState(t));
                        ma_si->getIndividual(r2)->addDynamicObstacle(
                            static_cast<double>(t), robot_sis[r], cloned);
                    }
                }
            } else {
                solved = false;
                break;
            }
            rrt->clear();
        }
    } else {
        // ===== KINODYNAMIC MODE: Use omrc::PP =====
        ma_si_control->setPlannerAllocator(plannerAllocator);
        auto planner = std::make_shared<omrc::PP>(ma_si_control);
        planner->setProblemDefinition(ma_pdef);
        solved = planner->solve(ob::timedPlannerTerminationCondition(timelimit));
    }

    auto end_time = std::chrono::steady_clock::now();
    double planning_time = std::chrono::duration<double>(end_time - start_time).count();

    std::cout << "Planning completed in " << planning_time << " seconds" << std::endl;
    std::cout << "Solution found: " << (solved ? "YES" : "NO") << std::endl;

    // Write output
    YAML::Node output;
    output["solved"] = solved;
    output["planning_time"] = planning_time;

    if (solved) {
        std::cout << "Extracting solution paths..." << std::endl;

        // Get solution plan (only needed for kinodynamic mode)
        omrb::PlanPtr solution;
        if (!use_geometric) {
            solution = ma_pdef->getSolutionPlan();
        }

        // Collect per-robot state sequences for validation
        // Each inner vector holds pointers to the states along that robot's path
        std::vector<std::vector<const ob::State*>> all_robot_states(num_robots);

        // Extract paths for each robot
        YAML::Node result;

        if (use_geometric) {
            // ===== GEOMETRIC PATH EXTRACTION =====
            for (int r = 0; r < num_robots && r < static_cast<int>(geometric_paths.size()); ++r) {
                YAML::Node robot_data;

                auto& robot_path = geometric_paths[r];

                if (robot_path) {
                    robot_path->interpolate();

                    std::cout << "  Robot " << r << ": " << robot_path->getStateCount()
                              << " states" << std::endl;

                    // Extract states
                    YAML::Node states_node;
                    for (size_t i = 0; i < robot_path->getStateCount(); ++i) {
                        const ob::State* robot_state = robot_path->getState(i);
                        all_robot_states[r].push_back(robot_state);

                        const ob::SE2StateSpace::StateType* se2_state =
                            robot_state->as<ob::SE2StateSpace::StateType>();

                        YAML::Node state_node;
                        state_node.push_back(se2_state->getX());
                        state_node.push_back(se2_state->getY());
                        state_node.push_back(se2_state->getYaw());
                        states_node.push_back(state_node);
                    }
                    robot_data["states"] = states_node;
                }

                result.push_back(robot_data);
            }
        } else {
            // ===== KINODYNAMIC PATH EXTRACTION =====
            auto control_plan = solution->as<omrc::PlanControl>();

            for (int r = 0; r < num_robots; ++r) {
                YAML::Node robot_data;

                auto robot_path = control_plan->getPath(r);

                if (robot_path) {
                    robot_path->interpolate();

                    std::cout << "  Robot " << r << ": " << robot_path->getStateCount()
                              << " states, " << robot_path->getControlCount() << " controls" << std::endl;

                    // Extract states
                    YAML::Node states_node;
                    for (size_t i = 0; i < robot_path->getStateCount(); ++i) {
                        const ob::State* robot_state = robot_path->getState(i);
                        all_robot_states[r].push_back(robot_state);

                        const ob::SE2StateSpace::StateType* se2_state =
                            robot_state->as<ob::SE2StateSpace::StateType>();

                        YAML::Node state_node;
                        state_node.push_back(se2_state->getX());
                        state_node.push_back(se2_state->getY());
                        state_node.push_back(se2_state->getYaw());
                        states_node.push_back(state_node);
                    }
                    robot_data["states"] = states_node;

                    // Extract controls
                    YAML::Node actions_node;
                    for (size_t i = 0; i < robot_path->getControlCount(); ++i) {
                        oc::Control* robot_control = robot_path->getControl(i);
                        auto robot_control_space = robots[r]->getSpaceInformation()->getControlSpace();
                        const size_t dim = robot_control_space->getDimension();

                        YAML::Node action_node;
                        for (size_t d = 0; d < dim; ++d) {
                            double* address = robot_control_space->getValueAddressAtIndex(robot_control, d);
                            action_node.push_back(*address);
                        }
                        actions_node.push_back(action_node);
                    }
                    robot_data["actions"] = actions_node;
                }

                result.push_back(robot_data);
            }
        }

        output["result"] = result;
        std::cout << "Solution extracted successfully" << std::endl;

        // Validate no inter-robot collisions along the paths
        // This validation is the same for both geometric and kinodynamic modes
        // since we collected state pointers into all_robot_states above
        std::cout << "Validating solution for inter-robot collisions..." << std::endl;
        bool collision_free = true;
        int collision_robot1 = -1, collision_robot2 = -1;
        size_t collision_time = 0;

        // Find maximum number of states across all paths
        size_t max_states = 0;
        for (int r = 0; r < num_robots; ++r) {
            if (all_robot_states[r].size() > max_states) {
                max_states = all_robot_states[r].size();
            }
        }

        // Check all robot pairs at each time step
        for (size_t t = 0; t < max_states && collision_free; ++t) {
            for (int r1 = 0; r1 < num_robots && collision_free; ++r1) {
                if (all_robot_states[r1].empty()) continue;

                // Get state for robot r1 (clamp to last state if path ended)
                size_t idx1 = std::min(t, all_robot_states[r1].size() - 1);
                const ob::State* state1 = all_robot_states[r1][idx1];

                for (int r2 = r1 + 1; r2 < num_robots && collision_free; ++r2) {
                    if (all_robot_states[r2].empty()) continue;

                    // Get state for robot r2 (clamp to last state if path ended)
                    size_t idx2 = std::min(t, all_robot_states[r2].size() - 1);
                    const ob::State* state2 = all_robot_states[r2][idx2];

                    // Check collision between all parts of robot r1 and r2
                    for (size_t part_i = 0; part_i < robots[r1]->numParts() && collision_free; ++part_i) {
                        const auto& transform_i = robots[r1]->getTransform(state1, part_i);

                        fcl::CollisionObjectf co_i(robots[r1]->getCollisionGeometry(part_i));
                        co_i.setTranslation(transform_i.translation());
                        co_i.setRotation(transform_i.rotation());
                        co_i.computeAABB();

                        for (size_t part_j = 0; part_j < robots[r2]->numParts() && collision_free; ++part_j) {
                            const auto& transform_j = robots[r2]->getTransform(state2, part_j);

                            fcl::CollisionObjectf co_j(robots[r2]->getCollisionGeometry(part_j));
                            co_j.setTranslation(transform_j.translation());
                            co_j.setRotation(transform_j.rotation());
                            co_j.computeAABB();

                            fcl::CollisionRequestf request;
                            fcl::CollisionResultf result;
                            fcl::collide(&co_i, &co_j, request, result);

                            if (result.isCollision()) {
                                collision_free = false;
                                collision_robot1 = r1;
                                collision_robot2 = r2;
                                collision_time = t;
                            }
                        }
                    }
                }
            }
        }

        if (!collision_free) {
            std::cout << "COLLISION DETECTED: Robot " << collision_robot1
                      << " and Robot " << collision_robot2
                      << " at time step " << collision_time << std::endl;
            solved = false;
            output["solved"] = false;
            output["collision_detected"] = true;
            output["collision_robot1"] = collision_robot1;
            output["collision_robot2"] = collision_robot2;
            output["collision_time_step"] = static_cast<int>(collision_time);
        } else {
            std::cout << "Solution validated: no inter-robot collisions" << std::endl;
        }
    } else {
        std::cout << "No solution found within time limit" << std::endl;
    }

    // Write output file
    try {
        std::ofstream fout(outputFile);
        fout << output;
        fout.close();
        std::cout << "Output written to " << outputFile << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "ERROR writing output file: " << e.what() << std::endl;
        return 1;
    }

    // Cleanup
    g_robot_registry.clear();

    for (auto* co : obstacles) {
        delete co;
    }

    for (size_t i = 0; i < start_states.size(); ++i) {
        robot_sis[i]->freeState(start_states[i]);
    }

    for (size_t i = 0; i < goal_states.size(); ++i) {
        robot_sis[i]->freeState(goal_states[i]);
    }

    std::cout << "Done!" << std::endl;
    return 0;
}
