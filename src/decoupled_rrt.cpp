/*********************************************************************
* Decoupled Kinodynamic RRT (Prioritized Planning) for Multi-Robot Planning
*
* This planner uses OMPL's Multi-Robot Prioritized Planning (PP) framework
* where robots are assigned priorities and planned sequentially. Lower
* priority robots treat higher priority robots as dynamic obstacles.
*********************************************************************/

// Multi-Robot OMPL headers
#include <ompl/multirobot/control/SpaceInformation.h>
#include <ompl/multirobot/base/ProblemDefinition.h>
#include <ompl/multirobot/control/planners/pp/PP.h>

// OMPL headers
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/goals/GoalRegion.h>

// FCL
#include <fcl/fcl.h>

// Standard library
#include <iostream>
#include <fstream>
#include <vector>
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
namespace omrb = ompl::multirobot::base;
namespace omrc = ompl::multirobot::control;
namespace po = boost::program_options;

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
        // Get the other robot's information
        // We need to extract the Robot object from the other SpaceInformation
        // For now, we'll do FCL-based collision checking directly

        // Check all parts of this robot against all parts of the other robot
        for (size_t part_i = 0; part_i < robot_->numParts(); ++part_i) {
            const auto& transform_i = robot_->getTransform(state1, part_i);

            fcl::CollisionObjectf co_i(robot_->getCollisionGeometry(part_i));
            co_i.setTranslation(transform_i.translation());
            co_i.setRotation(transform_i.rotation());
            co_i.computeAABB();

            // We need to get the other robot's geometry
            // Since we can't easily get the Robot object from state2.first,
            // we'll use a simple distance-based collision check as a fallback
            // This assumes all robots are similar in size

            // Extract positions from both states (assuming SE2 state space)
            const auto* se2_state1 = state1->as<ob::SE2StateSpace::StateType>();
            const auto* se2_state2 = state2.second->as<ob::SE2StateSpace::StateType>();

            const double* pos1 = se2_state1->as<ob::RealVectorStateSpace::StateType>(0)->values;
            const double* pos2 = se2_state2->as<ob::RealVectorStateSpace::StateType>(0)->values;

            double dist = sqrt(pow(pos1[0] - pos2[0], 2) + pow(pos1[1] - pos2[1], 2));

            // Use a conservative collision radius (sum of robot radii)
            // This is a simplification - ideally we'd get the actual geometry
            double collision_threshold = 0.4; // Conservative estimate for robot radius sum

            if (dist < collision_threshold) {
                return false; // Collision detected
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

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Show help message")
        ("input,i", po::value<std::string>(&inputFile)->required(), "Input YAML file")
        ("output,o", po::value<std::string>(&outputFile)->required(), "Output YAML file")
        ("cfg,c", po::value<std::string>(&configFile), "Configuration YAML file")
        ("timelimit,t", po::value<double>(&timelimit)->default_value(60.0), "Time limit in seconds");

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
        } catch (const YAML::Exception& e) {
            std::cerr << "ERROR loading config file: " << e.what() << std::endl;
            return 1;
        }
    }

    std::cout << "Decoupled RRT (Prioritized Planning) for Multi-Robot Systems" << std::endl;
    std::cout << "=============================================================" << std::endl;

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
    auto ma_si = std::make_shared<omrc::SpaceInformation>();
    auto ma_pdef = std::make_shared<omrb::ProblemDefinition>(ma_si);

    // Create robots from YAML
    std::cout << "Creating robots..." << std::endl;
    std::vector<std::shared_ptr<Robot>> robots;
    std::vector<ob::State*> start_states;
    std::vector<ob::State*> goal_states;
    int robot_idx = 0;

    for (const auto& robot_node : env["robots"]) {
        auto robotType = robot_node["type"].as<std::string>();
        std::cout << "  Robot " << robot_idx << " (" << robotType << ")" << std::endl;

        // Create robot
        auto robot = create_robot(robotType, position_bounds);
        robots.push_back(robot);

        // Get robot's control space information
        auto robot_si = robot->getSpaceInformation();
        auto robot_state_space = robot_si->getStateSpace();
        auto robot_control_space = robot_si->getControlSpace();

        // Set robot name
        std::string robot_name = "Robot " + std::to_string(robot_idx);
        robot_state_space->setName(robot_name);

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
        auto pdef = std::make_shared<ob::ProblemDefinition>(robot_si);
        pdef->addStartState(start_state);
        pdef->setGoal(std::make_shared<IndividualGoalCondition>(
            robot_si, goal_state, goal_threshold));

        // Add to multi-robot space information and problem definition
        ma_si->addIndividual(robot_si);
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

    // Set planner allocator
    ma_si->setPlannerAllocator(plannerAllocator);

    // Create PP planner
    auto planner = std::make_shared<omrc::PP>(ma_si);
    planner->setProblemDefinition(ma_pdef);

    std::cout << "Planner configured. Starting search..." << std::endl;
    std::cout << "  Goal threshold: " << goal_threshold << std::endl;
    std::cout << "  Propagation step size: " << propagation_step_size << std::endl;
    std::cout << "  Control duration: [" << min_control_duration << ", " << max_control_duration << "]" << std::endl;
    std::cout << "  Total time limit: " << timelimit << " seconds" << std::endl;

    // Solve
    auto start_time = std::chrono::steady_clock::now();
    bool solved = planner->as<omrb::Planner>()->solve(timelimit);
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

        // Get solution plan
        omrb::PlanPtr solution = ma_pdef->getSolutionPlan();
        auto control_plan = solution->as<omrc::PlanControl>();

        // Extract paths for each robot
        YAML::Node result;
        for (int r = 0; r < num_robots; ++r) {
            YAML::Node robot_data;

            // Get the path for this robot
            auto robot_path = control_plan->getPath(r);

            if (robot_path) {
                // Interpolate to get smoother path
                robot_path->interpolate();

                std::cout << "  Robot " << r << ": " << robot_path->getStateCount()
                          << " states, " << robot_path->getControlCount() << " controls" << std::endl;

                // Extract states
                YAML::Node states_node;
                for (size_t i = 0; i < robot_path->getStateCount(); ++i) {
                    const ob::State* robot_state = robot_path->getState(i);
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
        output["result"] = result;

        std::cout << "Solution extracted successfully" << std::endl;
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
    for (auto* co : obstacles) {
        delete co;
    }

    for (size_t i = 0; i < start_states.size(); ++i) {
        robots[i]->getSpaceInformation()->freeState(start_states[i]);
    }

    for (size_t i = 0; i < goal_states.size(); ++i) {
        robots[i]->getSpaceInformation()->freeState(goal_states[i]);
    }

    std::cout << "Done!" << std::endl;
    return 0;
}
