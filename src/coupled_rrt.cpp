/*********************************************************************
* Coupled Kinodynamic RRT for Multi-Robot Planning
*
* This planner operates in the joint/compound planning space of multiple
* robots, using kinodynamic RRT with control sampling and propagation.
*********************************************************************/

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
namespace po = boost::program_options;

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
        } catch (const YAML::Exception& e) {
            std::cerr << "ERROR loading config file: " << e.what() << std::endl;
            return 1;
        }
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

    // Create robots from YAML
    std::cout << "Creating robots..." << std::endl;
    std::vector<std::shared_ptr<Robot>> robots;
    std::vector<ob::State*> start_states;
    std::vector<ob::State*> goal_states;

    for (const auto& robot_node : env["robots"]) {
        auto robotType = robot_node["type"].as<std::string>();
        std::cout << "  Creating robot of type: " << robotType << std::endl;
        auto robot = create_robot(robotType, position_bounds);
        std::cout << "  Robot created" << std::endl;
        robots.push_back(robot);

        // Get robot's space information
        std::cout << "  Getting robot space information..." << std::endl;
        auto robot_si = robot->getSpaceInformation();
        auto robot_state_space = robot_si->getStateSpace();
        std::cout << "  Robot state space dimension: " << robot_state_space->getDimension() << std::endl;

        // Parse start state
        std::cout << "  Parsing start state..." << std::endl;
        const auto& start_vec = robot_node["start"];
        auto start_state = robot_state_space->allocState();

        // Manually set state values based on robot type
        // For now, assume all robots use SE2 state space (x, y, theta)
        // TODO: Handle other robot types if needed
        auto start_se2 = start_state->as<ob::SE2StateSpace::StateType>();
        start_se2->setX(start_vec[0].as<double>());
        start_se2->setY(start_vec[1].as<double>());
        start_se2->setYaw(start_vec.size() > 2 ? start_vec[2].as<double>() : 0.0);
        start_states.push_back(start_state);

        // Parse goal state
        std::cout << "  Parsing goal state..." << std::endl;
        const auto& goal_vec = robot_node["goal"];
        auto goal_state = robot_state_space->allocState();

        // Manually set state values based on robot type
        auto goal_se2 = goal_state->as<ob::SE2StateSpace::StateType>();
        goal_se2->setX(goal_vec[0].as<double>());
        goal_se2->setY(goal_vec[1].as<double>());
        goal_se2->setYaw(goal_vec.size() > 2 ? goal_vec[2].as<double>() : 0.0);
        goal_states.push_back(goal_state);
    }

    const int num_robots = robots.size();
    std::cout << "Planning for " << num_robots << " robots" << std::endl;

    // Create compound state space
    auto compound_state_space = std::make_shared<ob::CompoundStateSpace>();
    for (auto& robot : robots) {
        compound_state_space->addSubspace(
            robot->getSpaceInformation()->getStateSpace(), 1.0);
    }

    // Create compound control space
    auto compound_control_space = std::make_shared<oc::CompoundControlSpace>(compound_state_space);
    for (auto& robot : robots) {
        compound_control_space->addSubspace(
            robot->getSpaceInformation()->getControlSpace());
    }

    // Create compound SpaceInformation
    auto compound_si = std::make_shared<oc::SpaceInformation>(
        compound_state_space, compound_control_space);

    // Set state propagator
    auto propagator = std::make_shared<CompoundStatePropagator>(compound_si, robots);
    compound_si->setStatePropagator(propagator);

    // Set state validity checker
    auto validity_checker = std::make_shared<CompoundStateValidityChecker>(
        compound_si, col_mng_environment, robots);
    compound_si->setStateValidityChecker(validity_checker);

    // Set propagation step size (use minimum dt from all robots)
    double min_dt = robots[0]->dt();
    for (auto& robot : robots) {
        min_dt = std::min((double)min_dt, (double)robot->dt());
    }
    compound_si->setPropagationStepSize(min_dt);
    compound_si->setMinMaxControlDuration(min_control_duration, max_control_duration);

    // Setup SpaceInformation
    compound_si->setup();

    std::cout << "State space dimension: " << compound_state_space->getDimension() << std::endl;
    std::cout << "Control space dimension: " << compound_control_space->getDimension() << std::endl;
    std::cout << "Propagation step size: " << min_dt << std::endl;

    // Create problem definition
    auto pdef = std::make_shared<ob::ProblemDefinition>(compound_si);

    // Combine individual start states into compound start
    auto compound_start = compound_si->allocState();
    auto cs = compound_start->as<ob::CompoundState>();
    for (size_t i = 0; i < robots.size(); ++i) {
        auto individual_space = robots[i]->getSpaceInformation()->getStateSpace();
        individual_space->copyState(cs->components[i], start_states[i]);
    }
    pdef->addStartState(compound_start);

    // Combine individual goal states into compound goal
    auto compound_goal = compound_si->allocState();
    auto cg = compound_goal->as<ob::CompoundState>();
    for (size_t i = 0; i < robots.size(); ++i) {
        auto individual_space = robots[i]->getSpaceInformation()->getStateSpace();
        individual_space->copyState(cg->components[i], goal_states[i]);
    }
    auto goal = std::make_shared<MultiRobotGoalState>(compound_si);
    goal->setState(compound_goal);
    goal->setThreshold(goal_threshold);
    pdef->setGoal(goal);

    // Create planner
    auto planner = std::make_shared<oc::RRT>(compound_si);
    planner->setProblemDefinition(pdef);
    planner->setup();

    std::cout << "Planner configured. Starting search..." << std::endl;

    // Solve
    auto start_time = std::chrono::steady_clock::now();
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(timelimit);
    ob::PlannerStatus solved = planner->solve(ptc);
    auto end_time = std::chrono::steady_clock::now();
    double planning_time = std::chrono::duration<double>(end_time - start_time).count();

    std::cout << "Planning completed in " << planning_time << " seconds" << std::endl;
    std::cout << "Planner status: " << solved.asString() << std::endl;

    // Write output
    YAML::Node output;
    bool is_exact = (solved.asString() == "Exact solution");
    output["solved"] = is_exact;
    output["planning_time"] = planning_time;

    if (is_exact) {
        std::cout << "Exact solution found! Extracting path..." << std::endl;

        // Get solution path
        auto path = pdef->getSolutionPath()->as<oc::PathControl>();

        // Interpolate to uniform time steps
        path->interpolate();

        std::cout << "Path has " << path->getStateCount() << " states and "
                  << path->getControlCount() << " controls" << std::endl;

        // Extract states and controls for each robot
        YAML::Node result;
        for (int r = 0; r < num_robots; ++r) {
            YAML::Node robot_data;

            // Extract states
            YAML::Node states_node;
            for (size_t i = 0; i < path->getStateCount(); ++i) {
                auto compound = path->getState(i)->as<ob::CompoundState>();
                auto robot_state = compound->components[r];

                auto robot_space = robots[r]->getSpaceInformation()->getStateSpace();
                std::vector<double> state_vals(robot_space->getDimension());
                robot_space->copyToReals(state_vals, robot_state);

                YAML::Node state_node;
                for (double val : state_vals) {
                    state_node.push_back(val);
                }
                states_node.push_back(state_node);
            }
            robot_data["states"] = states_node;

            // Extract controls
            YAML::Node actions_node;
            for (size_t i = 0; i < path->getControlCount(); ++i) {
                auto compound_control = path->getControl(i)->as<oc::CompoundControl>();
                auto robot_control = compound_control->components[r];

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

            result.push_back(robot_data);
        }
        output["result"] = result;

        std::cout << "Solution extracted successfully" << std::endl;
    } else {
        std::cout << "No solution found (status: " << solved.asString() << ")" << std::endl;
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

    return 0;
}
