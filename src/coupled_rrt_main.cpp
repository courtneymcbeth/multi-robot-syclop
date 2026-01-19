/*********************************************************************
* Coupled Kinodynamic RRT - Executable Entry Point
*
* This file provides the command-line interface for the Coupled RRT planner,
* handling YAML I/O and converting to/from the planner's C++ API.
*********************************************************************/

#include "coupled_rrt.h"

// YAML
#include <yaml-cpp/yaml.h>

// Boost program options
#include <boost/program_options.hpp>

// OMPL random number generator
#include <ompl/util/RandomNumbers.h>

// Standard library
#include <iostream>
#include <fstream>
#include <string>

namespace po = boost::program_options;
namespace oc = ompl::control;
namespace ob = ompl::base;

// ============================================================================
// Helper Functions for YAML Conversion
// ============================================================================

PlanningProblem loadProblemFromYAML(const std::string& inputFile, std::vector<fcl::CollisionObjectf*>& obstacle_objects)
{
    std::cout << "Loading YAML file: " << inputFile << std::endl;
    YAML::Node env = YAML::LoadFile(inputFile);
    std::cout << "YAML loaded successfully" << std::endl;

    PlanningProblem problem;

    // Parse environment bounds
    const auto& env_min = env["environment"]["min"];
    const auto& env_max = env["environment"]["max"];
    problem.env_min = {env_min[0].as<double>(), env_min[1].as<double>()};
    problem.env_max = {env_max[0].as<double>(), env_max[1].as<double>()};

    // Parse obstacles and create FCL objects
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

                obstacle_objects.push_back(co);
                problem.obstacles.push_back(co);
            }
        }
    }

    // Parse robots
    std::cout << "Creating robots..." << std::endl;
    for (const auto& robot_node : env["robots"]) {
        RobotSpec robot;
        robot.type = robot_node["type"].as<std::string>();
        std::cout << "  Robot type: " << robot.type << std::endl;

        // Parse start state
        const auto& start_vec = robot_node["start"];
        for (size_t i = 0; i < start_vec.size(); ++i) {
            robot.start.push_back(start_vec[i].as<double>());
        }

        // Parse goal state
        const auto& goal_vec = robot_node["goal"];
        for (size_t i = 0; i < goal_vec.size(); ++i) {
            robot.goal.push_back(goal_vec[i].as<double>());
        }

        problem.robots.push_back(robot);
    }

    std::cout << "Planning for " << problem.robots.size() << " robots" << std::endl;

    return problem;
}

CoupledRRTConfig loadConfigFromYAML(const std::string& configFile)
{
    CoupledRRTConfig config;

    try {
        YAML::Node cfg = YAML::LoadFile(configFile);
        if (cfg["goal_threshold"]) {
            config.goal_threshold = cfg["goal_threshold"].as<double>();
        }
        if (cfg["min_control_duration"]) {
            config.min_control_duration = cfg["min_control_duration"].as<int>();
        }
        if (cfg["max_control_duration"]) {
            config.max_control_duration = cfg["max_control_duration"].as<int>();
        }
        if (cfg["seed"]) {
            config.seed = cfg["seed"].as<int>();
        }
    } catch (const YAML::Exception& e) {
        std::cerr << "ERROR loading config file: " << e.what() << std::endl;
        throw;
    }

    return config;
}

void writeResultToYAML(const std::string& outputFile,
                       const PlanningResult& result,
                       const PlanningProblem& problem)
{
    YAML::Node output;
    output["solved"] = result.solved;
    output["planning_time"] = result.planning_time;

    if (result.solved && result.path) {
        std::cout << "Exact solution found! Extracting path..." << std::endl;

        auto path = result.path;
        const int num_robots = problem.robots.size();

        std::cout << "Path has " << path->getStateCount() << " states and "
                  << path->getControlCount() << " controls" << std::endl;

        // Get the control space information (cast from base to control)
        auto si = std::static_pointer_cast<oc::SpaceInformation>(path->getSpaceInformation());
        auto compound_state_space = si->getStateSpace()->as<ob::CompoundStateSpace>();
        auto compound_control_space = si->getControlSpace()->as<oc::CompoundControlSpace>();

        // Extract states and controls for each robot
        YAML::Node result_node;
        for (int r = 0; r < num_robots; ++r) {
            YAML::Node robot_data;

            // Extract states
            YAML::Node states_node;
            for (size_t i = 0; i < path->getStateCount(); ++i) {
                auto compound = path->getState(i)->as<ob::CompoundState>();
                auto robot_state = compound->components[r];

                // Get state space for this robot to extract values
                auto robot_space = compound_state_space->getSubspace(r);

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

                auto robot_control_space = compound_control_space->getSubspace(r);
                const size_t dim = robot_control_space->getDimension();

                YAML::Node action_node;
                for (size_t d = 0; d < dim; ++d) {
                    double* address = robot_control_space->getValueAddressAtIndex(robot_control, d);
                    action_node.push_back(*address);
                }
                actions_node.push_back(action_node);
            }
            robot_data["actions"] = actions_node;

            result_node.push_back(robot_data);
        }
        output["result"] = result_node;

        std::cout << "Solution extracted successfully" << std::endl;
    } else {
        std::cout << "No solution found" << std::endl;
    }

    // Write output file
    std::ofstream fout(outputFile);
    fout << output;
    fout.close();
    std::cout << "Output written to " << outputFile << std::endl;
}

// ============================================================================
// Main Function
// ============================================================================

int main(int argc, char** argv)
{
    // Parse command line arguments
    std::string inputFile;
    std::string outputFile;
    std::string configFile;
    double timelimit = 60.0;

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

    // Vector to hold obstacle objects for cleanup
    std::vector<fcl::CollisionObjectf*> obstacle_objects;

    try {
        // Load configuration
        CoupledRRTConfig config;
        config.time_limit = timelimit;

        if (vm.count("cfg")) {
            config = loadConfigFromYAML(configFile);
            config.time_limit = timelimit; // Command line overrides config file
        }

        // Set the random seed
        if (config.seed >= 0) {
            std::cout << "Setting random seed to: " << config.seed << std::endl;
            ompl::RNG::setSeed(config.seed);
        } else {
            std::cout << "Using random seed" << std::endl;
        }

        // Load problem from YAML
        PlanningProblem problem = loadProblemFromYAML(inputFile, obstacle_objects);

        // Create planner
        CoupledRRTPlanner planner(config);

        // Plan
        std::cout << "Planner configured. Starting search..." << std::endl;
        PlanningResult result = planner.plan(problem);

        std::cout << "Planning completed in " << result.planning_time << " seconds" << std::endl;
        std::cout << "Planner status: " << (result.solved ? "Exact solution" : "No solution") << std::endl;

        // Write output
        writeResultToYAML(outputFile, result, problem);

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;

        // Clean up obstacles
        for (auto* co : obstacle_objects) {
            delete co;
        }

        return 1;
    }

    // Clean up obstacles
    for (auto* co : obstacle_objects) {
        delete co;
    }

    return 0;
}
