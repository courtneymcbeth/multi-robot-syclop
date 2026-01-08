/*********************************************************************
* Adaptive Robot Coordination (ARC) - Executable Entry Point
*
* This file provides the command-line interface for the ARC planner,
* handling YAML I/O and converting to/from the planner's C++ API.
*********************************************************************/

#include "arc.h"

// YAML
#include <yaml-cpp/yaml.h>

// Boost program options
#include <boost/program_options.hpp>

// Standard library
#include <iostream>
#include <fstream>
#include <string>

namespace po = boost::program_options;
namespace og = ompl::geometric;
namespace ob = ompl::base;

// ============================================================================
// Helper Functions for YAML Conversion
// ============================================================================

PlanningProblem loadProblemFromYAML(const std::string& inputFile,
                                   std::vector<fcl::CollisionObjectf*>& obstacle_objects)
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

ARCConfig loadConfigFromYAML(const std::string& configFile)
{
    ARCConfig config;

    try {
        YAML::Node cfg = YAML::LoadFile(configFile);

        // Basic planning parameters
        if (cfg["goal_threshold"]) {
            config.goal_threshold = cfg["goal_threshold"].as<double>();
        }

        // Subproblem parameters
        if (cfg["initial_time_window"]) {
            config.initial_time_window = cfg["initial_time_window"].as<int>();
        }
        if (cfg["time_window_expansion_step"]) {
            config.time_window_expansion_step = cfg["time_window_expansion_step"].as<int>();
        }
        if (cfg["max_subproblem_expansions"]) {
            config.max_subproblem_expansions = cfg["max_subproblem_expansions"].as<int>();
        }
        if (cfg["spatial_expansion_factor"]) {
            config.spatial_expansion_factor = cfg["spatial_expansion_factor"].as<double>();
        }

        // Method timeouts
        if (cfg["prioritized_query_timeout"]) {
            config.prioritized_query_timeout = cfg["prioritized_query_timeout"].as<double>();
        }
        if (cfg["decoupled_prm_timeout"]) {
            config.decoupled_prm_timeout = cfg["decoupled_prm_timeout"].as<double>();
        }
        if (cfg["composite_prm_timeout"]) {
            config.composite_prm_timeout = cfg["composite_prm_timeout"].as<double>();
        }

        // Discretization
        if (cfg["states_per_check"]) {
            config.states_per_check = cfg["states_per_check"].as<int>();
        }

        // Termination
        if (cfg["max_conflicts_resolved"]) {
            config.max_conflicts_resolved = cfg["max_conflicts_resolved"].as<int>();
        }

    } catch (const YAML::Exception& e) {
        std::cerr << "ERROR loading config file: " << e.what() << std::endl;
        throw;
    }

    return config;
}

void writeResultToYAML(const std::string& outputFile,
                      const ARCResult& result,
                      const PlanningProblem& problem)
{
    YAML::Node output;
    output["solved"] = result.solved;
    output["planning_time"] = result.planning_time;
    output["num_conflicts_found"] = result.num_conflicts_found;
    output["num_conflicts_resolved"] = result.num_conflicts_resolved;
    output["num_subproblems_created"] = result.num_subproblems_created;

    // Write statistics about subproblems
    if (!result.robots_per_subproblem.empty()) {
        YAML::Node subproblem_stats;
        for (size_t i = 0; i < result.robots_per_subproblem.size(); ++i) {
            YAML::Node sp;
            sp["robots"] = result.robots_per_subproblem[i];
            if (i < result.methods_used.size()) {
                sp["method"] = result.methods_used[i];
            }
            subproblem_stats.push_back(sp);
        }
        output["subproblem_stats"] = subproblem_stats;
    }

    if (result.solved && !result.paths.empty()) {
        std::cout << "Exact solution found! Extracting paths..." << std::endl;

        const int num_robots = problem.robots.size();
        std::cout << "Extracting paths for " << num_robots << " robots" << std::endl;

        YAML::Node result_node;
        for (int r = 0; r < num_robots; ++r) {
            YAML::Node robot_data;

            if (static_cast<size_t>(r) >= result.paths.size() || !result.paths[r]) {
                std::cout << "  Robot " << r << ": No path" << std::endl;
                result_node.push_back(robot_data);
                continue;
            }

            auto path = result.paths[r];
            std::cout << "  Robot " << r << ": " << path->getStateCount()
                     << " states" << std::endl;

            // Extract states
            YAML::Node states_node;
            for (size_t i = 0; i < path->getStateCount(); ++i) {
                const auto* state = path->getState(i);

                // Convert state to vector of doubles
                std::vector<double> state_vals;
                auto si = path->getSpaceInformation();
                si->getStateSpace()->copyToReals(state_vals, state);

                YAML::Node state_node;
                for (double val : state_vals) {
                    state_node.push_back(val);
                }
                states_node.push_back(state_node);
            }
            robot_data["states"] = states_node;

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
        ARCConfig config;
        config.time_limit = timelimit;

        if (vm.count("cfg")) {
            config = loadConfigFromYAML(configFile);
            config.time_limit = timelimit; // Command line overrides config file
        }

        // Print configuration
        std::cout << "\n=== ARC Configuration ===" << std::endl;
        std::cout << "Time limit: " << config.time_limit << " seconds" << std::endl;
        std::cout << "Goal threshold: " << config.goal_threshold << std::endl;
        std::cout << "Initial time window: " << config.initial_time_window << " timesteps" << std::endl;
        std::cout << "Max subproblem expansions: " << config.max_subproblem_expansions << std::endl;
        std::cout << "States per check: " << config.states_per_check << std::endl;
        std::cout << "=========================\n" << std::endl;

        // Load problem from YAML
        PlanningProblem problem = loadProblemFromYAML(inputFile, obstacle_objects);

        // Create planner
        ARCPlanner planner(config);

        // Plan
        std::cout << "\n=== Starting ARC Planner ===" << std::endl;
        ARCResult result = planner.plan(problem);

        std::cout << "\n=== Planning Results ===" << std::endl;
        std::cout << "Planning completed in " << result.planning_time << " seconds" << std::endl;
        std::cout << "Planner status: " << (result.solved ? "Exact solution" : "No solution") << std::endl;
        std::cout << "Conflicts found: " << result.num_conflicts_found << std::endl;
        std::cout << "Conflicts resolved: " << result.num_conflicts_resolved << std::endl;
        std::cout << "Subproblems created: " << result.num_subproblems_created << std::endl;

        if (!result.robots_per_subproblem.empty()) {
            std::cout << "\nSubproblem details:" << std::endl;
            for (size_t i = 0; i < result.robots_per_subproblem.size(); ++i) {
                std::cout << "  Subproblem " << i << ": "
                         << result.robots_per_subproblem[i] << " robots";
                if (i < result.methods_used.size()) {
                    std::cout << ", solved by " << result.methods_used[i];
                }
                std::cout << std::endl;
            }
        }
        std::cout << "=========================\n" << std::endl;

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
