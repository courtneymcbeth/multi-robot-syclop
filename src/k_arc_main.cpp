/**
 * @file k_arc_main.cpp
 * @brief K-ARC: Kinodynamic Adaptive Robot Coordination - CLI Entry Point
 *
 * Command-line interface for the K-ARC planner, handling YAML I/O
 * and converting to/from the planner's C++ API.
 * Mirrors the pattern from arc_main.cpp.
 */

// Pinocchio/Crocoddyl must be included before Boost/OMPL headers
#include <dynoplan/optimization/ocp.hpp>

#include "k_arc.h"

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

// ============================================================================
// YAML Loading (reuses coupled_rrt.h PlanningProblem)
// ============================================================================

PlanningProblem loadProblemFromYAML(const std::string& inputFile,
                                   std::vector<fcl::CollisionObjectf*>& obstacle_objects)
{
    std::cout << "Loading YAML file: " << inputFile << std::endl;
    YAML::Node env = YAML::LoadFile(inputFile);

    PlanningProblem problem;

    // Parse environment bounds
    const auto& env_min = env["environment"]["min"];
    const auto& env_max = env["environment"]["max"];
    problem.env_min = {env_min[0].as<double>(), env_min[1].as<double>()};
    problem.env_max = {env_max[0].as<double>(), env_max[1].as<double>()};

    // Parse obstacles
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

        const auto& start_vec = robot_node["start"];
        for (size_t i = 0; i < start_vec.size(); ++i) {
            robot.start.push_back(start_vec[i].as<double>());
        }

        const auto& goal_vec = robot_node["goal"];
        for (size_t i = 0; i < goal_vec.size(); ++i) {
            robot.goal.push_back(goal_vec[i].as<double>());
        }

        problem.robots.push_back(robot);
    }

    std::cout << "Planning for " << problem.robots.size() << " robots" << std::endl;
    return problem;
}

KARCConfig loadConfigFromYAML(const std::string& configFile)
{
    YAML::Node cfg = YAML::LoadFile(configFile);
    return loadKARCConfig(cfg);
}

void writeResultToYAML(const std::string& outputFile,
                       const KARCResult& result,
                       const PlanningProblem& problem)
{
    YAML::Node output;
    output["solved"] = result.solved;
    output["planning_time"] = result.planning_time;
    output["num_segments"] = result.num_segments;
    output["num_conflicts_found"] = result.num_conflicts_found;
    output["num_conflicts_resolved"] = result.num_conflicts_resolved;
    output["failure_reason"] = result.failure_reason;

    // Methods used for conflict resolution
    if (!result.methods_used.empty()) {
        YAML::Node methods_node;
        for (const auto& method : result.methods_used) {
            methods_node.push_back(method);
        }
        output["methods_used"] = methods_node;
    }

    // Write trajectories
    if (result.solved && !result.trajectories.empty()) {
        std::cout << "Solution found! Extracting trajectories..." << std::endl;

        YAML::Node result_node;
        for (size_t r = 0; r < result.trajectories.size(); ++r) {
            YAML::Node robot_data;
            const auto& traj = result.trajectories[r];

            // States
            YAML::Node states_node;
            for (size_t i = 0; i < traj.states.size(); ++i) {
                YAML::Node state_node;
                for (int j = 0; j < traj.states[i].size(); ++j) {
                    state_node.push_back(traj.states[i](j));
                }
                states_node.push_back(state_node);
            }
            robot_data["states"] = states_node;

            // Actions
            YAML::Node actions_node;
            for (size_t i = 0; i < traj.actions.size(); ++i) {
                YAML::Node action_node;
                for (int j = 0; j < traj.actions[i].size(); ++j) {
                    action_node.push_back(traj.actions[i](j));
                }
                actions_node.push_back(action_node);
            }
            robot_data["actions"] = actions_node;

            std::cout << "  Robot " << r << ": " << traj.states.size()
                      << " states, " << traj.actions.size() << " actions" << std::endl;

            result_node.push_back(robot_data);
        }
        output["result"] = result_node;
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
// Main
// ============================================================================

int main(int argc, char** argv)
{
    std::string inputFile;
    std::string outputFile;
    std::string configFile;
    double timelimit = 300.0;

    po::options_description desc("K-ARC: Kinodynamic Adaptive Robot Coordination");
    desc.add_options()
        ("help,h", "Show help message")
        ("input,i", po::value<std::string>(&inputFile)->required(), "Input YAML file")
        ("output,o", po::value<std::string>(&outputFile)->required(), "Output YAML file")
        ("cfg,c", po::value<std::string>(&configFile), "Configuration YAML file")
        ("timelimit,t", po::value<double>(&timelimit)->default_value(300.0), "Time limit in seconds");

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

    std::vector<fcl::CollisionObjectf*> obstacle_objects;

    try {
        // Load configuration
        KARCConfig config;
        config.total_time_limit = timelimit;

        if (vm.count("cfg")) {
            config = loadConfigFromYAML(configFile);
            config.total_time_limit = timelimit;  // Command line overrides
        }

        // Set the random seed
        if (config.seed >= 0) {
            std::cout << "Setting random seed to: " << config.seed << std::endl;
            ompl::RNG::setSeed(config.seed);
        } else {
            std::cout << "Using random seed" << std::endl;
        }

        // Print configuration
        std::cout << "\n=== K-ARC Configuration ===" << std::endl;
        std::cout << "Time limit: " << config.total_time_limit << " seconds" << std::endl;
        std::cout << "Num segments: " << config.num_segments << std::endl;
        std::cout << "Goal threshold: " << config.goal_threshold << std::endl;
        std::cout << "Segment planning time: " << config.segment_planning_time << "s" << std::endl;
        std::cout << "Prioritized timeout: " << config.prioritized_timeout << "s" << std::endl;
        std::cout << "Composite timeout: " << config.composite_timeout << "s" << std::endl;
        std::cout << "Max solver escalations: " << config.max_solver_escalations << std::endl;
        std::cout << "Models base path: " << config.models_base_path << std::endl;
        std::cout << "Motions file: " << config.motions_file << std::endl;
        std::cout << "Seed: " << config.seed << std::endl;
        std::cout << "============================\n" << std::endl;

        // Load problem
        PlanningProblem problem = loadProblemFromYAML(inputFile, obstacle_objects);

        // Create planner and plan
        KARCPlanner planner(config);

        std::cout << "\n=== Starting K-ARC Planner ===" << std::endl;
        KARCResult result = planner.plan(problem);

        // Print results
        std::cout << "\n=== K-ARC Results ===" << std::endl;
        std::cout << "Planning completed in " << result.planning_time << " seconds" << std::endl;
        std::cout << "Status: " << (result.solved ? "SOLVED" : "FAILED") << std::endl;
        std::cout << "Segments: " << result.num_segments << std::endl;
        std::cout << "Conflicts found: " << result.num_conflicts_found << std::endl;
        std::cout << "Conflicts resolved: " << result.num_conflicts_resolved << std::endl;

        if (!result.failure_reason.empty()) {
            std::cout << "Failure reason: " << result.failure_reason << std::endl;
        }

        if (!result.methods_used.empty()) {
            std::cout << "Resolution methods used:" << std::endl;
            for (size_t i = 0; i < result.methods_used.size(); ++i) {
                std::cout << "  " << i << ": " << result.methods_used[i] << std::endl;
            }
        }
        std::cout << "=====================\n" << std::endl;

        // Write output
        writeResultToYAML(outputFile, result, problem);

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        for (auto* co : obstacle_objects) delete co;
        return 1;
    }

    // Cleanup
    for (auto* co : obstacle_objects) delete co;

    return 0;
}
