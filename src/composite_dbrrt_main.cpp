/**
 * @file composite_dbrrt_main.cpp
 * @brief Main entry point for Composite DB-RRT Planner executable
 */

// Pinocchio/Crocoddyl must be included before Boost/OMPL headers
#include <dynoplan/optimization/ocp.hpp>

#include "composite_dbrrt.h"

#include <iostream>
#include <fstream>

#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>

#include <fcl/fcl.h>

namespace po = boost::program_options;

int main(int argc, char** argv) {
    std::string input_file;
    std::string output_file;
    std::string config_file;
    double time_limit = 60.0;

    po::options_description desc("Composite DB-RRT Multi-Robot Motion Planning");
    desc.add_options()
        ("help,h", "Show help message")
        ("input,i", po::value<std::string>(&input_file)->required(), "Input YAML file")
        ("output,o", po::value<std::string>(&output_file)->required(), "Output YAML file")
        ("cfg,c", po::value<std::string>(&config_file), "Configuration YAML file")
        ("timelimit,t", po::value<double>(&time_limit)->default_value(60.0), "Time limit in seconds");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }

        po::notify(vm);
    } catch (const po::error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << desc << std::endl;
        return 1;
    }

    // Load configuration
    CompositeDBRRTConfig config;
    config.time_limit = time_limit;

    if (vm.count("cfg")) {
        try {
            YAML::Node cfg = YAML::LoadFile(config_file);
            config = loadCompositeDBRRTConfig(cfg);
            config.time_limit = time_limit;  // Override with command line
        } catch (const YAML::Exception& e) {
            std::cerr << "Error loading config file: " << e.what() << std::endl;
            return 1;
        }
    }

    std::cout << "Composite DB-RRT Multi-Robot Motion Planning" << std::endl;
    std::cout << "=============================================" << std::endl;
    std::cout << "Input: " << input_file << std::endl;
    std::cout << "Output: " << output_file << std::endl;
    if (vm.count("cfg")) {
        std::cout << "Config: " << config_file << std::endl;
    }
    std::cout << "Time limit: " << config.time_limit << "s" << std::endl;
    std::cout << std::endl;

    // Load input YAML
    YAML::Node input;
    try {
        input = YAML::LoadFile(input_file);
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading input file: " << e.what() << std::endl;
        return 1;
    }

    // Build problem
    CompositeDBRRTProblem problem;

    // Environment bounds
    problem.env_min = {
        input["environment"]["min"][0].as<double>(),
        input["environment"]["min"][1].as<double>()
    };
    problem.env_max = {
        input["environment"]["max"][0].as<double>(),
        input["environment"]["max"][1].as<double>()
    };

    // Obstacles
    std::vector<fcl::CollisionObjectf*> obstacle_objects;
    if (input["environment"]["obstacles"]) {
        for (const auto& obs : input["environment"]["obstacles"]) {
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

                // Also add to dynobench obstacles
                dynobench::Obstacle dyn_obs;
                dyn_obs.type = "box";
                dyn_obs.size = Eigen::Vector3d(
                    size[0].as<double>(),
                    size[1].as<double>(),
                    1.0);
                dyn_obs.center = Eigen::Vector3d(
                    center[0].as<double>(),
                    center[1].as<double>(),
                    0.0);
                problem.dynobench_obstacles.push_back(dyn_obs);
            }
        }
    }
    std::cout << "Loaded " << obstacle_objects.size() << " obstacles" << std::endl;

    // Robots
    for (const auto& robot_config : input["robots"]) {
        CompositeRobotSpec spec;
        spec.type = robot_config["type"].as<std::string>();

        for (size_t i = 0; i < robot_config["start"].size(); ++i) {
            spec.start.push_back(robot_config["start"][i].as<double>());
        }
        for (size_t i = 0; i < robot_config["goal"].size(); ++i) {
            spec.goal.push_back(robot_config["goal"][i].as<double>());
        }

        problem.robots.push_back(spec);
    }
    std::cout << "Loaded " << problem.robots.size() << " robots" << std::endl;
    std::cout << std::endl;

    // Create planner and solve
    CompositeDBRRTPlanner planner(config);
    CompositeDBRRTResult result = planner.plan(problem);

    // Write output
    YAML::Node output;
    output["solved"] = result.solved;
    output["planning_time"] = result.planning_time;

    if (result.solved) {
        std::cout << "\nSolution found!" << std::endl;

        for (size_t r = 0; r < result.trajectories.size(); ++r) {
            YAML::Node robot_result;
            const auto& traj = result.trajectories[r];

            std::vector<std::vector<double>> states_list;
            for (const auto& state : traj.states) {
                std::vector<double> state_vec(state.data(), state.data() + state.size());
                states_list.push_back(state_vec);
            }
            robot_result["states"] = states_list;

            if (!traj.actions.empty()) {
                std::vector<std::vector<double>> actions_list;
                for (const auto& action : traj.actions) {
                    std::vector<double> action_vec(action.data(), action.data() + action.size());
                    actions_list.push_back(action_vec);
                }
                robot_result["actions"] = actions_list;
            }

            output["result"].push_back(robot_result);

            std::cout << "Robot " << r << ": " << traj.states.size() << " states" << std::endl;
        }
    } else {
        std::cout << "\nNo solution found" << std::endl;
    }

    std::ofstream fout(output_file);
    fout << output;
    fout.close();

    std::cout << "\nResults written to: " << output_file << std::endl;

    // Cleanup obstacles
    for (auto* co : obstacle_objects) {
        delete co;
    }

    return result.solved ? 0 : 1;
}
