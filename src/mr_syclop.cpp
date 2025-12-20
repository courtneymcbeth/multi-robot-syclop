// Pinocchio/Crocoddyl must be included before Boost headers
#include <dynoplan/optimization/ocp.hpp>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <iterator>
#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>
#include <boost/heap/d_ary_heap.hpp>

// OMPL headers
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
// #include <ompl/base/objectives/ControlDurationObjective.h>
#include <ompl/base/OptimizationObjective.h>

#include "robots.h"
#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"

// #define DBG_PRINTS
#include "db_astar.hpp"
#include "planresult.hpp"


// #include "multirobot_trajectory.hpp"
#include "dynoplan/optimization/multirobot_optimization.hpp"

#include "decomposition.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

/**
Below is skeleton code for SyCLoMP with TODO markers for us.
Change function definitions or make helper files/classes as needed.
*/

void compute_high_level_paths() {
    // TODO @courtneymcbeth
    std::cout << "High-level path computation is not implemented yet." << std::endl;
}

void compute_kinodynamic_paths() {
    // TODO @imngui
    std::cout << "Kinodynamic path computation is not implemented yet." << std::endl;
}

void segment_kinodynamic_paths() {
    // TODO @imngui
    std::cout << "Kinodynamic path segmentation is not implemented yet." << std::endl;
}

void check_segments_for_collisions() {
    // TODO @imngui
    std::cout << "Collision checking for segments is not implemented yet." << std::endl;
}

void update_decomposition() {
    // TODO @imngui
    std::cout << "Decomposition update is not implemented yet." << std::endl;
}

void expand_subproblem() {
    // TODO @imngui
    std::cout << "Subproblem expansion is not implemented yet." << std::endl;
}

void use_composite_planner() {
    // TODO @courtneymcbeth
    std::cout << "Composite planner usage is not implemented yet." << std::endl;
}

void resolve_collisions() {
    // TODO @imngui and @courtneymcbeth
    std::cout << "Collision resolution is not implemented yet." << std::endl;

    // TODO Decide on criteria for each method

    // If changing decomposition
    update_decomposition();

    // If expanding subproblem
    expand_subproblem();

    // If using composite planner
    use_composite_planner();
}

int main(int argc, char* argv[]) {
    
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string outputFile;
    std::string jointFile;
    std::string optimizationFile;
    std::string cfgFile;

    desc.add_options()
      ("help", "produce help message")
      ("input,i", po::value<std::string>(&inputFile)->required(), "input file (yaml)")
      ("output,o", po::value<std::string>(&outputFile)->required(), "output file (yaml)")
      ("joint,jnt", po::value<std::string>(&jointFile)->required(), "joint output file (yaml)")
      ("optimization,opt", po::value<std::string>(&optimizationFile)->required(), "optimization file (yaml)")
      ("cfg,c", po::value<std::string>(&cfgFile)->required(), "configuration file (yaml)");

    try {
      po::variables_map vm;
      po::store(po::parse_command_line(argc, argv, desc), vm);
      po::notify(vm);

      if (vm.count("help") != 0u) {
        std::cout << desc << "\n";
        return 0;
      }
    } catch (po::error& e) {
      std::cerr << e.what() << std::endl << std::endl;
      std::cerr << desc << std::endl;
      return 1;
    }

    // load config file
    YAML::Node cfg = YAML::LoadFile(cfgFile);
    float decompRegionLength = cfg["decompRegionLength"].as<float>();

    // load problem description
    YAML::Node env = YAML::LoadFile(inputFile);
    std::vector<fcl::CollisionObjectf *> obstacles;
    std::vector<std::vector<fcl::Vector3f>> positions;
    for (const auto &obs : env["environment"]["obstacles"])
    {
        if (obs["type"].as<std::string>() == "box"){
            const auto &size = obs["size"];
            std::shared_ptr<fcl::CollisionGeometryf> geom;
            geom.reset(new fcl::Boxf(size[0].as<float>(), size[1].as<float>(), 1.0));
            const auto &center = obs["center"];
            auto co = new fcl::CollisionObjectf(geom);
            co->setTranslation(fcl::Vector3f(center[0].as<float>(), center[1].as<float>(), 0));
            co->computeAABB();
            obstacles.push_back(co);
        }
        else {
        throw std::runtime_error("Unknown obstacle type!");
        }
    }
    const auto &env_min = env["environment"]["min"];
    const auto &env_max = env["environment"]["max"];
    ob::RealVectorBounds position_bounds(env_min.size());
    for (size_t i = 0; i < env_min.size(); ++i) {
        position_bounds.setLow(i, env_min[i].as<double>());
        position_bounds.setHigh(i, env_max[i].as<double>());
    }

    fcl::AABBf workspace_aabb(
        fcl::Vector3f(env_min[0].as<double>(),
        env_min[1].as<double>(),-1),
        fcl::Vector3f(env_max[0].as<double>(), env_max[1].as<double>(), 1));

    std::vector<std::vector<double>> starts;
    std::vector<std::vector<double>> goals;
    std::vector<std::string> robot_types;

    std::vector<std::shared_ptr<Robot>> robots;
    for (const auto &robot_node : env["robots"]) {
        auto robotType = robot_node["type"].as<std::string>();
        std::shared_ptr<Robot> robot = create_robot(robotType, position_bounds);
        robots.push_back(robot);

        std::vector<double> start_reals;
        for (const auto& v : robot_node["start"]) {
            start_reals.push_back(v.as<double>());
        }
        starts.push_back(start_reals);

        std::vector<double> goal_reals;
        for (const auto& v : robot_node["goal"]) {
            goal_reals.push_back(v.as<double>());
        }
        goals.push_back(goal_reals);
        robot_types.push_back(robotType);
    }

    // Create decomposition based on decomposition length and workspace bounds
    ob::RealVectorBounds workspace_bounds(2);
    workspace_bounds.setLow(0, env_min[0].as<double>());
    workspace_bounds.setLow(1, env_min[1].as<double>());
    workspace_bounds.setHigh(0, env_max[0].as<double>());
    workspace_bounds.setHigh(1, env_max[1].as<double>());
    GridDecompositionImpl *decomp = new GridDecompositionImpl(
        decompRegionLength, 2, workspace_bounds);

    // Compute high-level paths over decomposition
    compute_high_level_paths();
    
    // Compute kinodynamic low-level paths
    compute_kinodynamic_paths();

    // Segment kinodynamic paths
    segment_kinodynamic_paths();

    // Check segments for collisions
    check_segments_for_collisions();

    // If collisions are found, decide how to resolve them
    resolve_collisions();

    // Write the final plan to the output file
    // TODO anyone: pull this from db-cbs

    return 0;
}
