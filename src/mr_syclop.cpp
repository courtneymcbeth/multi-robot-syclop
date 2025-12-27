// Pinocchio/Crocoddyl must be included before Boost headers
#include <dynoplan/optimization/ocp.hpp>

#include "mr_syclop.h"

#include <iostream>
#include <algorithm>
#include <chrono>
#include <boost/heap/d_ary_heap.hpp>

#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"
#include "mapf/mapf_solver.h"

#define DBG_PRINTS
#include "db_astar.hpp"
#include "planresult.hpp"
#include "dynoplan/optimization/multirobot_optimization.hpp"

// ============================================================================
// MRSyCLoPPlanner Implementation
// ============================================================================

MRSyCLoPPlanner::MRSyCLoPPlanner(const MRSyCLoPConfig& config)
    : config_(config), workspace_bounds_(2)
{
}

MRSyCLoPPlanner::~MRSyCLoPPlanner()
{
    cleanup();
}

void MRSyCLoPPlanner::cleanup()
{
    // Clean up allocated states
    if (!start_states_.empty() && !robots_.empty()) {
        for (size_t i = 0; i < start_states_.size() && i < robots_.size(); ++i) {
            if (start_states_[i]) {
                robots_[i]->getSpaceInformation()->getStateSpace()->freeState(start_states_[i]);
            }
        }
        start_states_.clear();
    }

    if (!goal_states_.empty() && !robots_.empty()) {
        for (size_t i = 0; i < goal_states_.size() && i < robots_.size(); ++i) {
            if (goal_states_[i]) {
                robots_[i]->getSpaceInformation()->getStateSpace()->freeState(goal_states_[i]);
            }
        }
        goal_states_.clear();
    }

    // Clear other data (but don't delete obstacles - caller owns them)
    robots_.clear();
    high_level_paths_.clear();
    problem_loaded_ = false;
}

void MRSyCLoPPlanner::loadProblem(
    const std::vector<std::string>& robot_types,
    const std::vector<std::vector<double>>& starts,
    const std::vector<std::vector<double>>& goals,
    const std::vector<fcl::CollisionObjectf*>& obstacles,
    const std::vector<double>& env_min,
    const std::vector<double>& env_max)
{
    // Clean up any previous problem
    cleanup();

    // Store problem data
    robot_types_ = robot_types;
    starts_ = starts;
    goals_ = goals;
    obstacles_ = obstacles;
    env_min_ = env_min;
    env_max_ = env_max;

    // Setup workspace bounds
    workspace_bounds_.setLow(0, env_min[0]);
    workspace_bounds_.setLow(1, env_min[1]);
    workspace_bounds_.setHigh(0, env_max[0]);
    workspace_bounds_.setHigh(1, env_max[1]);

    // Setup decomposition and robots
    setupDecomposition();
    setupRobots();

    problem_loaded_ = true;

#ifdef DBG_PRINTS
    std::cout << "Problem loaded successfully" << std::endl;
    std::cout << "  " << robots_.size() << " robots" << std::endl;
    std::cout << "  " << obstacles_.size() << " obstacles" << std::endl;
    std::cout << "  " << decomp_->getNumRegions() << " decomposition regions" << std::endl;
#endif
}

void MRSyCLoPPlanner::setupDecomposition()
{
#ifdef DBG_PRINTS
    std::cout << "Creating decomposition..." << std::endl;
#endif
    decomp_ = std::make_shared<GridDecompositionImpl>(
        config_.decomposition_region_length, 2, workspace_bounds_);
#ifdef DBG_PRINTS
    std::cout << "  Decomposition created with " << decomp_->getNumRegions() << " regions" << std::endl;
#endif
}

void MRSyCLoPPlanner::setupRobots()
{
#ifdef DBG_PRINTS
    std::cout << "Initializing robots..." << std::endl;
#endif

    ob::RealVectorBounds position_bounds(env_min_.size());
    for (size_t i = 0; i < env_min_.size(); ++i) {
        position_bounds.setLow(i, env_min_[i]);
        position_bounds.setHigh(i, env_max_[i]);
    }

    for (size_t i = 0; i < robot_types_.size(); ++i) {
        // Create robot
        auto robot = create_robot(robot_types_[i], position_bounds);
        auto si = robot->getSpaceInformation();
        si->setStatePropagator(std::make_shared<RobotStatePropagator>(si, robot));
        si->setup();
        robots_.push_back(robot);

        // Create start state
        ob::State* start = si->getStateSpace()->allocState();
        si->getStateSpace()->copyFromReals(start, starts_[i]);
        start_states_.push_back(start);

        // Create goal state
        ob::State* goal = si->getStateSpace()->allocState();
        si->getStateSpace()->copyFromReals(goal, goals_[i]);
        goal_states_.push_back(goal);
    }

#ifdef DBG_PRINTS
    std::cout << "  Initialized " << robots_.size() << " robots" << std::endl;
#endif
}

void MRSyCLoPPlanner::computeHighLevelPaths()
{
    if (!problem_loaded_) {
        throw std::runtime_error("Problem not loaded. Call loadProblem() first.");
    }

#ifdef DBG_PRINTS
    std::cout << "Computing high-level paths..." << std::endl;
#endif

    auto mapf_solver = createMAPFSolver(
        config_.mapf_config.method,
        config_.mapf_config.region_capacity,
        config_.planning_time_limit);

    high_level_paths_ = mapf_solver->solve(decomp_, start_states_, goal_states_);

#ifdef DBG_PRINTS
    std::cout << "High-level paths computed using "
              << mapf_solver->getName() << ":" << std::endl;
    for (size_t i = 0; i < high_level_paths_.size(); ++i) {
        std::cout << "  Robot " << i << ": ";
        for (int region : high_level_paths_[i]) {
            std::cout << region << " ";
        }
        std::cout << std::endl;
    }
#endif
}

void MRSyCLoPPlanner::computeKinodynamicPaths()
{
    // TODO @imngui
    std::cout << "Kinodynamic path computation is not implemented yet." << std::endl;
}

void MRSyCLoPPlanner::segmentKinodynamicPaths()
{
    // TODO @imngui
    std::cout << "Kinodynamic path segmentation is not implemented yet." << std::endl;
}

bool MRSyCLoPPlanner::checkSegmentsForCollisions()
{
    // TODO @imngui
    std::cout << "Collision checking for segments is not implemented yet." << std::endl;
    return false; // No collisions detected (placeholder)
}

void MRSyCLoPPlanner::updateDecomposition()
{
    // TODO @imngui
    std::cout << "Decomposition update is not implemented yet." << std::endl;
}

void MRSyCLoPPlanner::expandSubproblem()
{
    // TODO @imngui
    std::cout << "Subproblem expansion is not implemented yet." << std::endl;
}

std::vector<fcl::CollisionObjectf*> MRSyCLoPPlanner::getObstaclesInRegion(
    const std::vector<double>& region_min,
    const std::vector<double>& region_max) const
{
    std::vector<fcl::CollisionObjectf*> obstacles_in_region;

    // Create AABB for the region
    fcl::AABBf region_aabb(
        fcl::Vector3f(region_min[0], region_min[1], -1.0f),
        fcl::Vector3f(region_max[0], region_max[1], 1.0f)
    );

    // Check each obstacle for overlap with the region
    for (auto* obstacle : obstacles_) {
        const fcl::AABBf& obstacle_aabb = obstacle->getAABB();

        // Check if AABBs overlap
        if (region_aabb.overlap(obstacle_aabb)) {
            obstacles_in_region.push_back(obstacle);
        }
    }

    return obstacles_in_region;
}

PlanningResult MRSyCLoPPlanner::useCompositePlanner(
    const std::vector<size_t>& robot_indices,
    const std::vector<std::vector<double>>& subproblem_starts,
    const std::vector<std::vector<double>>& subproblem_goals,
    const std::vector<double>& subproblem_env_min,
    const std::vector<double>& subproblem_env_max)
{
    std::cout << "Using composite (coupled RRT) planner on subproblem..." << std::endl;
    std::cout << "  Subproblem robots: ";
    for (size_t idx : robot_indices) {
        std::cout << idx << " ";
    }
    std::cout << std::endl;

    // Get obstacles that overlap with the subproblem region
    auto subproblem_obstacles = getObstaclesInRegion(subproblem_env_min, subproblem_env_max);
    std::cout << "  Subproblem obstacles: " << subproblem_obstacles.size() << std::endl;

    // Create planning problem for subproblem
    PlanningProblem problem;
    problem.env_min = subproblem_env_min;
    problem.env_max = subproblem_env_max;
    problem.obstacles = subproblem_obstacles;

    // Add robot specifications for robots in subproblem
    for (size_t i = 0; i < robot_indices.size(); ++i) {
        size_t robot_idx = robot_indices[i];

        RobotSpec robot_spec;
        robot_spec.type = robot_types_[robot_idx];
        robot_spec.start = subproblem_starts[i];
        robot_spec.goal = subproblem_goals[i];
        problem.robots.push_back(robot_spec);
    }

    // Create planner and solve
    CoupledRRTPlanner planner(config_.coupled_rrt_config);
    PlanningResult result = planner.plan(problem);

    std::cout << "Coupled RRT planning completed in " << result.planning_time << " seconds" << std::endl;
    std::cout << "Solution found: " << (result.solved ? "Yes" : "No") << std::endl;

    if (result.solved && result.path) {
        std::cout << "Path has " << result.path->getStateCount() << " states and "
                  << result.path->getControlCount() << " controls" << std::endl;
    } else {
        std::cout << "No solution found by coupled RRT planner" << std::endl;
    }

    return result;
}

void MRSyCLoPPlanner::resolveCollisions()
{
    // TODO @imngui and @courtneymcbeth
    std::cout << "Collision resolution is not implemented yet." << std::endl;

    // TODO Decide on criteria for each method

    // If changing decomposition
    updateDecomposition();

    // If expanding subproblem
    expandSubproblem();

    // If using composite planner
    // TODO: Construct the actual subproblem (determine which robots are in conflict,
    // compute subproblem bounds, etc.)
    // Example placeholder:
    // std::vector<size_t> conflict_robots = {0, 1};  // robots involved in collision
    // std::vector<std::vector<double>> sub_starts = {starts_[0], starts_[1]};
    // std::vector<std::vector<double>> sub_goals = {goals_[0], goals_[1]};
    // std::vector<double> sub_env_min = env_min_;  // or computed from conflict region
    // std::vector<double> sub_env_max = env_max_;
    // PlanningResult result = useCompositePlanner(
    //     conflict_robots, sub_starts, sub_goals, sub_env_min, sub_env_max);

    // TODO: Extract and integrate the path from result.path
    // The path is an OMPL PathControl object containing the compound solution
    // for the robots in the subproblem. You can extract individual robot
    // trajectories and integrate them into the full solution.
}

MRSyCLoPResult MRSyCLoPPlanner::plan()
{
    if (!problem_loaded_) {
        throw std::runtime_error("Problem not loaded. Call loadProblem() first.");
    }

    MRSyCLoPResult result;
    auto start_time = std::chrono::steady_clock::now();

    try {
        // Compute high-level paths over decomposition
        computeHighLevelPaths();

        // Compute kinodynamic low-level paths
        computeKinodynamicPaths();

        // Segment kinodynamic paths
        segmentKinodynamicPaths();

        // Check segments for collisions
        bool collisions_found = checkSegmentsForCollisions();

        // If collisions are found, decide how to resolve them
        if (collisions_found) {
            resolveCollisions();
        }

        result.success = true; // Placeholder

    } catch (const std::exception& e) {
        std::cerr << "Planning failed: " << e.what() << std::endl;
        result.success = false;
    }

    auto end_time = std::chrono::steady_clock::now();
    result.planning_time = std::chrono::duration<double>(end_time - start_time).count();

    return result;
}
int main(int argc, char* argv[]) {

    // Parse command line arguments
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

    // Vector to hold obstacle objects for cleanup
    std::vector<fcl::CollisionObjectf*> obstacles;

    try {
        // Load config file
        std::cout << "Loading configuration file..." << std::endl;
        YAML::Node cfg = YAML::LoadFile(cfgFile);

        MRSyCLoPConfig config;
        config.decomposition_region_length = cfg["decomposition_region_length"].as<int>();

        // Load MAPF configuration
        if (cfg["mapf"]) {
            if (cfg["mapf"]["method"]) {
                config.mapf_config.method = cfg["mapf"]["method"].as<std::string>();
            }
            if (cfg["mapf"]["region_capacity"]) {
                config.mapf_config.region_capacity = cfg["mapf"]["region_capacity"].as<int>();
            }
        }

        // Set coupled RRT config if needed
        config.coupled_rrt_config.goal_threshold = 0.5;
        config.coupled_rrt_config.min_control_duration = 1;
        config.coupled_rrt_config.max_control_duration = 10;

        std::cout << "  Decomposition region length: " << config.decomposition_region_length << std::endl;
        std::cout << "  MAPF method: " << config.mapf_config.method << std::endl;
        std::cout << "  MAPF region capacity: " << config.mapf_config.region_capacity << std::endl;

        // Load problem description
        std::cout << "Loading problem description..." << std::endl;
        YAML::Node env = YAML::LoadFile(inputFile);

        // Parse obstacles
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
        std::cout << "  Loaded " << obstacles.size() << " obstacles" << std::endl;

        // Parse environment bounds
        const auto &env_min = env["environment"]["min"];
        const auto &env_max = env["environment"]["max"];
        std::vector<double> env_min_vec = {env_min[0].as<double>(), env_min[1].as<double>()};
        std::vector<double> env_max_vec = {env_max[0].as<double>(), env_max[1].as<double>()};

        // Parse robots
        std::vector<std::vector<double>> starts;
        std::vector<std::vector<double>> goals;
        std::vector<std::string> robot_types;

        std::cout << "Parsing robot specifications..." << std::endl;
        for (const auto &robot_node : env["robots"]) {
            auto robotType = robot_node["type"].as<std::string>();
            robot_types.push_back(robotType);

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
        }
        std::cout << "  Parsed " << robot_types.size() << " robot specifications" << std::endl;

        // Create planner and load problem
        MRSyCLoPPlanner planner(config);
        planner.loadProblem(robot_types, starts, goals, obstacles, env_min_vec, env_max_vec);

        // Run planner
        std::cout << "\nStarting planning..." << std::endl;
        MRSyCLoPResult result = planner.plan();

        std::cout << "\nPlanning completed!" << std::endl;
        std::cout << "  Success: " << (result.success ? "Yes" : "No") << std::endl;
        std::cout << "  Planning time: " << result.planning_time << " seconds" << std::endl;

        // TODO: Write the final plan to the output files
        // For now, just write a placeholder
        YAML::Node output;
        output["success"] = result.success;
        output["planning_time"] = result.planning_time;

        std::ofstream fout(outputFile);
        fout << output;
        fout.close();
        std::cout << "Output written to " << outputFile << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;

        // Clean up obstacles
        for (auto* co : obstacles) {
            delete co;
        }

        return 1;
    }

    // Clean up obstacles
    for (auto* co : obstacles) {
        delete co;
    }

    return 0;
}
