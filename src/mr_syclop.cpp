// Pinocchio/Crocoddyl must be included before Boost headers
#include <dynoplan/optimization/ocp.hpp>

#include "mr_syclop.h"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <climits>
#include <queue>
#include <set>
#include <boost/heap/d_ary_heap.hpp>
#include <ompl/util/RandomNumbers.h>

#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"
#include "mapf/mapf_solver.h"

#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

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
    guided_planning_results_.clear();
    path_segments_.clear();
    collision_manager_.reset();
    problem_loaded_ = false;
    resolution_stats_ = ResolutionStats();  // Reset resolution statistics
    robot_pair_collision_counts_.clear();   // Reset cycle detection counters
    decomposition_hierarchy_.clear();       // Clear decomposition hierarchy
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

    // Setup decomposition, collision manager, and robots
    setupDecomposition();
    setupCollisionManager();
    // Convert FCL obstacles to dynobench format if using DB-RRT
    if (config_.guided_planner_method == "db_rrt") {
        setupDynobenchObstacles();
    }
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

    // Initialize decomposition hierarchy tracking
    initializeDecompositionHierarchy();

    // Save decomposition to file for visualization
    saveDecompositionToFile(decomp_, "initial");
}

void MRSyCLoPPlanner::setupCollisionManager()
{
#ifdef DBG_PRINTS
    std::cout << "Setting up collision manager..." << std::endl;
#endif
    collision_manager_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
    collision_manager_->registerObjects(obstacles_);
    collision_manager_->setup();
#ifdef DBG_PRINTS
    std::cout << "  Collision manager setup with " << obstacles_.size() << " obstacles" << std::endl;
#endif
}

void MRSyCLoPPlanner::setupDynobenchObstacles()
{
#ifdef DBG_PRINTS
    std::cout << "Converting obstacles to dynobench format..." << std::endl;
#endif
    dynobench_obstacles_.clear();
    dynobench_obstacles_.reserve(obstacles_.size());

    for (const auto* obs : obstacles_) {
        const auto* geom = obs->collisionGeometry().get();
        const auto& translation = obs->getTranslation();

        // Check if it's a box (currently the only supported type)
        if (const auto* box = dynamic_cast<const fcl::Boxf*>(geom)) {
            dynobench::Obstacle dyno_obs;
            dyno_obs.type = "box";

            // FCL Boxf stores half-extents as side[i], full size = 2 * side[i]
            // But we created it with full size, so box->side is actually half the size
            // Actually, FCL Boxf constructor takes full width/height/depth
            dyno_obs.size.resize(2);
            dyno_obs.size(0) = box->side[0];  // width
            dyno_obs.size(1) = box->side[1];  // height

            dyno_obs.center.resize(2);
            dyno_obs.center(0) = static_cast<double>(translation[0]);
            dyno_obs.center(1) = static_cast<double>(translation[1]);

            dynobench_obstacles_.push_back(dyno_obs);
        }
    }

#ifdef DBG_PRINTS
    std::cout << "  Converted " << dynobench_obstacles_.size() << " obstacles to dynobench format" << std::endl;
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

    high_level_paths_ = mapf_solver->solve(
        decomp_, start_states_, goal_states_,
        obstacles_, config_.mapf_config.max_obstacle_volume_percent);

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

void MRSyCLoPPlanner::computeGuidedPaths()
{
    /// @todo @imngui: Need to check if this is actually using the high level paths properly

    if (!problem_loaded_) {
        throw std::runtime_error("Problem not loaded. Call loadProblem() first.");
    }

    if (high_level_paths_.empty()) {
        throw std::runtime_error(
            "High-level paths not computed. Call computeHighLevelPaths() first.");
    }

#ifdef DBG_PRINTS
    std::cout << "Computing guided low-level paths using "
              << config_.guided_planner_method << "..." << std::endl;
#endif

    // Clear previous results
    guided_planning_results_.clear();

    // Create guided planner for individual robot planning
    auto guided_planner = createGuidedPlannerWithDBRRT(
        config_.guided_planner_method,
        config_.guided_planner_config,
        config_.db_rrt_config,
        collision_manager_,
        dynobench_obstacles_);

    // Plan for each robot independently
    for (size_t i = 0; i < robots_.size(); ++i) {
#ifdef DBG_PRINTS
        std::cout << "Planning for robot " << i << "..." << std::endl;
#endif

        mr_syclop::GuidedPlanningResult result = guided_planner->solve(
            robots_[i],
            decomp_,
            start_states_[i],
            goal_states_[i],
            high_level_paths_[i],
            i);

        guided_planning_results_.push_back(result);

#ifdef DBG_PRINTS
        if (result.success) {
            std::cout << "  Success! Planning time: "
                      << result.planning_time << " seconds" << std::endl;
        } else {
            std::cout << "  Failed after " << result.planning_time
                      << " seconds" << std::endl;
        }
#endif
    }

#ifdef DBG_PRINTS
    size_t num_successful = 0;
    for (const auto& result : guided_planning_results_) {
        if (result.success) num_successful++;
    }
    std::cout << "Guided path computation completed: "
              << num_successful << "/" << robots_.size()
              << " robots found paths" << std::endl;
#endif
}

void MRSyCLoPPlanner::computeGuidedPathsWithCompositeDBRRT()
{
#ifdef DBG_PRINTS
    std::cout << "Using CompositeDBRRT for joint multi-robot planning..." << std::endl;
#endif

    // Build the CompositeDBRRT problem
    CompositeDBRRTProblem problem;
    problem.env_min = env_min_;
    problem.env_max = env_max_;
    problem.obstacles = obstacles_;
    problem.dynobench_obstacles = dynobench_obstacles_;

    // Add robot specifications
    for (size_t i = 0; i < robots_.size(); ++i) {
        CompositeRobotSpec spec;
        spec.type = robot_types_[i];
        spec.start = starts_[i];
        spec.goal = goals_[i];
        problem.robots.push_back(spec);
    }

    // Create and run the CompositeDBRRT planner
    CompositeDBRRTPlanner composite_planner(config_.composite_dbrrt_config);
    CompositeDBRRTResult result = composite_planner.plan(problem);

#ifdef DBG_PRINTS
    std::cout << "CompositeDBRRT planning " << (result.solved ? "succeeded" : "failed")
              << " in " << result.planning_time << " seconds" << std::endl;
#endif

    // Convert results to GuidedPlanningResult format
    if (result.solved && result.trajectories.size() == robots_.size()) {
        for (size_t i = 0; i < robots_.size(); ++i) {
            mr_syclop::GuidedPlanningResult guided_result;
            guided_result.success = true;
            guided_result.planning_time = result.planning_time;
            guided_result.robot_index = i;
            guided_result.path = convertDynobenchTrajectory(result.trajectories[i], robots_[i]);
            guided_planning_results_.push_back(guided_result);

#ifdef DBG_PRINTS
            std::cout << "  Robot " << i << ": trajectory with "
                      << result.trajectories[i].states.size() << " states" << std::endl;
#endif
        }
    } else {
        // Planning failed - create empty results for each robot
        for (size_t i = 0; i < robots_.size(); ++i) {
            mr_syclop::GuidedPlanningResult guided_result;
            guided_result.success = false;
            guided_result.planning_time = result.planning_time;
            guided_result.robot_index = i;
            guided_result.path = nullptr;
            guided_planning_results_.push_back(guided_result);
        }
    }

#ifdef DBG_PRINTS
    size_t num_successful = 0;
    for (const auto& res : guided_planning_results_) {
        if (res.success) num_successful++;
    }
    std::cout << "CompositeDBRRT path computation completed: "
              << num_successful << "/" << robots_.size()
              << " robots found paths" << std::endl;
#endif
}

std::shared_ptr<oc::PathControl> MRSyCLoPPlanner::convertDynobenchTrajectory(
    const dynobench::Trajectory& traj,
    const std::shared_ptr<Robot>& robot)
{
    auto si = robot->getSpaceInformation();
    auto path = std::make_shared<oc::PathControl>(si);
    auto state_space = si->getStateSpace();

    // Convert states and controls
    for (size_t i = 0; i < traj.states.size(); ++i) {
        // Allocate and convert state
        ob::State* state = si->allocState();

        // Convert Eigen vector to OMPL state
        size_t idx = 0;
        if (auto compound = state_space->as<ob::CompoundStateSpace>()) {
            auto* compound_state = state->as<ob::CompoundState>();
            for (size_t s = 0; s < compound->getSubspaceCount(); ++s) {
                auto subspace = compound->getSubspace(s);
                auto* substate = compound_state->as<ob::State>(s);

                if (subspace->getType() == ob::STATE_SPACE_SO2) {
                    auto* so2_state = substate->as<ob::SO2StateSpace::StateType>();
                    so2_state->value = traj.states[i](idx++);
                } else if (subspace->getType() == ob::STATE_SPACE_REAL_VECTOR) {
                    auto rv_space = subspace->as<ob::RealVectorStateSpace>();
                    auto* rv_state = substate->as<ob::RealVectorStateSpace::StateType>();
                    for (size_t j = 0; j < rv_space->getDimension(); ++j) {
                        rv_state->values[j] = traj.states[i](idx++);
                    }
                }
            }
        } else if (auto rv_space = state_space->as<ob::RealVectorStateSpace>()) {
            auto* rv_state = state->as<ob::RealVectorStateSpace::StateType>();
            for (size_t j = 0; j < rv_space->getDimension(); ++j) {
                rv_state->values[j] = traj.states[i](j);
            }
        }

        if (i < traj.actions.size()) {
            // Allocate and convert control
            oc::Control* control = si->allocControl();
            auto* rv_control = control->as<oc::RealVectorControlSpace::ControlType>();

            for (size_t j = 0; j < traj.actions[i].size() &&
                              j < si->getControlSpace()->getDimension(); ++j) {
                rv_control->values[j] = traj.actions[i](j);
            }

            // Default timestep duration (should match dynobench model dt)
            double duration = 0.1;
            path->append(state, control, duration);
        } else {
            // Last state, no control
            path->append(state);
        }
    }

    return path;
}

void MRSyCLoPPlanner::segmentGuidedPaths()
{
    if (!problem_loaded_) {
        throw std::runtime_error("Problem not loaded. Call loadProblem() first.");
    }

    if (guided_planning_results_.empty()) {
        throw std::runtime_error(
            "Guided paths not computed. Call computeGuidedPaths() first.");
    }

#ifdef DBG_PRINTS
    std::cout << "Segmenting guided paths by timesteps (m="
              << config_.segment_timesteps << ")..." << std::endl;
#endif

    // Clear previous segments
    path_segments_.clear();
    path_segments_.resize(robots_.size());

    // Segment each robot's path
    for (size_t robot_idx = 0; robot_idx < guided_planning_results_.size(); ++robot_idx) {
        const auto& result = guided_planning_results_[robot_idx];

        if (!result.success || !result.path) {
#ifdef DBG_PRINTS
            std::cout << "  Robot " << robot_idx << ": No path to segment (planning failed)" << std::endl;
#endif
            continue;
        }

        auto& path = result.path;
        size_t num_controls = path->getControlCount();

        if (num_controls == 0) {
#ifdef DBG_PRINTS
            std::cout << "  Robot " << robot_idx << ": Empty path" << std::endl;
#endif
            continue;
        }

#ifdef DBG_PRINTS
        std::cout << "  Robot " << robot_idx << ": " << num_controls << " controls" << std::endl;
#endif

        // Segment the path
        int current_timestep = 0;
        size_t segment_idx = 0;
        size_t control_idx = 0;
        double remaining_control_duration = 0.0;  // Track partial control durations

        while (control_idx < num_controls || remaining_control_duration > 0.0) {
            PathSegment segment;
            segment.robot_index = robot_idx;
            segment.segment_index = segment_idx;
            segment.start_timestep = current_timestep;

            // Start state is the current state we're at
            segment.start_state = path->getState(control_idx < num_controls ? control_idx : num_controls);
            segment.total_duration = 0.0;

            int timesteps_in_segment = 0;

            // First, handle any remaining duration from previous control
            if (remaining_control_duration > 0.0) {
                int remaining_timesteps_from_prev = static_cast<int>(std::ceil(remaining_control_duration));

                if (remaining_timesteps_from_prev <= config_.segment_timesteps) {
                    // All remaining duration fits in this segment
                    segment.controls.push_back(path->getControl(control_idx - 1));
                    segment.control_durations.push_back(remaining_control_duration);
                    segment.total_duration += remaining_control_duration;
                    timesteps_in_segment += remaining_timesteps_from_prev;
                    remaining_control_duration = 0.0;
                } else {
                    // Only part of remaining duration fits
                    double partial_duration = config_.segment_timesteps;
                    segment.controls.push_back(path->getControl(control_idx - 1));
                    segment.control_durations.push_back(partial_duration);
                    segment.total_duration += partial_duration;
                    timesteps_in_segment += config_.segment_timesteps;
                    remaining_control_duration -= partial_duration;
                }
            }

            // Accumulate controls until we reach segment_timesteps
            while (control_idx < num_controls && timesteps_in_segment < config_.segment_timesteps) {
                double control_duration = path->getControlDuration(control_idx);
                int control_timesteps = static_cast<int>(std::ceil(control_duration));

                int remaining_timesteps = config_.segment_timesteps - timesteps_in_segment;

                if (control_timesteps <= remaining_timesteps) {
                    // Entire control fits in this segment
                    segment.controls.push_back(path->getControl(control_idx));
                    segment.control_durations.push_back(control_duration);
                    segment.total_duration += control_duration;
                    timesteps_in_segment += control_timesteps;
                    control_idx++;
                } else {
                    // Control needs to be split
                    // Use only the portion that fits in this segment
                    double partial_duration = remaining_timesteps;
                    segment.controls.push_back(path->getControl(control_idx));
                    segment.control_durations.push_back(partial_duration);
                    segment.total_duration += partial_duration;
                    timesteps_in_segment += remaining_timesteps;

                    // Save the remaining duration for next segment
                    remaining_control_duration = control_duration - partial_duration;
                    control_idx++;
                    break;
                }
            }

            segment.end_timestep = current_timestep + timesteps_in_segment;
            current_timestep = segment.end_timestep;

            // Set end state (state we'll be at after this segment)
            segment.end_state = path->getState(control_idx < num_controls ? control_idx : num_controls);

            path_segments_[robot_idx].push_back(segment);
            segment_idx++;
        }

#ifdef DBG_PRINTS
        std::cout << "    Created " << path_segments_[robot_idx].size()
                  << " segments for robot " << robot_idx << std::endl;

        // Print detailed segment info
        for (const auto& seg : path_segments_[robot_idx]) {
            std::cout << "      Segment " << seg.segment_index
                      << ": timesteps [" << seg.start_timestep << ", " << seg.end_timestep << "), "
                      << seg.controls.size() << " controls, "
                      << "total duration: " << seg.total_duration << std::endl;
        }
#endif
    }

#ifdef DBG_PRINTS
    size_t total_segments = 0;
    for (const auto& robot_segments : path_segments_) {
        total_segments += robot_segments.size();
    }
    std::cout << "Path segmentation completed: " << total_segments
              << " total segments across " << robots_.size() << " robots" << std::endl;
#endif
}

// ============================================================================
// Collision Checking Helper Functions
// ============================================================================

const PathSegment* MRSyCLoPPlanner::findSegmentAtTimestep(size_t robot_idx, int timestep) const
{
    if (robot_idx >= path_segments_.size()) {
        return nullptr;
    }

    const auto& segments = path_segments_[robot_idx];
    for (const auto& segment : segments) {
        if (segment.start_timestep <= timestep && timestep < segment.end_timestep) {
            return &segment;
        }
    }
    return nullptr;
}

void MRSyCLoPPlanner::propagateToTimestep(
    size_t robot_idx,
    size_t segment_idx,
    int timestep,
    ob::State* result) const
{
    const auto& segment = path_segments_[robot_idx][segment_idx];
    auto si = robots_[robot_idx]->getSpaceInformation();

    // Validate that timestep is within the segment's range
    if (timestep < segment.start_timestep || timestep >= segment.end_timestep) {
        std::cerr << "ERROR: propagateToTimestep called with invalid timestep!" << std::endl;
        std::cerr << "  robot_idx=" << robot_idx << ", segment_idx=" << segment_idx << std::endl;
        std::cerr << "  timestep=" << timestep << std::endl;
        std::cerr << "  segment.start_timestep=" << segment.start_timestep << std::endl;
        std::cerr << "  segment.end_timestep=" << segment.end_timestep << std::endl;

        // Clamp to valid range as a fallback
        if (timestep < segment.start_timestep) {
            timestep = segment.start_timestep;
        } else {
            timestep = segment.end_timestep - 1;
        }
    }

    // Start from segment start state
    si->copyState(result, segment.start_state);

    // If at start timestep, no propagation needed
    if (timestep == segment.start_timestep) {
        return;
    }

    // Calculate relative timestep within segment
    int relative_timestep = timestep - segment.start_timestep;

    // Propagate through controls until we reach the target timestep
    int accumulated_timesteps = 0;
    ob::State* temp_state = si->getStateSpace()->allocState();
    si->copyState(temp_state, segment.start_state);

    for (size_t ctrl_idx = 0; ctrl_idx < segment.controls.size(); ++ctrl_idx) {
        double duration = segment.control_durations[ctrl_idx];
        int control_timesteps = static_cast<int>(std::ceil(duration));

        if (accumulated_timesteps + control_timesteps <= relative_timestep) {
            // Apply full control
            robots_[robot_idx]->propagate(
                temp_state,
                segment.controls[ctrl_idx],
                duration,
                temp_state);
            accumulated_timesteps += control_timesteps;
        } else {
            // Partial control application
            double partial_duration = relative_timestep - accumulated_timesteps;
            robots_[robot_idx]->propagate(
                temp_state,
                segment.controls[ctrl_idx],
                partial_duration,
                temp_state);
            break;
        }
    }

    si->copyState(result, temp_state);
    si->getStateSpace()->freeState(temp_state);
}

bool MRSyCLoPPlanner::checkTwoRobotCollision(
    size_t robot_idx_1,
    const ob::State* state_1,
    size_t robot_idx_2,
    const ob::State* state_2,
    size_t& part_1,
    size_t& part_2) const
{
    auto robot_1 = robots_[robot_idx_1];
    auto robot_2 = robots_[robot_idx_2];

    for (size_t p1 = 0; p1 < robot_1->numParts(); ++p1) {
        for (size_t p2 = 0; p2 < robot_2->numParts(); ++p2) {
            const auto& transform_1 = robot_1->getTransform(state_1, p1);
            const auto& transform_2 = robot_2->getTransform(state_2, p2);

            fcl::CollisionObjectf co_1(robot_1->getCollisionGeometry(p1));
            co_1.setTranslation(transform_1.translation());
            co_1.setRotation(transform_1.rotation());
            co_1.computeAABB();

            fcl::CollisionObjectf co_2(robot_2->getCollisionGeometry(p2));
            co_2.setTranslation(transform_2.translation());
            co_2.setRotation(transform_2.rotation());
            co_2.computeAABB();

            fcl::CollisionRequestf request;
            fcl::CollisionResultf result;
            fcl::collide(&co_1, &co_2, request, result);

            if (result.isCollision()) {
                part_1 = p1;
                part_2 = p2;
                return true;
            }
        }
    }

    return false;
}

// ============================================================================
// Collision Checking Main Function
// ============================================================================

bool MRSyCLoPPlanner::checkSegmentsForCollisions()
{
    // Clear previous collision data
    segment_collisions_.clear();

#ifdef DBG_PRINTS
    std::cout << "Checking segments for collisions..." << std::endl;
#endif

    // Handle edge cases
    if (path_segments_.empty()) {
        return false;  // No segments to check
    }

    // Check if any robot has segments
    bool any_robot_has_segments = false;
    for (const auto& robot_segments : path_segments_) {
        if (!robot_segments.empty()) {
            any_robot_has_segments = true;
            break;
        }
    }
    if (!any_robot_has_segments) {
        return false;  // No segments to check
    }

    // Find the global maximum timestep across ALL robots and ALL segments
    // This ensures we check every timestep where any robot is still moving
    int max_timestep = 0;
    for (size_t robot_idx = 0; robot_idx < robots_.size(); ++robot_idx) {
        if (!path_segments_[robot_idx].empty()) {
            const auto& last_segment = path_segments_[robot_idx].back();
            max_timestep = std::max(max_timestep, last_segment.end_timestep);
        }
    }

    if (max_timestep == 0) {
        return false;  // No timesteps to check
    }

#ifdef DBG_PRINTS
    std::cout << "  Checking timesteps 0 to " << max_timestep - 1 << std::endl;
#endif

    // Allocate states for each robot (one per robot for current timestep)
    std::vector<ob::State*> current_states(robots_.size(), nullptr);
    for (size_t i = 0; i < robots_.size(); ++i) {
        current_states[i] = robots_[i]->getSpaceInformation()->getStateSpace()->allocState();
    }

    // Iterate over ALL absolute timesteps to ensure complete coverage
    for (int timestep = 0; timestep < max_timestep; ++timestep) {

        // Propagate each robot to this absolute timestep
        for (size_t robot_idx = 0; robot_idx < robots_.size(); ++robot_idx) {
            if (path_segments_[robot_idx].empty()) continue;

            // Find the segment that contains this timestep
            const PathSegment* seg = findSegmentAtTimestep(robot_idx, timestep);

            if (seg != nullptr) {
                // Robot has a segment at this timestep - propagate to it
                propagateToTimestep(robot_idx, seg->segment_index, timestep, current_states[robot_idx]);
            } else {
                // Robot's path has ended - use final state (goal)
                const auto& last_segment = path_segments_[robot_idx].back();
                robots_[robot_idx]->getSpaceInformation()->copyState(
                    current_states[robot_idx], last_segment.end_state);
            }
        }

        // Check robot-robot collisions (only if 2+ robots)
        if (robots_.size() >= 2) {
            for (size_t i = 0; i < robots_.size(); ++i) {
                if (path_segments_[i].empty()) continue;

                for (size_t j = i + 1; j < robots_.size(); ++j) {
                    if (path_segments_[j].empty()) continue;

                    size_t part_i, part_j;
                    if (checkTwoRobotCollision(i, current_states[i], j, current_states[j], part_i, part_j)) {
                        // Record collision
                        SegmentCollision collision;
                        collision.type = SegmentCollision::ROBOT_ROBOT;
                        collision.robot_index_1 = i;
                        collision.robot_index_2 = j;
                        collision.timestep = timestep;
                        collision.part_index_1 = part_i;
                        collision.part_index_2 = part_j;

                        // Find which segment each robot is in at this timestep
                        const PathSegment* seg_i = findSegmentAtTimestep(i, timestep);
                        const PathSegment* seg_j = findSegmentAtTimestep(j, timestep);

                        collision.segment_index_1 = seg_i ? seg_i->segment_index : path_segments_[i].size() - 1;
                        collision.segment_index_2 = seg_j ? seg_j->segment_index : path_segments_[j].size() - 1;

                        segment_collisions_.push_back(collision);

#ifdef DBG_PRINTS
                        std::cout << "  Robot-robot collision: Robots " << i << " and " << j
                                  << " at timestep " << timestep
                                  << " (segments " << collision.segment_index_1 << ", " << collision.segment_index_2 << ")" << std::endl;
#endif
                    }
                }
            }
        }

        // Also check robot-environment collisions at this timestep
        // (backup check in case guided planner missed something)
        for (size_t robot_idx = 0; robot_idx < robots_.size(); ++robot_idx) {
            if (path_segments_[robot_idx].empty()) continue;

            auto robot = robots_[robot_idx];
            for (size_t part = 0; part < robot->numParts(); ++part) {
                const auto& transform = robot->getTransform(current_states[robot_idx], part);

                fcl::CollisionObjectf robot_co(robot->getCollisionGeometry(part));
                robot_co.setTranslation(transform.translation());
                robot_co.setRotation(transform.rotation());
                robot_co.computeAABB();

                fcl::DefaultCollisionData<float> collision_data;
                collision_manager_->collide(&robot_co, &collision_data,
                    fcl::DefaultCollisionFunction<float>);

                if (collision_data.result.isCollision()) {
                    // Record environment collision
                    SegmentCollision collision;
                    collision.type = SegmentCollision::ROBOT_OBSTACLE;
                    collision.robot_index_1 = robot_idx;
                    collision.robot_index_2 = robot_idx;  // Not used for obstacle collision
                    collision.timestep = timestep;
                    collision.part_index_1 = part;
                    collision.part_index_2 = 0;

                    const PathSegment* seg = findSegmentAtTimestep(robot_idx, timestep);
                    collision.segment_index_1 = seg ? seg->segment_index : path_segments_[robot_idx].size() - 1;
                    collision.segment_index_2 = collision.segment_index_1;

                    segment_collisions_.push_back(collision);

#ifdef DBG_PRINTS
                    std::cout << "  Robot-obstacle collision: Robot " << robot_idx
                              << " at timestep " << timestep << std::endl;
#endif
                }
            }
        }
    }

    // Free allocated states
    for (size_t i = 0; i < current_states.size(); ++i) {
        if (current_states[i]) {
            robots_[i]->getSpaceInformation()->getStateSpace()->freeState(current_states[i]);
        }
    }

#ifdef DBG_PRINTS
    std::cout << "  Found " << segment_collisions_.size() << " collision(s)" << std::endl;
#endif

    return !segment_collisions_.empty();
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
    std::cout << "  Subproblem robots: ";
    for (size_t idx : robot_indices) {
        std::cout << idx << " ";
    }
    std::cout << std::endl;

    // Get obstacles that overlap with the subproblem region
    auto subproblem_obstacles = getObstaclesInRegion(subproblem_env_min, subproblem_env_max);
    std::cout << "  Subproblem obstacles: " << subproblem_obstacles.size() << std::endl;

    if (config_.guided_planner_method == "db_rrt") {
        // Use CompositeDBRRT planner for DB-RRT guided planning
        std::cout << "Using composite DB-RRT planner on subproblem..." << std::endl;

        CompositeDBRRTProblem problem;
        problem.env_min = subproblem_env_min;
        problem.env_max = subproblem_env_max;
        problem.obstacles = subproblem_obstacles;
        problem.dynobench_obstacles = dynobench_obstacles_;

        for (size_t i = 0; i < robot_indices.size(); ++i) {
            size_t robot_idx = robot_indices[i];
            CompositeRobotSpec robot_spec;
            robot_spec.type = robot_types_[robot_idx];
            robot_spec.start = subproblem_starts[i];
            robot_spec.goal = subproblem_goals[i];
            problem.robots.push_back(robot_spec);
        }

        CompositeDBRRTPlanner planner(config_.composite_dbrrt_config);
        CompositeDBRRTResult composite_result = planner.plan(problem);

        std::cout << "Composite DB-RRT planning completed in " << composite_result.planning_time << " seconds" << std::endl;
        std::cout << "Solution found: " << (composite_result.solved ? "Yes" : "No") << std::endl;

        PlanningResult result;
        result.solved = composite_result.solved;
        result.planning_time = composite_result.planning_time;
        result.path = nullptr;

        if (composite_result.solved && composite_result.trajectories.size() == robot_indices.size()) {
            for (size_t i = 0; i < robot_indices.size(); ++i) {
                size_t robot_idx = robot_indices[i];
                auto path = convertDynobenchTrajectory(composite_result.trajectories[i], robots_[robot_idx]);
                result.individual_paths.push_back(path);

                std::cout << "  Robot " << robot_idx << ": trajectory with "
                          << composite_result.trajectories[i].states.size() << " states" << std::endl;
            }
        } else if (!composite_result.solved) {
            std::cout << "No solution found by composite DB-RRT planner" << std::endl;
        }

        return result;
    } else {
        // Use CoupledRRT planner for non-DB-RRT guided planning (e.g., syclop_rrt)
        std::cout << "Using coupled RRT planner on subproblem..." << std::endl;

        PlanningProblem problem;
        problem.env_min = subproblem_env_min;
        problem.env_max = subproblem_env_max;
        problem.obstacles = subproblem_obstacles;

        for (size_t i = 0; i < robot_indices.size(); ++i) {
            size_t robot_idx = robot_indices[i];
            RobotSpec robot_spec;
            robot_spec.type = robot_types_[robot_idx];
            robot_spec.start = subproblem_starts[i];
            robot_spec.goal = subproblem_goals[i];
            problem.robots.push_back(robot_spec);
        }

        CoupledRRTPlanner planner(config_.coupled_rrt_config);
        PlanningResult result = planner.plan(problem);

        std::cout << "Coupled RRT planning completed in " << result.planning_time << " seconds" << std::endl;
        std::cout << "Solution found: " << (result.solved ? "Yes" : "No") << std::endl;

        if (result.solved) {
            for (size_t i = 0; i < robot_indices.size(); ++i) {
                size_t robot_idx = robot_indices[i];
                if (i < result.individual_paths.size() && result.individual_paths[i]) {
                    std::cout << "  Robot " << robot_idx << ": trajectory with "
                              << result.individual_paths[i]->getStateCount() << " states" << std::endl;
                }
            }
        } else {
            std::cout << "No solution found by coupled RRT planner" << std::endl;
        }

        return result;
    }
}

bool MRSyCLoPPlanner::resolveCollisions()
{
    // Keep resolving collisions until none remain or a collision exhausts all strategies
    while (!segment_collisions_.empty()) {
        // Check for timeout before attempting to resolve each collision
        if (isTimeoutExceeded()) {
            std::cerr << "Planning timeout exceeded during collision resolution ("
                      << segment_collisions_.size() << " collisions remaining)" << std::endl;

            // Log the collision we were about to attempt as timed out
            CollisionResolutionEntry entry;
            entry.collision_number = resolution_stats_.total_collisions_encountered + 1;
            entry.robot_1 = segment_collisions_[0].robot_index_1;
            entry.robot_2 = segment_collisions_[0].robot_index_2;
            entry.timestep = segment_collisions_[0].timestep;
            entry.resolved = false;
            entry.outcome = "timeout";
            resolution_stats_.collision_log.push_back(entry);
            return false;
        }

        resolution_stats_.total_collisions_encountered++;

        SegmentCollision collision = segment_collisions_[0];

        // Create log entry for this collision
        CollisionResolutionEntry entry;
        entry.collision_number = resolution_stats_.total_collisions_encountered;
        entry.robot_1 = collision.robot_index_1;
        entry.robot_2 = collision.robot_index_2;
        entry.timestep = collision.timestep;

        std::cout << "Resolving collision " << resolution_stats_.total_collisions_encountered << ": Robots "
                  << collision.robot_index_1 << " and " << collision.robot_index_2
                  << " at timestep " << collision.timestep << std::endl;

        bool resolved = resolveCollisionWithStrategies(collision, entry);

        // Finalize the entry
        entry.resolved = resolved;
        if (resolved) {
            entry.outcome = "resolved";
        } else if (isTimeoutExceeded()) {
            entry.outcome = "timeout";
        } else {
            entry.outcome = "strategies_exhausted";
        }
        resolution_stats_.collision_log.push_back(entry);

        if (!resolved) {
            std::cerr << "Failed to resolve collision " << entry.collision_number
                      << " (outcome: " << entry.outcome << ", "
                      << entry.attempts.size() << " strategy attempts made)" << std::endl;
            return false;  // Planning failed - problem may be too hard
        }
        resolution_stats_.total_collisions_resolved++;
    }

    std::cout << "All collisions resolved successfully after " << resolution_stats_.total_collisions_resolved << " collision resolutions" << std::endl;
    return true;
}

bool MRSyCLoPPlanner::isTimeoutExceeded() const
{
    if (config_.max_total_time <= 0.0) {
        return false;  // No timeout configured
    }

    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - planning_start_time_).count();
    return elapsed >= config_.max_total_time;
}

MRSyCLoPResult MRSyCLoPPlanner::plan()
{
    if (!problem_loaded_) {
        throw std::runtime_error("Problem not loaded. Call loadProblem() first.");
    }

    MRSyCLoPResult result;
    planning_start_time_ = std::chrono::steady_clock::now();

    try {
        // Phase 1: Compute high-level paths over decomposition
        std::cout << "[Phase 1] Computing high-level paths..." << std::endl;
        computeHighLevelPaths();
        std::cout << "[Phase 1] High-level paths computed" << std::endl;

        if (isTimeoutExceeded()) {
            std::cerr << "Planning timeout exceeded after computing high-level paths" << std::endl;
            result.success = false;
            result.failure_reason = "timeout_high_level_paths";
            auto end_time = std::chrono::steady_clock::now();
            result.planning_time = std::chrono::duration<double>(end_time - planning_start_time_).count();
            result.resolution_stats = resolution_stats_;
            saveDecompositionToFile(decomp_, "final");
            return result;
        }

        // Phase 2: Compute guided low-level paths
        std::cout << "[Phase 2] Computing guided paths..." << std::endl;
        computeGuidedPaths();
        std::cout << "[Phase 2] Guided paths computed" << std::endl;

        if (isTimeoutExceeded()) {
            std::cerr << "Planning timeout exceeded after computing guided paths" << std::endl;
            result.success = false;
            result.failure_reason = "timeout_guided_paths";
            auto end_time = std::chrono::steady_clock::now();
            result.planning_time = std::chrono::duration<double>(end_time - planning_start_time_).count();
            result.resolution_stats = resolution_stats_;
            saveDecompositionToFile(decomp_, "final");
            return result;
        }

        // Phase 3: Segment guided paths
        std::cout << "[Phase 3] Segmenting guided paths..." << std::endl;
        segmentGuidedPaths();
        std::cout << "[Phase 3] Segmentation complete" << std::endl;

        // Phase 4: Check segments for collisions
        std::cout << "[Phase 4] Checking for collisions..." << std::endl;
        bool collisions_found = checkSegmentsForCollisions();
        std::cout << "[Phase 4] Collision check complete: "
                  << segment_collisions_.size() << " collisions found" << std::endl;

        // Phase 5: If collisions are found, resolve them
        if (collisions_found) {
            std::cout << "[Phase 5] Resolving collisions..." << std::endl;
            bool collisions_resolved = resolveCollisions();
            if (!collisions_resolved) {
                std::cerr << "Planning failed: could not resolve all collisions" << std::endl;
                result.success = false;
                if (isTimeoutExceeded()) {
                    result.failure_reason = "timeout_collision_resolution";
                } else {
                    result.failure_reason = "strategies_exhausted";
                }
            } else {
                std::cout << "[Phase 5] All collisions resolved" << std::endl;
                result.success = true;
            }
        } else {
            std::cout << "[Phase 5] No collisions to resolve" << std::endl;
            result.success = true;
        }

    } catch (const std::exception& e) {
        std::cerr << "Planning failed with exception: " << e.what() << std::endl;
        result.success = false;
        result.failure_reason = std::string("exception: ") + e.what();
    }

    auto end_time = std::chrono::steady_clock::now();
    result.planning_time = std::chrono::duration<double>(end_time - planning_start_time_).count();
    result.resolution_stats = resolution_stats_;

    // Always save final decomposition state for visualization (regardless of success/failure)
    saveDecompositionToFile(decomp_, "final");

    return result;
}

// ============================================================================
// Collision Resolution - Modular Strategy System
// ============================================================================

// Helper to check if a collision between the same robot pair at the same timestep persists
// A collision is considered "the same" only if it involves the same two robots AND same timestep
// If the collision is at a different timestep, it's a new collision that gets fresh strategy attempts
// For ROBOT_OBSTACLE collisions, robot_1 == robot_2, so we check if the same robot still has an obstacle collision
bool MRSyCLoPPlanner::collisionPersistsForRobots(size_t robot_1, size_t robot_2, int timestep) const
{
    for (const auto& coll : segment_collisions_) {
        if (coll.timestep == timestep) {
            if (coll.type == SegmentCollision::ROBOT_ROBOT) {
                if ((coll.robot_index_1 == robot_1 && coll.robot_index_2 == robot_2) ||
                    (coll.robot_index_1 == robot_2 && coll.robot_index_2 == robot_1)) {
                    return true;
                }
            } else if (coll.type == SegmentCollision::ROBOT_OBSTACLE) {
                // For obstacle collisions, robot_1 == robot_2
                if (coll.robot_index_1 == robot_1) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool MRSyCLoPPlanner::resolveCollisionWithStrategies(const SegmentCollision& collision,
                                                      CollisionResolutionEntry& log_entry)
{
    const auto& config = config_.collision_resolution_config;

    // Calculate max expansion layers if auto-detect
    int max_expansion_layers = config.max_expansion_layers;
    if (max_expansion_layers < 0) {
        max_expansion_layers = calculateMaxExpansionLayers();
    }

    // Cycle detection: track collision counts per robot pair and escalate
    // the minimum expansion layer when the same pair keeps colliding
    int min_expansion_layer = 0;
    if (config.escalation_frequency > 0) {
        auto pair_key = std::make_tuple(
            std::min(collision.robot_index_1, collision.robot_index_2),
            std::max(collision.robot_index_1, collision.robot_index_2),
            collision.timestep);
        int pair_count = ++robot_pair_collision_counts_[pair_key];
        min_expansion_layer = (pair_count - 1) / config.escalation_frequency;
        min_expansion_layer = std::min(min_expansion_layer, max_expansion_layers);

        if (min_expansion_layer > 0) {
            std::cout << "  Cycle detection: robot pair (" << std::get<0>(pair_key) << ", "
                      << std::get<1>(pair_key) << ") at timestep " << std::get<2>(pair_key)
                      << " collision #" << pair_count
                      << ", escalating to min expansion layer " << min_expansion_layer
                      << std::endl;
        }
    }

    // Strategy 1: Hierarchical Expansion + Refinement
    // This combines the old decomposition refinement and subproblem expansion into one
    // unified hierarchical approach:
    // - expansion_layer=0: refine just the collision cell (K times)
    // - expansion_layer=1,2,...: expand to neighbors, then refine all cells (K times each)
    // - Continue until expansion covers the whole decomposition
    if (config.max_refinement_levels > 0) {
        std::cout << "  Trying hierarchical expansion+refinement (max "
                  << config.max_refinement_levels << " refinement levels, max "
                  << max_expansion_layers << " expansion layers, min "
                  << min_expansion_layer << " expansion layer)..." << std::endl;

        if (resolveWithHierarchicalExpansionRefinement(
                collision,
                config.max_refinement_levels,
                max_expansion_layers,
                min_expansion_layer,
                log_entry)) {
            std::cout << "  Hierarchical expansion+refinement resolved the collision" << std::endl;
            return true;
        }

        // Check if we timed out during hierarchical resolution
        if (isTimeoutExceeded()) {
            std::cerr << "  Timeout during hierarchical expansion+refinement" << std::endl;
            return false;
        }

        std::cout << "  Hierarchical expansion+refinement exhausted, escalating to local composite..." << std::endl;
    }

    // Strategy 2: Local Composite Planner (plans colliding robots jointly in local bounds)
    if (collision.type == SegmentCollision::ROBOT_ROBOT) {
        std::cout << "  Trying local composite planner (joint planning of colliding robots)..." << std::endl;

        if (resolveWithLocalCompositePlanner(collision, log_entry)) {
            std::cout << "  Local composite planner resolved the collision" << std::endl;
            return true;
        }

        // Check if we timed out during local composite planning
        if (isTimeoutExceeded()) {
            std::cerr << "  Timeout during local composite planner" << std::endl;
            return false;
        }

        std::cout << "  Local composite planner failed, escalating to full-problem composite..." << std::endl;
    }

    // Strategy 3: Full-Problem Composite Planner (ALL robots, original starts/goals)
    if (config.max_composite_attempts > 0) {
        std::cout << "  Trying full-problem composite planner (max "
                  << config.max_composite_attempts << " attempts)..." << std::endl;
        resolution_stats_.composite_planner_attempts++;

        if (resolveWithFullProblemCompositePlanner(config.max_composite_attempts, log_entry)) {
            std::cout << "  Full-problem composite planner resolved the collision" << std::endl;
            resolution_stats_.composite_planner_successes++;
            return true;
        }

        std::cerr << "  Full-problem composite planner failed after " << config.max_composite_attempts
                  << " attempts for collision at timestep " << collision.timestep << std::endl;
    }

    // All strategies exhausted - collision could not be resolved
    std::cerr << "  All collision resolution strategies exhausted for collision at timestep "
              << collision.timestep << " between robots " << collision.robot_index_1
              << " and " << collision.robot_index_2 << std::endl;
    return false;
}

// ============================================================================
// Helper Methods
// ============================================================================

oc::DecompositionPtr MRSyCLoPPlanner::createLocalDecomposition(
    int parent_region,
    double subdivision_factor)
{
    // Get parent region bounds
    auto grid_decomp = std::dynamic_pointer_cast<GridDecompositionImpl>(decomp_);
    const auto& parent_bounds = grid_decomp->getRegionBoundsPublic(parent_region);

    // Create local bounds
    ob::RealVectorBounds local_bounds(decomp_->getDimension());
    for (int i = 0; i < decomp_->getDimension(); ++i) {
        local_bounds.setLow(i, parent_bounds.low[i]);
        local_bounds.setHigh(i, parent_bounds.high[i]);
    }

    // Calculate refined region length
    double parent_side_length = parent_bounds.high[0] - parent_bounds.low[0];
    double refined_length = parent_side_length / subdivision_factor;

#ifdef DBG_PRINTS
    std::cout << "    Creating local decomposition: " << subdivision_factor << "x"
              << subdivision_factor << " grid (region_length=" << refined_length << ")" << std::endl;
#endif

    // Create and return local decomposition
    auto local_decomp = std::make_shared<GridDecompositionImpl>(
        static_cast<int>(subdivision_factor),
        decomp_->getDimension(),
        local_bounds);

    // Record the refinement in the hierarchy
    recordRefinement(parent_region, local_decomp);

    // Save decomposition to file for visualization
    saveDecompositionToFile(local_decomp, "local_region_" + std::to_string(parent_region));

    return local_decomp;
}

bool MRSyCLoPPlanner::extractReplanningBounds(
    const SegmentCollision& collision,
    int collision_region,
    PathUpdateInfo& update_info_1,
    PathUpdateInfo& update_info_2)
{
    size_t robot_1 = collision.robot_index_1;
    size_t robot_2 = collision.robot_index_2;

    // Helper lambda to extract bounds for one robot
    auto extractForRobot = [&](size_t robot_idx, PathUpdateInfo& info) -> bool {
        info.robot_index = robot_idx;

        auto si = robots_[robot_idx]->getSpaceInformation();
        ob::State* temp_state = si->getStateSpace()->allocState();

        int collision_ts = collision.timestep;

        // Check if robot has finished its path before the collision timestep.
        // In that case, the robot is stationary at its goal — no replanning needed.
        int path_end_timestep = 0;
        if (!path_segments_[robot_idx].empty()) {
            path_end_timestep = path_segments_[robot_idx].back().end_timestep;
        }

        if (collision_ts >= path_end_timestep) {
            // Robot is stationary at goal — entry and exit are both the goal state
            info.entry_state = si->getStateSpace()->allocState();
            info.exit_state = si->getStateSpace()->allocState();
            si->copyState(info.entry_state, goal_states_[robot_idx]);
            si->copyState(info.exit_state, goal_states_[robot_idx]);
            info.start_timestep = path_end_timestep;
            info.end_timestep = path_end_timestep;
            info.start_segment_idx = path_segments_[robot_idx].empty() ? 0 : path_segments_[robot_idx].size() - 1;
            info.end_segment_idx = info.start_segment_idx;
            si->getStateSpace()->freeState(temp_state);
#ifdef DBG_PRINTS
            std::cout << "        Robot " << robot_idx << " is stationary at goal (path ended at timestep "
                      << path_end_timestep << ", collision at " << collision_ts << ")" << std::endl;
#endif
            return true;
        }

        // Find entry to collision region (scan backwards)
        int entry_timestep = 0;
        size_t entry_segment = 0;
        bool found_entry = false;

        for (int t = collision_ts; t >= 0; --t) {
            const PathSegment* seg = findSegmentAtTimestep(robot_idx, t);
            if (!seg) break;

            propagateToTimestep(robot_idx, seg->segment_index, t, temp_state);
            int region = decomp_->locateRegion(temp_state);

            if (region != collision_region) {
                // Found entry point
                entry_timestep = t + 1;
                entry_segment = seg->segment_index;
                if (entry_timestep >= seg->end_timestep && seg->segment_index + 1 < path_segments_[robot_idx].size()) {
                    entry_segment = seg->segment_index + 1;
                }
                found_entry = true;
                break;
            }
        }

        if (!found_entry) {
            // Robot starts in collision region
            entry_timestep = 0;
            entry_segment = 0;
        }

        // Find exit from collision region (scan forwards)
        int exit_timestep = -1;
        size_t exit_segment = 0;
        bool found_exit = false;

        int max_timestep = path_end_timestep;

        for (int t = collision_ts; t < max_timestep; ++t) {
            const PathSegment* seg = findSegmentAtTimestep(robot_idx, t);
            if (!seg) break;

            propagateToTimestep(robot_idx, seg->segment_index, t, temp_state);
            int region = decomp_->locateRegion(temp_state);

            if (region != collision_region) {
                // Found exit point
                exit_timestep = t;
                exit_segment = seg->segment_index;
                found_exit = true;
                break;
            }
        }

        if (!found_exit) {
            // Robot's goal is in collision region (or robot has finished its path)
            exit_timestep = max_timestep;
            exit_segment = path_segments_[robot_idx].size() - 1;  // Use last valid segment index
        }

        // Allocate and set entry/exit states
        info.entry_state = si->getStateSpace()->allocState();
        info.exit_state = si->getStateSpace()->allocState();

        if (entry_timestep == 0) {
            si->copyState(info.entry_state, start_states_[robot_idx]);
        } else {
            propagateToTimestep(robot_idx, entry_segment, entry_timestep, info.entry_state);
        }

        if (exit_timestep >= max_timestep) {
            si->copyState(info.exit_state, goal_states_[robot_idx]);
        } else {
            propagateToTimestep(robot_idx, exit_segment, exit_timestep, info.exit_state);
        }

        info.start_timestep = entry_timestep;
        info.end_timestep = exit_timestep;
        info.start_segment_idx = entry_segment;
        info.end_segment_idx = exit_segment;

        si->getStateSpace()->freeState(temp_state);
        return true;
    };

    bool success_1 = extractForRobot(robot_1, update_info_1);
    bool success_2 = extractForRobot(robot_2, update_info_2);

    return success_1 && success_2;
}

void MRSyCLoPPlanner::recheckCollisionsFromTimestep(int start_timestep)
{
#ifdef DBG_PRINTS
    std::cout << "    Re-checking collisions from timestep " << start_timestep << std::endl;
#endif

    // Clear all collisions
    segment_collisions_.clear();

    // Find the maximum timestep across all robots
    int max_timestep = 0;
    for (const auto& robot_segments : path_segments_) {
        if (!robot_segments.empty()) {
            max_timestep = std::max(max_timestep, robot_segments.back().end_timestep);
        }
    }

    // Allocate states for each robot
    std::vector<ob::State*> current_states(robots_.size(), nullptr);
    for (size_t i = 0; i < robots_.size(); ++i) {
        current_states[i] = robots_[i]->getSpaceInformation()->getStateSpace()->allocState();
    }

    // Check collisions at each timestep
    for (int timestep = start_timestep; timestep < max_timestep; ++timestep) {
        // Propagate each robot to this timestep
        for (size_t robot_idx = 0; robot_idx < robots_.size(); ++robot_idx) {
            if (path_segments_[robot_idx].empty()) continue;

            // Find the segment that contains this timestep for this robot
            const PathSegment* seg = findSegmentAtTimestep(robot_idx, timestep);

            if (seg != nullptr) {
                propagateToTimestep(robot_idx, seg->segment_index, timestep,
                                    current_states[robot_idx]);
            } else {
                // Robot's path has ended, use final state
                const auto& last_segment = path_segments_[robot_idx].back();
                robots_[robot_idx]->getSpaceInformation()->copyState(
                    current_states[robot_idx], last_segment.end_state);
            }
        }

        // Check robot-robot collisions at this timestep
        for (size_t i = 0; i < robots_.size(); ++i) {
            if (path_segments_[i].empty()) continue;

            for (size_t j = i + 1; j < robots_.size(); ++j) {
                if (path_segments_[j].empty()) continue;

                size_t part_i, part_j;
                if (checkTwoRobotCollision(i, current_states[i], j,
                                           current_states[j], part_i, part_j)) {
                    // Found a collision - record it and stop
                    const PathSegment* seg_i = findSegmentAtTimestep(i, timestep);
                    const PathSegment* seg_j = findSegmentAtTimestep(j, timestep);

                    SegmentCollision coll;
                    coll.type = SegmentCollision::ROBOT_ROBOT;
                    coll.robot_index_1 = i;
                    coll.robot_index_2 = j;
                    coll.timestep = timestep;
                    coll.part_index_1 = part_i;
                    coll.part_index_2 = part_j;

                    if (seg_i != nullptr) {
                        coll.segment_index_1 = seg_i->segment_index;
                    } else if (!path_segments_[i].empty()) {
                        coll.segment_index_1 = path_segments_[i].size() - 1;
                    }

                    if (seg_j != nullptr) {
                        coll.segment_index_2 = seg_j->segment_index;
                    } else if (!path_segments_[j].empty()) {
                        coll.segment_index_2 = path_segments_[j].size() - 1;
                    }

                    segment_collisions_.push_back(coll);

                    // Only report first collision found
#ifdef DBG_PRINTS
                    std::cout << "    Found robot-robot collision at timestep " << timestep << std::endl;
#endif
                    goto cleanup;
                }
            }
        }

        // Check robot-environment collisions at this timestep
        for (size_t robot_idx = 0; robot_idx < robots_.size(); ++robot_idx) {
            if (path_segments_[robot_idx].empty()) continue;

            auto robot = robots_[robot_idx];
            for (size_t part = 0; part < robot->numParts(); ++part) {
                const auto& transform = robot->getTransform(current_states[robot_idx], part);

                fcl::CollisionObjectf robot_co(robot->getCollisionGeometry(part));
                robot_co.setTranslation(transform.translation());
                robot_co.setRotation(transform.rotation());
                robot_co.computeAABB();

                fcl::DefaultCollisionData<float> collision_data;
                collision_manager_->collide(&robot_co, &collision_data,
                    fcl::DefaultCollisionFunction<float>);

                if (collision_data.result.isCollision()) {
                    // Found an obstacle collision - record it and stop
                    const PathSegment* seg = findSegmentAtTimestep(robot_idx, timestep);

                    SegmentCollision coll;
                    coll.type = SegmentCollision::ROBOT_OBSTACLE;
                    coll.robot_index_1 = robot_idx;
                    coll.robot_index_2 = robot_idx;
                    coll.timestep = timestep;
                    coll.part_index_1 = part;
                    coll.part_index_2 = 0;

                    coll.segment_index_1 = seg ? seg->segment_index : path_segments_[robot_idx].size() - 1;
                    coll.segment_index_2 = coll.segment_index_1;

                    segment_collisions_.push_back(coll);

#ifdef DBG_PRINTS
                    std::cout << "    Found robot-obstacle collision at timestep " << timestep << std::endl;
#endif
                    goto cleanup;
                }
            }
        }
    }

cleanup:
    // Free allocated states
    for (size_t i = 0; i < current_states.size(); ++i) {
        if (current_states[i]) {
            robots_[i]->getSpaceInformation()->getStateSpace()->freeState(current_states[i]);
        }
    }

#ifdef DBG_PRINTS
    std::cout << "    Total collisions found: " << segment_collisions_.size() << std::endl;
#endif
}

int MRSyCLoPPlanner::getRecheckStartTimestep(const SegmentCollision& collision)
{
    // Find the segment start timestep for each robot involved
    // Use the minimum of the two to ensure we catch all potential collisions
    int segment_start_1 = collision.timestep;
    int segment_start_2 = collision.timestep;

    // For robot 1 - get the start of the current segment containing the collision
    if (!path_segments_[collision.robot_index_1].empty() &&
        collision.segment_index_1 < path_segments_[collision.robot_index_1].size()) {
        segment_start_1 = path_segments_[collision.robot_index_1][collision.segment_index_1].start_timestep;
    }

    // For robot 2 (only for robot-robot collisions)
    if (collision.type == SegmentCollision::ROBOT_ROBOT) {
        if (!path_segments_[collision.robot_index_2].empty() &&
            collision.segment_index_2 < path_segments_[collision.robot_index_2].size()) {
            segment_start_2 = path_segments_[collision.robot_index_2][collision.segment_index_2].start_timestep;
        }
    }

    // Default: use the start of the current segment (minimum of both robots)
    int result = std::min(segment_start_1, segment_start_2);

    // If flag is set, go back to the prior segment instead
    if (config_.collision_resolution_config.recheck_from_prior_segment) {
        int prior_start_1 = segment_start_1;
        int prior_start_2 = segment_start_2;

        // For robot 1
        if (collision.segment_index_1 > 0 &&
            collision.segment_index_1 - 1 < path_segments_[collision.robot_index_1].size()) {
            prior_start_1 = path_segments_[collision.robot_index_1][collision.segment_index_1 - 1].start_timestep;
        }

        // For robot 2 (only for robot-robot collisions)
        if (collision.type == SegmentCollision::ROBOT_ROBOT) {
            if (collision.segment_index_2 > 0 &&
                collision.segment_index_2 - 1 < path_segments_[collision.robot_index_2].size()) {
                prior_start_2 = path_segments_[collision.robot_index_2][collision.segment_index_2 - 1].start_timestep;
            }
        }

        result = std::min(prior_start_1, prior_start_2);
    }

#ifdef DBG_PRINTS
    std::cout << "    Recheck start: using segment start " << result
              << " (collision timestep was " << collision.timestep << ")" << std::endl;
#endif

    return result;
}

void MRSyCLoPPlanner::segmentSinglePath(
    size_t robot_idx,
    const std::shared_ptr<oc::PathControl>& path,
    int start_timestep_offset,
    std::vector<PathSegment>& segments)
{
    segments.clear();

    size_t num_controls = path->getControlCount();
    if (num_controls == 0) return;

    int current_timestep = start_timestep_offset;
    size_t segment_idx = 0;
    size_t control_idx = 0;

    while (control_idx < num_controls) {
        PathSegment segment;
        segment.robot_index = robot_idx;
        segment.segment_index = segment_idx;
        segment.start_timestep = current_timestep;
        segment.start_state = path->getState(control_idx);
        segment.total_duration = 0.0;

        int timesteps_in_segment = 0;

        // Accumulate controls up to segment_timesteps
        while (control_idx < num_controls && timesteps_in_segment < config_.segment_timesteps) {
            double control_duration = path->getControlDuration(control_idx);
            int control_timesteps = static_cast<int>(std::ceil(control_duration));

            int remaining_timesteps = config_.segment_timesteps - timesteps_in_segment;

            if (control_timesteps <= remaining_timesteps) {
                // Full control fits
                segment.controls.push_back(path->getControl(control_idx));
                segment.control_durations.push_back(control_duration);
                segment.total_duration += control_duration;
                timesteps_in_segment += control_timesteps;
                control_idx++;
            } else {
                // Partial control
                double partial_duration = remaining_timesteps;
                segment.controls.push_back(path->getControl(control_idx));
                segment.control_durations.push_back(partial_duration);
                segment.total_duration += partial_duration;
                timesteps_in_segment += remaining_timesteps;
                // Note: in reality we'd need to split the control, but for simplicity we take full control
                control_idx++;
                break;
            }
        }

        segment.end_timestep = current_timestep + timesteps_in_segment;
        current_timestep = segment.end_timestep;
        segment.end_state = path->getState(control_idx < num_controls ? control_idx : num_controls);

        segments.push_back(segment);
        segment_idx++;
    }
}

void MRSyCLoPPlanner::integrateRefinedPaths(
    const std::vector<size_t>& robot_indices,
    const std::vector<mr_syclop::GuidedPlanningResult>& local_results,
    const PathUpdateInfo& update_info_1,
    const PathUpdateInfo& update_info_2)
{
    std::vector<PathUpdateInfo> update_infos = {update_info_1, update_info_2};

    for (size_t i = 0; i < robot_indices.size(); ++i) {
        size_t robot_idx = robot_indices[i];
        const auto& result = local_results[i];
        const auto& update_info = update_infos[i];

#ifdef DBG_PRINTS
        std::cout << "    Integrating refined path for robot " << robot_idx << std::endl;
#endif

        // Note: We'll re-segment the entire path after splicing the PathControl below
        // This is necessary because segment state pointers become invalid after path updates

        // Splice the full PathControl for guided_planning_results
        // We need to construct a complete path: before + refined + after
        auto si = robots_[robot_idx]->getSpaceInformation();
        auto original_path = guided_planning_results_[robot_idx].path;

#ifdef DBG_PRINTS
        if (original_path) {
            std::cout << "      Original path has " << original_path->getStateCount()
                      << " states and " << original_path->getControlCount() << " controls" << std::endl;
        }
        if (result.path) {
            std::cout << "      Refined path has " << result.path->getStateCount()
                      << " states and " << result.path->getControlCount() << " controls" << std::endl;
        }
#endif

        if (original_path && result.path) {
            // Create a new path that combines the three parts
            auto spliced_path = std::make_shared<oc::PathControl>(si);

            // Part 1: Extract states/controls from original path up to entry state
            // Find which state in original path corresponds to entry
            size_t entry_state_idx = 0;
            for (size_t s = 0; s < original_path->getStateCount(); ++s) {
                if (si->getStateSpace()->distance(original_path->getState(s), update_info.entry_state) < 1e-3) {
                    entry_state_idx = s;
                    break;
                }
            }

#ifdef DBG_PRINTS
            std::cout << "      Entry state found at index " << entry_state_idx << std::endl;
#endif

            // Add states and controls before entry
            for (size_t s = 0; s < entry_state_idx && s < original_path->getControlCount(); ++s) {
                spliced_path->append(original_path->getState(s),
                                    original_path->getControl(s),
                                    original_path->getControlDuration(s));
            }

            // Part 2: Add the refined path
            for (size_t s = 0; s < result.path->getControlCount(); ++s) {
                spliced_path->append(result.path->getState(s),
                                    result.path->getControl(s),
                                    result.path->getControlDuration(s));
            }

            // Part 3: Extract states/controls from original path after exit state
            // Find which state in original path corresponds to exit
            size_t exit_state_idx = original_path->getStateCount() - 1;
            for (size_t s = entry_state_idx; s < original_path->getStateCount(); ++s) {
                if (si->getStateSpace()->distance(original_path->getState(s), update_info.exit_state) < 1e-3) {
                    exit_state_idx = s;
                    break;
                }
            }

            // Add states and controls after exit (if exit is not the final state)
            for (size_t s = exit_state_idx; s < original_path->getControlCount(); ++s) {
                spliced_path->append(original_path->getState(s),
                                    original_path->getControl(s),
                                    original_path->getControlDuration(s));
            }

            // Add final state
            spliced_path->append(original_path->getState(original_path->getStateCount() - 1));

#ifdef DBG_PRINTS
            std::cout << "      Spliced path has " << spliced_path->getStateCount()
                      << " states and " << spliced_path->getControlCount() << " controls" << std::endl;
#endif

            // Update the guided planning result with the spliced path
            guided_planning_results_[robot_idx].path = spliced_path;

            // Re-segment the entire updated path from scratch
            std::vector<PathSegment> new_segments;
            segmentSinglePath(robot_idx, spliced_path, 0, new_segments);
            path_segments_[robot_idx] = new_segments;

#ifdef DBG_PRINTS
            std::cout << "      Re-segmented path has " << new_segments.size() << " segments" << std::endl;
#endif
        } else {
            // If we can't splice, fall back to just using the refined path
            guided_planning_results_[robot_idx] = result;

            // Re-segment the refined path
            std::vector<PathSegment> new_segments;
            segmentSinglePath(robot_idx, result.path, 0, new_segments);
            path_segments_[robot_idx] = new_segments;
        }
    }
}

// ============================================================================
// Hierarchical Collision Resolution Strategy
// ============================================================================

int MRSyCLoPPlanner::calculateMaxExpansionLayers() const
{
    // For a grid decomposition of NxN, the maximum useful expansion from center is N/2
    // This ensures we don't expand beyond the grid boundaries pointlessly
    int num_regions = decomp_->getNumRegions();
    int grid_side = static_cast<int>(std::sqrt(num_regions));
    return (grid_side + 1) / 2;  // ceil(grid_side / 2)
}

bool MRSyCLoPPlanner::expansionCoversFullDecomposition(int expansion_layers) const
{
    int num_regions = decomp_->getNumRegions();

    // For a 2D grid, expansion by L layers from center gives at most (2L+1)^2 cells
    // If (2L+1)^2 >= num_regions, we've covered everything
    int max_cells_in_expansion = (2 * expansion_layers + 1) * (2 * expansion_layers + 1);

    return max_cells_in_expansion >= num_regions;
}

void MRSyCLoPPlanner::freeUpdateInfoStates(
    size_t robot_1, size_t robot_2,
    PathUpdateInfo& update_info_1, PathUpdateInfo& update_info_2)
{
    robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(update_info_1.entry_state);
    robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(update_info_1.exit_state);
    robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(update_info_2.entry_state);
    robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(update_info_2.exit_state);
}

bool MRSyCLoPPlanner::extractReplanningBoundsForExpandedRegion(
    const SegmentCollision& collision,
    const std::vector<int>& expanded_regions,
    PathUpdateInfo& update_info_1,
    PathUpdateInfo& update_info_2)
{
    size_t robot_1 = collision.robot_index_1;
    size_t robot_2 = collision.robot_index_2;

    // Convert expanded_regions to a set for O(1) lookup
    std::set<int> region_set(expanded_regions.begin(), expanded_regions.end());

    // Helper lambda to extract bounds for one robot
    auto extractForRobot = [&](size_t robot_idx, PathUpdateInfo& info) -> bool {
        info.robot_index = robot_idx;

        auto si = robots_[robot_idx]->getSpaceInformation();
        ob::State* temp_state = si->getStateSpace()->allocState();

        int collision_ts = collision.timestep;

        // Check if robot has finished its path before the collision timestep.
        // In that case, the robot is stationary at its goal — no replanning needed.
        int path_end_timestep = 0;
        if (!path_segments_[robot_idx].empty()) {
            path_end_timestep = path_segments_[robot_idx].back().end_timestep;
        }

        if (collision_ts >= path_end_timestep) {
            // Robot is stationary at goal — entry and exit are both the goal state
            info.entry_state = si->getStateSpace()->allocState();
            info.exit_state = si->getStateSpace()->allocState();
            si->copyState(info.entry_state, goal_states_[robot_idx]);
            si->copyState(info.exit_state, goal_states_[robot_idx]);
            info.start_timestep = path_end_timestep;
            info.end_timestep = path_end_timestep;
            info.start_segment_idx = path_segments_[robot_idx].empty() ? 0 : path_segments_[robot_idx].size() - 1;
            info.end_segment_idx = info.start_segment_idx;
            si->getStateSpace()->freeState(temp_state);
#ifdef DBG_PRINTS
            std::cout << "        Robot " << robot_idx << " is stationary at goal (path ended at timestep "
                      << path_end_timestep << ", collision at " << collision_ts << ")" << std::endl;
#endif
            return true;
        }

        // Find entry to expanded region (scan backwards from collision)
        int entry_timestep = 0;
        size_t entry_segment = 0;
        bool found_entry = false;

        for (int t = collision_ts; t >= 0; --t) {
            const PathSegment* seg = findSegmentAtTimestep(robot_idx, t);
            if (!seg) break;

            propagateToTimestep(robot_idx, seg->segment_index, t, temp_state);
            int region = decomp_->locateRegion(temp_state);

            // Check if region is NOT in the expanded set
            if (region_set.find(region) == region_set.end()) {
                // Found entry point (one timestep after leaving the region set)
                entry_timestep = t + 1;
                entry_segment = seg->segment_index;
                if (entry_timestep >= seg->end_timestep &&
                    seg->segment_index + 1 < path_segments_[robot_idx].size()) {
                    entry_segment = seg->segment_index + 1;
                }
                found_entry = true;
                break;
            }
        }

        if (!found_entry) {
            // Robot starts in the expanded region
            entry_timestep = 0;
            entry_segment = 0;
        }

        // Find exit from expanded region (scan forwards)
        int exit_timestep = -1;
        size_t exit_segment = 0;
        bool found_exit = false;

        int max_timestep = path_end_timestep;

        for (int t = collision_ts; t < max_timestep; ++t) {
            const PathSegment* seg = findSegmentAtTimestep(robot_idx, t);
            if (!seg) break;

            propagateToTimestep(robot_idx, seg->segment_index, t, temp_state);
            int region = decomp_->locateRegion(temp_state);

            // Check if region is NOT in the expanded set
            if (region_set.find(region) == region_set.end()) {
                exit_timestep = t;
                exit_segment = seg->segment_index;
                found_exit = true;
                break;
            }
        }

        if (!found_exit) {
            // Robot's goal is in the expanded region (or robot has finished its path)
            exit_timestep = max_timestep;
            exit_segment = path_segments_[robot_idx].size() - 1;  // Use last valid segment index
        }

        // Allocate and set entry/exit states
        info.entry_state = si->getStateSpace()->allocState();
        info.exit_state = si->getStateSpace()->allocState();

        if (entry_timestep == 0) {
            si->copyState(info.entry_state, start_states_[robot_idx]);
        } else {
            propagateToTimestep(robot_idx, entry_segment, entry_timestep, info.entry_state);
        }

        if (exit_timestep >= max_timestep) {
            si->copyState(info.exit_state, goal_states_[robot_idx]);
        } else {
            propagateToTimestep(robot_idx, exit_segment, exit_timestep, info.exit_state);
        }

        info.start_timestep = entry_timestep;
        info.end_timestep = exit_timestep;
        info.start_segment_idx = entry_segment;
        info.end_segment_idx = exit_segment;

        si->getStateSpace()->freeState(temp_state);
        return true;
    };

    bool success_1 = extractForRobot(robot_1, update_info_1);
    bool success_2 = extractForRobot(robot_2, update_info_2);

    return success_1 && success_2;
}

bool MRSyCLoPPlanner::resolveWithHierarchicalExpansionRefinement(
    const SegmentCollision& collision,
    int max_refinement_levels,
    int max_expansion_layers,
    int min_expansion_layer,
    CollisionResolutionEntry& log_entry)
{
    size_t robot_1 = collision.robot_index_1;
    size_t robot_2 = collision.robot_index_2;

    // Allocate temporary states for collision location
    ob::State* state_1 = robots_[robot_1]->getSpaceInformation()->getStateSpace()->allocState();
    ob::State* state_2 = robots_[robot_2]->getSpaceInformation()->getStateSpace()->allocState();

    // Locate collision region - handle robots that have finished their paths (stationary at goal)
    const PathSegment* seg_1 = findSegmentAtTimestep(robot_1, collision.timestep);
    const PathSegment* seg_2 = findSegmentAtTimestep(robot_2, collision.timestep);

    // Get state for robot 1 at collision time
    if (seg_1) {
        propagateToTimestep(robot_1, seg_1->segment_index, collision.timestep, state_1);
    } else if (!path_segments_[robot_1].empty()) {
        // Robot has reached its goal - use final state
        const auto& last_seg = path_segments_[robot_1].back();
        robots_[robot_1]->getSpaceInformation()->copyState(state_1, last_seg.end_state);
    } else {
        std::cout << "    Robot " << robot_1 << " has no path segments" << std::endl;
        robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(state_1);
        robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(state_2);
        return false;
    }

    // Get state for robot 2 at collision time (needed for potential future use)
    if (seg_2) {
        propagateToTimestep(robot_2, seg_2->segment_index, collision.timestep, state_2);
    } else if (!path_segments_[robot_2].empty()) {
        // Robot has reached its goal - use final state
        const auto& last_seg = path_segments_[robot_2].back();
        robots_[robot_2]->getSpaceInformation()->copyState(state_2, last_seg.end_state);
    } else {
        std::cout << "    Robot " << robot_2 << " has no path segments" << std::endl;
        robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(state_1);
        robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(state_2);
        return false;
    }
    int collision_region = decomp_->locateRegion(state_1);

    std::cout << "    Collision in region " << collision_region << std::endl;

    robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(state_1);
    robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(state_2);

    // Outer loop: expansion layers (0 = just collision cell, 1 = 9 cells, 2 = 25 cells, etc.)
    // min_expansion_layer is set by cycle detection to skip small expansions that keep failing
    for (int expansion_layer = min_expansion_layer; expansion_layer <= max_expansion_layers; ++expansion_layer) {

        // Check for timeout before each expansion layer
        if (isTimeoutExceeded()) {
            std::cerr << "    Timeout during hierarchical expansion at layer " << expansion_layer << std::endl;
            return false;
        }

        // Check termination: expansion covers the whole decomposition
        if (expansion_layer > 0 && expansionCoversFullDecomposition(expansion_layer)) {
            std::cout << "    Expansion layer " << expansion_layer
                      << " covers entire decomposition, stopping expansion" << std::endl;
            break;
        }

        // Get expanded region for this layer
        std::vector<int> expanded_regions = getExpandedRegion(collision_region, expansion_layer);

        std::cout << "    Expansion layer " << expansion_layer
                  << ": " << expanded_regions.size() << " regions" << std::endl;

        // Try K refinement levels at this expansion level
        if (attemptRefinementAtExpansionLevel(
                collision,
                collision_region,
                expanded_regions,
                expansion_layer,
                max_refinement_levels,
                log_entry)) {
            return true;
        }

        // Check if we timed out during refinement attempts
        if (isTimeoutExceeded()) {
            std::cerr << "    Timeout during refinement at expansion layer " << expansion_layer << std::endl;
            return false;
        }

        // Refinement at this expansion level failed, expand further
        std::cout << "    All refinement levels exhausted at expansion layer " << expansion_layer
                  << ", trying wider expansion..." << std::endl;
    }

    // All expansion levels exhausted
    std::cout << "    All expansion layers exhausted" << std::endl;
    return false;
}

bool MRSyCLoPPlanner::attemptRefinementAtExpansionLevel(
    const SegmentCollision& collision,
    int /* collision_region */,
    const std::vector<int>& expanded_regions,
    int expansion_layer,
    int max_refinement_levels,
    CollisionResolutionEntry& log_entry)
{
    resolution_stats_.decomposition_refinement_attempts++;

    for (int refinement_level = 1; refinement_level <= max_refinement_levels; ++refinement_level) {
        // Check for timeout before each refinement attempt
        if (isTimeoutExceeded()) {
            std::cerr << "      Timeout before refinement level " << refinement_level
                      << " at expansion layer " << expansion_layer << std::endl;
            return false;
        }

        std::cout << "      Refinement level " << refinement_level << "/" << max_refinement_levels
                  << " at expansion layer " << expansion_layer << std::endl;

        StrategyAttempt attempt;
        attempt.strategy = "hierarchical_refinement";
        attempt.expansion_layer = expansion_layer;
        attempt.refinement_level = refinement_level;

        if (refineExpandedRegion(collision, expanded_regions, refinement_level)) {
            attempt.planning_succeeded = true;

            // Refinement and replanning succeeded, check if collision is resolved
            if (!collisionPersistsForRobots(collision.robot_index_1,
                                            collision.robot_index_2,
                                            collision.timestep)) {
                std::cout << "      Collision resolved at expansion=" << expansion_layer
                          << ", refinement=" << refinement_level << std::endl;
                attempt.collision_resolved = true;
                log_entry.attempts.push_back(attempt);
                resolution_stats_.decomposition_refinement_successes++;
                return true;
            }

            std::cout << "      Collision persists after refinement level " << refinement_level
                      << ", trying higher refinement..." << std::endl;
        } else {
            attempt.planning_succeeded = false;
            std::cout << "      Refinement level " << refinement_level << " failed (planning failed)" << std::endl;
        }

        log_entry.attempts.push_back(attempt);
    }

    return false;
}

bool MRSyCLoPPlanner::refineExpandedRegion(
    const SegmentCollision& collision,
    const std::vector<int>& expanded_regions,
    int refinement_level)
{
    size_t robot_1 = collision.robot_index_1;
    size_t robot_2 = collision.robot_index_2;

    // Calculate subdivision factor based on refinement level
    double subdivision_factor = std::pow(
        config_.collision_resolution_config.decomposition_subdivision_factor,
        refinement_level);

    // Create decomposition covering ALL cells in expanded region
    // Each cell will be refined to (subdivision_factor x subdivision_factor) sub-cells
    oc::DecompositionPtr local_decomp;

    if (expanded_regions.size() == 1) {
        // Single cell: use createLocalDecomposition
        local_decomp = createLocalDecomposition(expanded_regions[0], subdivision_factor);
    } else {
        // Multiple cells: use createMultiCellDecomposition
        // This handles refining each cell in the expanded region
        local_decomp = createMultiCellDecomposition(expanded_regions, subdivision_factor);
    }

#ifdef DBG_PRINTS
    std::cout << "        Created local decomposition with "
              << local_decomp->getNumRegions() << " sub-regions" << std::endl;
#endif

    // Extract replanning bounds for the expanded region
    PathUpdateInfo update_info_1, update_info_2;
    if (!extractReplanningBoundsForExpandedRegion(
            collision, expanded_regions, update_info_1, update_info_2)) {
#ifdef DBG_PRINTS
        std::cout << "        Failed to extract replanning bounds" << std::endl;
#endif
        return false;
    }

    // Determine which robots need replanning (stationary robots have entry == exit timestep)
    bool robot_1_stationary = (update_info_1.start_timestep == update_info_1.end_timestep);
    bool robot_2_stationary = (update_info_2.start_timestep == update_info_2.end_timestep);

    if (robot_1_stationary && robot_2_stationary) {
        // Both robots are stationary — can't replan either one
#ifdef DBG_PRINTS
        std::cout << "        Both robots are stationary at goal, cannot refine" << std::endl;
#endif
        freeUpdateInfoStates(robot_1, robot_2, update_info_1, update_info_2);
        return false;
    }

    // Build lists of robots that need replanning
    std::vector<size_t> replan_robot_indices;
    std::vector<ob::State*> replan_starts;
    std::vector<ob::State*> replan_goals;
    // Track which index in replan arrays corresponds to which collision robot (0=robot_1, 1=robot_2)
    std::vector<int> replan_to_collision_idx;

    if (!robot_1_stationary) {
        replan_robot_indices.push_back(robot_1);
        replan_starts.push_back(update_info_1.entry_state);
        replan_goals.push_back(update_info_1.exit_state);
        replan_to_collision_idx.push_back(0);
    }
    if (!robot_2_stationary) {
        replan_robot_indices.push_back(robot_2);
        replan_starts.push_back(update_info_2.entry_state);
        replan_goals.push_back(update_info_2.exit_state);
        replan_to_collision_idx.push_back(1);
    }

#ifdef DBG_PRINTS
    if (robot_1_stationary) {
        std::cout << "        Robot " << robot_1 << " is stationary at goal, only replanning robot " << robot_2 << std::endl;
    } else if (robot_2_stationary) {
        std::cout << "        Robot " << robot_2 << " is stationary at goal, only replanning robot " << robot_1 << std::endl;
    }
#endif

    // Validate that entry/exit states of robots to replan are within local decomposition bounds
    bool states_in_bounds = true;
    for (const auto* state : replan_starts) {
        if (local_decomp->locateRegion(state) < 0) {
            states_in_bounds = false;
#ifdef DBG_PRINTS
            std::cout << "        Entry state outside local decomposition bounds" << std::endl;
#endif
            break;
        }
    }
    if (states_in_bounds) {
        for (const auto* state : replan_goals) {
            if (local_decomp->locateRegion(state) < 0) {
                states_in_bounds = false;
#ifdef DBG_PRINTS
                std::cout << "        Exit state outside local decomposition bounds" << std::endl;
#endif
                break;
            }
        }
    }

    if (!states_in_bounds) {
        freeUpdateInfoStates(robot_1, robot_2, update_info_1, update_info_2);
        return false;
    }

    // MAPF replanning on the refined decomposition (only for robots that need replanning)
    auto mapf_solver = createMAPFSolver(
        config_.mapf_config.method,
        config_.mapf_config.region_capacity,
        config_.planning_time_limit);

    auto local_high_level_paths = mapf_solver->solve(
        local_decomp, replan_starts, replan_goals,
        obstacles_, config_.mapf_config.max_obstacle_volume_percent);

    if (local_high_level_paths.empty()) {
#ifdef DBG_PRINTS
        std::cout << "        MAPF failed" << std::endl;
#endif
        freeUpdateInfoStates(robot_1, robot_2, update_info_1, update_info_2);
        return false;
    }
    for (size_t i = 0; i < replan_robot_indices.size(); ++i) {
        if (i >= local_high_level_paths.size() || local_high_level_paths[i].empty()) {
#ifdef DBG_PRINTS
            std::cout << "        MAPF failed for robot " << replan_robot_indices[i] << std::endl;
#endif
            freeUpdateInfoStates(robot_1, robot_2, update_info_1, update_info_2);
            return false;
        }
    }

    // Guided planning (only for robots that need replanning)
    auto guided_planner = createGuidedPlannerWithDBRRT(
        config_.guided_planner_method,
        config_.guided_planner_config,
        config_.db_rrt_config,
        collision_manager_,
        dynobench_obstacles_);

    std::vector<mr_syclop::GuidedPlanningResult> replan_results;

    bool all_succeeded = true;
    for (size_t i = 0; i < replan_robot_indices.size(); ++i) {
        size_t robot_idx = replan_robot_indices[i];
        mr_syclop::GuidedPlanningResult result = guided_planner->solve(
            robots_[robot_idx],
            local_decomp,
            replan_starts[i],
            replan_goals[i],
            local_high_level_paths[i],
            robot_idx);

        replan_results.push_back(result);

        if (!result.success) {
            all_succeeded = false;
            break;
        }
    }

    if (!all_succeeded) {
#ifdef DBG_PRINTS
        std::cout << "        Guided planning failed" << std::endl;
#endif
        freeUpdateInfoStates(robot_1, robot_2, update_info_1, update_info_2);
        return false;
    }

    // Success! Integrate refined paths (only for robots that were replanned)
#ifdef DBG_PRINTS
    std::cout << "        Planning succeeded, integrating refined paths" << std::endl;
#endif

    // Build the full local_results array expected by integrateRefinedPaths
    // (must have entries for both robots in order: robot_1, robot_2)
    std::vector<size_t> all_robot_indices = {robot_1, robot_2};
    std::vector<mr_syclop::GuidedPlanningResult> all_local_results(2);

    // Fill in results for robots that were replanned
    size_t replan_idx = 0;
    for (size_t i = 0; i < 2; ++i) {
        bool is_stationary = (i == 0) ? robot_1_stationary : robot_2_stationary;
        if (is_stationary) {
            // Stationary robot: mark as success with null path (no change needed)
            all_local_results[i].success = true;
            all_local_results[i].path = nullptr;
            all_local_results[i].robot_index = all_robot_indices[i];
        } else {
            all_local_results[i] = replan_results[replan_idx++];
        }
    }

    // Only integrate paths for non-stationary robots
    for (size_t i = 0; i < 2; ++i) {
        if (all_local_results[i].path != nullptr) {
            std::vector<size_t> single_robot = {all_robot_indices[i]};
            std::vector<mr_syclop::GuidedPlanningResult> single_result = {all_local_results[i]};
            PathUpdateInfo& update_info = (i == 0) ? update_info_1 : update_info_2;
            // Use a dummy update_info for the second parameter (integrateRefinedPaths
            // only processes robots in the robot_indices vector)
            integrateRefinedPaths(single_robot, single_result, update_info, update_info);
        }
    }

    // Re-check collisions
    recheckCollisionsFromTimestep(getRecheckStartTimestep(collision));

    freeUpdateInfoStates(robot_1, robot_2, update_info_1, update_info_2);

    return true;
}


std::vector<int> MRSyCLoPPlanner::getExpandedRegion(int center_region, int expansion_layers)
{
    std::set<int> visited;
    std::queue<std::pair<int, int>> frontier;  // (region_id, distance)

    frontier.push({center_region, 0});
    visited.insert(center_region);

    while (!frontier.empty()) {
        auto [current_region, distance] = frontier.front();
        frontier.pop();

        if (distance < expansion_layers) {
            std::vector<int> neighbors;
            decomp_->getNeighbors(current_region, neighbors);

            for (int neighbor : neighbors) {
                if (visited.find(neighbor) == visited.end()) {
                    visited.insert(neighbor);
                    frontier.push({neighbor, distance + 1});
                }
            }
        }
    }

    return std::vector<int>(visited.begin(), visited.end());
}

void MRSyCLoPPlanner::computeExpandedBounds(
    const std::vector<int>& regions,
    std::vector<double>& env_min,
    std::vector<double>& env_max)
{
    if (regions.empty()) {
        return;
    }

    auto grid_decomp = std::dynamic_pointer_cast<GridDecompositionImpl>(decomp_);
    int dim = decomp_->getDimension();

    env_min.resize(dim);
    env_max.resize(dim);

    // Initialize with first region's bounds
    const auto& first_bounds = grid_decomp->getRegionBoundsPublic(regions[0]);
    for (int i = 0; i < dim; ++i) {
        env_min[i] = first_bounds.low[i];
        env_max[i] = first_bounds.high[i];
    }

    // Expand to include all regions
    for (size_t j = 1; j < regions.size(); ++j) {
        const auto& bounds = grid_decomp->getRegionBoundsPublic(regions[j]);
        for (int i = 0; i < dim; ++i) {
            env_min[i] = std::min(env_min[i], bounds.low[i]);
            env_max[i] = std::max(env_max[i], bounds.high[i]);
        }
    }
}

oc::DecompositionPtr MRSyCLoPPlanner::createMultiCellDecomposition(
    const std::vector<int>& regions,
    double subdivision_factor)
{
    // Compute bounding box of all regions
    std::vector<double> env_min, env_max;
    computeExpandedBounds(regions, env_min, env_max);

    ob::RealVectorBounds expanded_bounds(decomp_->getDimension());
    for (int i = 0; i < decomp_->getDimension(); ++i) {
        expanded_bounds.setLow(i, env_min[i]);
        expanded_bounds.setHigh(i, env_max[i]);
    }

    auto grid_decomp = std::dynamic_pointer_cast<GridDecompositionImpl>(decomp_);

    // Get original cell size
    const auto& first_bounds = grid_decomp->getRegionBoundsPublic(regions[0]);
    double cell_size = first_bounds.high[0] - first_bounds.low[0];

    // Calculate new cell size after subdivision
    double refined_cell_size = cell_size / subdivision_factor;

    // Calculate total grid length (number of cells per dimension)
    double total_span = env_max[0] - env_min[0];
    int grid_length = static_cast<int>(std::round(total_span / refined_cell_size));

#ifdef DBG_PRINTS
    std::cout << "      Multi-cell decomposition: " << grid_length << "x" << grid_length
              << " grid (" << (grid_length * grid_length) << " regions)" << std::endl;
#endif

    auto multi_cell_decomp = std::make_shared<GridDecompositionImpl>(
        grid_length,
        decomp_->getDimension(),
        expanded_bounds);

    // Record refinements for all regions involved in the multi-cell decomposition
    // Each original region maps to a subset of the new decomposition cells
    for (int region : regions) {
        recordRefinement(region, multi_cell_decomp);
    }

    // Save decomposition to file for visualization
    saveDecompositionToFile(multi_cell_decomp, "multi_cell");

    return multi_cell_decomp;
}

// ============================================================================
// Local Composite Planner (plans colliding robots jointly in local bounds)
// ============================================================================

bool MRSyCLoPPlanner::resolveWithLocalCompositePlanner(
    const SegmentCollision& collision,
    CollisionResolutionEntry& log_entry)
{
    size_t robot_1 = collision.robot_index_1;
    size_t robot_2 = collision.robot_index_2;

    std::cout << "    Local composite planner: jointly planning robots "
              << robot_1 << " and " << robot_2 << std::endl;

    // Check for timeout
    if (isTimeoutExceeded()) {
        std::cerr << "    Timeout before local composite planner" << std::endl;
        return false;
    }

    StrategyAttempt attempt;
    attempt.strategy = "local_composite";

    // Locate collision region and get expanded region for the subproblem
    // Handle robots that have finished their paths (stationary at goal)
    ob::State* temp_state = robots_[robot_1]->getSpaceInformation()->getStateSpace()->allocState();
    const PathSegment* seg_1 = findSegmentAtTimestep(robot_1, collision.timestep);
    if (seg_1) {
        propagateToTimestep(robot_1, seg_1->segment_index, collision.timestep, temp_state);
    } else if (!path_segments_[robot_1].empty()) {
        // Robot has reached its goal - use final state
        const auto& last_seg = path_segments_[robot_1].back();
        robots_[robot_1]->getSpaceInformation()->copyState(temp_state, last_seg.end_state);
    } else {
        robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(temp_state);
        std::cout << "    Robot " << robot_1 << " has no path segments" << std::endl;
        attempt.planning_succeeded = false;
        log_entry.attempts.push_back(attempt);
        return false;
    }
    int collision_region = decomp_->locateRegion(temp_state);
    robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(temp_state);

    // Use 2 layers of expansion around the collision region for local bounds
    std::vector<int> expanded_regions = getExpandedRegion(collision_region, 2);

    // Extract replanning bounds (entry/exit states) for both robots
    PathUpdateInfo update_info_1, update_info_2;
    if (!extractReplanningBoundsForExpandedRegion(
            collision, expanded_regions, update_info_1, update_info_2)) {
        std::cout << "    Failed to extract replanning bounds" << std::endl;
        attempt.planning_succeeded = false;
        log_entry.attempts.push_back(attempt);
        return false;
    }

    // Convert OMPL entry/exit states to std::vector<double>
    auto si_1 = robots_[robot_1]->getSpaceInformation();
    auto si_2 = robots_[robot_2]->getSpaceInformation();

    std::vector<double> start_1, goal_1, start_2, goal_2;
    si_1->getStateSpace()->copyToReals(start_1, update_info_1.entry_state);
    si_1->getStateSpace()->copyToReals(goal_1, update_info_1.exit_state);
    si_2->getStateSpace()->copyToReals(start_2, update_info_2.entry_state);
    si_2->getStateSpace()->copyToReals(goal_2, update_info_2.exit_state);

    // Compute local bounds from expanded region
    std::vector<double> local_env_min, local_env_max;
    computeExpandedBounds(expanded_regions, local_env_min, local_env_max);

    // Call useCompositePlanner to jointly plan both robots
    std::vector<size_t> robot_indices = {robot_1, robot_2};
    std::vector<std::vector<double>> subproblem_starts = {start_1, start_2};
    std::vector<std::vector<double>> subproblem_goals = {goal_1, goal_2};

    PlanningResult result = useCompositePlanner(
        robot_indices, subproblem_starts, subproblem_goals,
        local_env_min, local_env_max);

    if (result.solved && result.individual_paths.size() == 2) {
        attempt.planning_succeeded = true;
        std::cout << "    Local composite planning succeeded" << std::endl;

        // Convert PlanningResult individual paths to GuidedPlanningResult format
        std::vector<mr_syclop::GuidedPlanningResult> local_results;
        for (size_t i = 0; i < robot_indices.size(); ++i) {
            mr_syclop::GuidedPlanningResult guided_result;
            guided_result.success = true;
            guided_result.planning_time = result.planning_time;
            guided_result.robot_index = robot_indices[i];
            guided_result.path = result.individual_paths[i];
            local_results.push_back(guided_result);
        }

        // Integrate refined paths and re-check collisions
        integrateRefinedPaths(robot_indices, local_results, update_info_1, update_info_2);
        recheckCollisionsFromTimestep(getRecheckStartTimestep(collision));

        // Check if the collision is resolved
        if (!collisionPersistsForRobots(robot_1, robot_2, collision.timestep)) {
            std::cout << "    Local composite planner resolved the collision" << std::endl;
            attempt.collision_resolved = true;
            log_entry.attempts.push_back(attempt);
            freeUpdateInfoStates(robot_1, robot_2, update_info_1, update_info_2);
            return true;
        }

        std::cout << "    Local composite planner: collision persists after replanning" << std::endl;
    } else {
        attempt.planning_succeeded = false;
        std::cout << "    Local composite planner failed to find solution" << std::endl;
    }

    log_entry.attempts.push_back(attempt);
    freeUpdateInfoStates(robot_1, robot_2, update_info_1, update_info_2);
    return false;
}

// ============================================================================
// Full-Problem Composite Planner Fallback
// ============================================================================

bool MRSyCLoPPlanner::resolveWithFullProblemCompositePlanner(int max_attempts,
                                                             CollisionResolutionEntry& log_entry)
{
    std::cout << "    Full-problem composite planner: planning ALL "
              << robots_.size() << " robots from original starts to goals" << std::endl;

    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        // Check for timeout before each composite attempt
        if (isTimeoutExceeded()) {
            std::cerr << "    Timeout before composite planner attempt " << attempt << std::endl;
            return false;
        }

        std::cout << "    Full-problem composite attempt " << attempt << "/" << max_attempts << std::endl;

        StrategyAttempt sa;
        sa.strategy = "composite_planner";
        sa.attempt_number = attempt;

        // Build problem for ALL robots with original starts/goals
        std::vector<size_t> all_robot_indices;
        std::vector<std::vector<double>> all_starts;
        std::vector<std::vector<double>> all_goals;

        for (size_t i = 0; i < robots_.size(); ++i) {
            all_robot_indices.push_back(i);
            all_starts.push_back(starts_[i]);
            all_goals.push_back(goals_[i]);
        }

        // Use full environment bounds
        PlanningResult result = useCompositePlanner(
            all_robot_indices,
            all_starts,
            all_goals,
            env_min_,
            env_max_);

        if (result.solved && result.individual_paths.size() == robots_.size()) {
            sa.planning_succeeded = true;
            std::cout << "    Full-problem composite planner succeeded" << std::endl;

            // Replace ALL robot paths with the composite solution
            for (size_t i = 0; i < robots_.size(); ++i) {
                mr_syclop::GuidedPlanningResult guided_result;
                guided_result.success = true;
                guided_result.path = result.individual_paths[i];
                guided_result.planning_time = result.planning_time;
                guided_result.robot_index = i;
                guided_planning_results_[i] = guided_result;

                // Re-segment the entire path
                std::vector<PathSegment> new_segments;
                segmentSinglePath(i, result.individual_paths[i], 0, new_segments);
                path_segments_[i] = new_segments;

                std::cout << "      Robot " << i << ": replaced path with "
                          << result.individual_paths[i]->getStateCount() << " states, "
                          << new_segments.size() << " segments" << std::endl;
            }

            // Re-check ALL collisions from timestep 0
            recheckCollisionsFromTimestep(0);

            // If no collisions remain, we're done
            if (segment_collisions_.empty()) {
                std::cout << "    Full-problem composite planner resolved all collisions" << std::endl;
                sa.collision_resolved = true;
                log_entry.attempts.push_back(sa);
                return true;
            }

            std::cout << "    Full-problem composite: " << segment_collisions_.size()
                      << " collisions remain, trying next attempt..." << std::endl;
        } else {
            sa.planning_succeeded = false;
            std::cout << "    Full-problem composite attempt " << attempt << " failed to find solution" << std::endl;
        }

        log_entry.attempts.push_back(sa);
    }

    // Max attempts reached
    std::cerr << "    Full-problem composite planner failed: exhausted all " << max_attempts
              << " attempts" << std::endl;
    return false;
}

bool MRSyCLoPPlanner::extractIndividualPaths(
    const std::shared_ptr<oc::PathControl>& compound_path,
    std::vector<std::shared_ptr<oc::PathControl>>& individual_paths)
{
    // TODO: Implement
    return false;
}

void MRSyCLoPPlanner::exportDebugData(YAML::Node& output) const {
    // Export decomposition information
    YAML::Node decomp_node;
    decomp_node["type"] = "grid";

    auto grid_decomp = std::dynamic_pointer_cast<GridDecompositionImpl>(decomp_);
    if (grid_decomp) {
        // Use decomposition_region_length for grid size (this is the 'length' parameter
        // passed to GridDecomposition, representing cells per dimension)
        // Note: OMPL GridDecomposition uses uniform grids where length^dim = total regions
        int grid_length = config_.decomposition_region_length;

        YAML::Node grid_size_node;
        grid_size_node.push_back(grid_length);
        grid_size_node.push_back(grid_length);
        decomp_node["grid_size"] = grid_size_node;

        // Decomposition bounds
        YAML::Node bounds_node;
        YAML::Node bounds_min_node;
        bounds_min_node.push_back(env_min_[0]);
        bounds_min_node.push_back(env_min_[1]);
        bounds_node["min"] = bounds_min_node;

        YAML::Node bounds_max_node;
        bounds_max_node.push_back(env_max_[0]);
        bounds_max_node.push_back(env_max_[1]);
        bounds_node["max"] = bounds_max_node;
        decomp_node["bounds"] = bounds_node;

        // High-level paths (leads)
        YAML::Node leads_node;
        for (const auto& path : high_level_paths_) {
            YAML::Node path_node;
            for (int region_id : path) {
                path_node.push_back(region_id);
            }
            leads_node.push_back(path_node);
        }
        decomp_node["leads"] = leads_node;

        // Hierarchical decomposition structure
        // Format: [[1], [5, [9, 10, 11, 12], 7, 8], [3], [4]]
        // where a simple integer means no refinement, and an array means
        // [parent_region_id, [child_cells...]]
        decomp_node["hierarchy"] = serializeDecompositionHierarchy();
    }

    output["decomposition"] = decomp_node;

    // Export path segments information
    YAML::Node segments_node;
    for (size_t robot_idx = 0; robot_idx < path_segments_.size(); ++robot_idx) {
        YAML::Node robot_segments_node;
        for (const auto& segment : path_segments_[robot_idx]) {
            YAML::Node segment_node;
            segment_node["segment_index"] = static_cast<int>(segment.segment_index);
            segment_node["start_timestep"] = segment.start_timestep;
            segment_node["end_timestep"] = segment.end_timestep;
            segment_node["total_duration"] = segment.total_duration;

            // Extract start state
            if (segment.start_state) {
                std::vector<double> start_vals;
                robots_[robot_idx]->getSpaceInformation()->getStateSpace()->copyToReals(start_vals, segment.start_state);
                YAML::Node start_node;
                for (double val : start_vals) {
                    start_node.push_back(val);
                }
                segment_node["start_state"] = start_node;
            }

            // Extract end state
            if (segment.end_state) {
                std::vector<double> end_vals;
                robots_[robot_idx]->getSpaceInformation()->getStateSpace()->copyToReals(end_vals, segment.end_state);
                YAML::Node end_node;
                for (double val : end_vals) {
                    end_node.push_back(val);
                }
                segment_node["end_state"] = end_node;
            }

            robot_segments_node.push_back(segment_node);
        }
        segments_node.push_back(robot_segments_node);
    }
    output["segments"] = segments_node;

    // Export collision information
    YAML::Node collisions_node;
    for (const auto& collision : segment_collisions_) {
        YAML::Node collision_node;
        collision_node["type"] = (collision.type == SegmentCollision::ROBOT_ROBOT) ? "robot_robot" : "robot_obstacle";
        collision_node["robot_1"] = static_cast<int>(collision.robot_index_1);
        collision_node["robot_2"] = static_cast<int>(collision.robot_index_2);
        collision_node["segment_1"] = static_cast<int>(collision.segment_index_1);
        collision_node["segment_2"] = static_cast<int>(collision.segment_index_2);
        collision_node["timestep"] = collision.timestep;
        collision_node["part_1"] = static_cast<int>(collision.part_index_1);
        collision_node["part_2"] = static_cast<int>(collision.part_index_2);

        collisions_node.push_back(collision_node);
    }
    output["collisions"] = collisions_node;

    // Export guided paths (full continuous trajectories)
    YAML::Node guided_paths_node;
    for (size_t robot_idx = 0; robot_idx < guided_planning_results_.size(); ++robot_idx) {
        YAML::Node robot_path_node;
        const auto& result = guided_planning_results_[robot_idx];

        robot_path_node["success"] = result.success;
        robot_path_node["planning_time"] = result.planning_time;

        if (result.success && result.path) {
            auto path = result.path;
            auto si = robots_[robot_idx]->getSpaceInformation();

            // Export all states
            YAML::Node states_node;
            for (size_t i = 0; i < path->getStateCount(); ++i) {
                const auto state = path->getState(i);
                std::vector<double> state_vals;
                si->getStateSpace()->copyToReals(state_vals, state);

                YAML::Node state_node;
                for (double val : state_vals) {
                    state_node.push_back(val);
                }
                states_node.push_back(state_node);
            }
            robot_path_node["states"] = states_node;

            // Export control count and durations (control values require specific type knowledge)
            YAML::Node controls_node;
            for (size_t i = 0; i < path->getControlCount(); ++i) {
                YAML::Node control_node;
                control_node["duration"] = path->getControlDuration(i);
                controls_node.push_back(control_node);
            }
            robot_path_node["controls"] = controls_node;
        }

        guided_paths_node.push_back(robot_path_node);
    }
    output["guided_paths"] = guided_paths_node;
}

void MRSyCLoPPlanner::saveDecompositionToFile(
    const oc::DecompositionPtr& decomp,
    const std::string& label)
{
    // Skip if output directory not configured
    if (config_.decomposition_output_dir.empty()) {
        return;
    }

    auto grid_decomp = std::dynamic_pointer_cast<GridDecompositionImpl>(decomp);
    if (!grid_decomp) {
        std::cerr << "Warning: Cannot save decomposition - not a GridDecompositionImpl" << std::endl;
        return;
    }

    // Create filename with counter and optional label
    std::string filename;
    if (label.empty()) {
        filename = config_.decomposition_output_dir + "/decomposition_" +
                   std::to_string(decomposition_save_counter_) + ".yaml";
    } else {
        filename = config_.decomposition_output_dir + "/decomposition_" +
                   std::to_string(decomposition_save_counter_) + "_" + label + ".yaml";
    }
    decomposition_save_counter_++;

    YAML::Node output;

    // Basic decomposition info
    output["type"] = "grid";
    output["dimension"] = decomp->getDimension();
    output["num_regions"] = decomp->getNumRegions();

    // Compute grid length from number of regions (assuming square grid)
    int num_regions = decomp->getNumRegions();
    int dim = decomp->getDimension();
    int grid_length = static_cast<int>(std::round(std::pow(num_regions, 1.0 / dim)));
    output["grid_length"] = grid_length;

    // Overall bounds (from first and last region)
    YAML::Node bounds_node;
    const auto& first_bounds = grid_decomp->getRegionBoundsPublic(0);
    const auto& last_bounds = grid_decomp->getRegionBoundsPublic(num_regions - 1);

    YAML::Node bounds_min_node;
    YAML::Node bounds_max_node;
    for (int d = 0; d < dim; ++d) {
        bounds_min_node.push_back(first_bounds.low[d]);
        bounds_max_node.push_back(last_bounds.high[d]);
    }
    bounds_node["min"] = bounds_min_node;
    bounds_node["max"] = bounds_max_node;
    output["bounds"] = bounds_node;

    // Export all regions with their bounds
    YAML::Node regions_node;
    for (int rid = 0; rid < num_regions; ++rid) {
        const auto& region_bounds = grid_decomp->getRegionBoundsPublic(rid);

        YAML::Node region_node;
        region_node["id"] = rid;

        YAML::Node region_min_node;
        YAML::Node region_max_node;
        for (int d = 0; d < dim; ++d) {
            region_min_node.push_back(region_bounds.low[d]);
            region_max_node.push_back(region_bounds.high[d]);
        }
        region_node["min"] = region_min_node;
        region_node["max"] = region_max_node;

        // Also include neighbors
        std::vector<int> neighbors;
        grid_decomp->getNeighbors(rid, neighbors);
        YAML::Node neighbors_node;
        for (int neighbor : neighbors) {
            neighbors_node.push_back(neighbor);
        }
        region_node["neighbors"] = neighbors_node;

        regions_node.push_back(region_node);
    }
    output["regions"] = regions_node;

    // Write to file
    std::ofstream fout(filename);
    if (!fout.is_open()) {
        std::cerr << "Warning: Could not open file for decomposition output: " << filename << std::endl;
        return;
    }
    fout << output;
    fout.close();

#ifdef DBG_PRINTS
    std::cout << "  Saved decomposition to " << filename << std::endl;
#endif
}

// ============================================================================
// Decomposition Hierarchy Tracking
// ============================================================================

void MRSyCLoPPlanner::initializeDecompositionHierarchy()
{
    decomposition_hierarchy_.clear();

    auto grid_decomp = std::dynamic_pointer_cast<GridDecompositionImpl>(decomp_);
    if (!grid_decomp) {
        return;
    }

    int num_regions = decomp_->getNumRegions();
    decomposition_hierarchy_.reserve(num_regions);

    for (int rid = 0; rid < num_regions; ++rid) {
        DecompositionCell cell(rid);

        // Store bounds for this cell
        const auto& bounds = grid_decomp->getRegionBoundsPublic(rid);
        int dim = decomp_->getDimension();
        cell.bounds_min.resize(dim);
        cell.bounds_max.resize(dim);
        for (int d = 0; d < dim; ++d) {
            cell.bounds_min[d] = bounds.low[d];
            cell.bounds_max[d] = bounds.high[d];
        }

        decomposition_hierarchy_.push_back(cell);
    }

#ifdef DBG_PRINTS
    std::cout << "  Initialized decomposition hierarchy with " << num_regions << " cells" << std::endl;
#endif
}

DecompositionCell* MRSyCLoPPlanner::findCellByRegion(int region_id)
{
    // First check top-level cells
    for (auto& cell : decomposition_hierarchy_) {
        if (cell.region_id == region_id && !cell.isRefined()) {
            return &cell;
        }
        // Also search recursively in case we need to find a parent
        DecompositionCell* found = findCellByRegionRecursive(cell, region_id);
        if (found) {
            return found;
        }
    }
    return nullptr;
}

DecompositionCell* MRSyCLoPPlanner::findCellByRegionRecursive(DecompositionCell& cell, int region_id)
{
    if (cell.region_id == region_id) {
        return &cell;
    }

    for (auto& child : cell.children) {
        DecompositionCell* found = findCellByRegionRecursive(child, region_id);
        if (found) {
            return found;
        }
    }
    return nullptr;
}

void MRSyCLoPPlanner::recordRefinement(int parent_region, const oc::DecompositionPtr& local_decomp)
{
    // Find the cell to refine
    DecompositionCell* parent_cell = nullptr;

    // Search for the cell with this region_id that hasn't been refined yet
    for (auto& cell : decomposition_hierarchy_) {
        if (cell.region_id == parent_region && !cell.isRefined()) {
            parent_cell = &cell;
            break;
        }
    }

    if (!parent_cell) {
#ifdef DBG_PRINTS
        std::cout << "  Warning: Could not find cell " << parent_region << " for refinement" << std::endl;
#endif
        return;
    }

    auto grid_decomp = std::dynamic_pointer_cast<GridDecompositionImpl>(local_decomp);
    if (!grid_decomp) {
        return;
    }

    int num_sub_regions = local_decomp->getNumRegions();
    int dim = local_decomp->getDimension();

    // Create child cells for each sub-region
    for (int rid = 0; rid < num_sub_regions; ++rid) {
        DecompositionCell child_cell;
        // Use negative IDs to distinguish from global region IDs
        // Format: -(parent_region * 1000 + local_rid)
        child_cell.region_id = -(parent_region * 1000 + rid);

        // Store bounds
        const auto& bounds = grid_decomp->getRegionBoundsPublic(rid);
        child_cell.bounds_min.resize(dim);
        child_cell.bounds_max.resize(dim);
        for (int d = 0; d < dim; ++d) {
            child_cell.bounds_min[d] = bounds.low[d];
            child_cell.bounds_max[d] = bounds.high[d];
        }

        parent_cell->children.push_back(child_cell);
    }

#ifdef DBG_PRINTS
    std::cout << "  Recorded refinement: cell " << parent_region
              << " -> " << num_sub_regions << " sub-cells" << std::endl;
#endif
}

YAML::Node MRSyCLoPPlanner::serializeDecompositionHierarchy() const
{
    YAML::Node hierarchy_node;

    for (const auto& cell : decomposition_hierarchy_) {
        hierarchy_node.push_back(serializeCellRecursive(cell));
    }

    return hierarchy_node;
}

YAML::Node MRSyCLoPPlanner::serializeCellRecursive(const DecompositionCell& cell) const
{
    if (!cell.isRefined()) {
        // Leaf cell - just return the region ID
        return YAML::Node(cell.region_id);
    }

    // Refined cell - return an array where first element is the parent region ID
    // and subsequent elements are the children (which may themselves be arrays)
    YAML::Node cell_node;
    cell_node.push_back(cell.region_id);  // Parent region ID

    // Add children as a nested array
    YAML::Node children_node;
    for (const auto& child : cell.children) {
        children_node.push_back(serializeCellRecursive(child));
    }
    cell_node.push_back(children_node);

    return cell_node;
}

int main(int argc, char* argv[]) {

    // Parse command line arguments
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string outputFile;
    std::string cfgFile;
    double time_limit;

    desc.add_options()
      ("help", "produce help message")
      ("input,i", po::value<std::string>(&inputFile)->required(), "input file (yaml)")
      ("output,o", po::value<std::string>(&outputFile)->required(), "output file (yaml)")
      ("cfg,c", po::value<std::string>(&cfgFile)->required(), "configuration file (yaml)")
      ("timelimit,t", po::value<double>(&time_limit)->default_value(60.0), "time limit in seconds");

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

        // Load maximum total planning time (0 = no limit)
        if (cfg["max_total_time"]) {
            config.max_total_time = cfg["max_total_time"].as<double>();
        }

        // Override with command-line time limit if provided
        config.max_total_time = time_limit;

        // Load decomposition resolution
        if (cfg["decomposition"] && cfg["decomposition"]["resolution"]) {
            auto res = cfg["decomposition"]["resolution"];
            config.decomposition_resolution.clear();
            for (size_t i = 0; i < res.size(); i++) {
                config.decomposition_resolution.push_back(res[i].as<int>());
            }
        }

        // Load random seed
        if (cfg["seed"]) {
            config.seed = cfg["seed"].as<int>();
        }

        // Set the random seed
        if (config.seed >= 0) {
            std::cout << "  Setting random seed to: " << config.seed << std::endl;
            ompl::RNG::setSeed(config.seed);
        } else {
            std::cout << "  Using random seed" << std::endl;
        }

        // Load segmentation configuration
        if (cfg["segment_timesteps"]) {
            config.segment_timesteps = cfg["segment_timesteps"].as<int>();
        }

        // Load decomposition output directory
        if (cfg["decomposition_output_dir"]) {
            config.decomposition_output_dir = cfg["decomposition_output_dir"].as<std::string>();
        }

        // Load MAPF configuration
        if (cfg["mapf"]) {
            if (cfg["mapf"]["method"]) {
                config.mapf_config.method = cfg["mapf"]["method"].as<std::string>();
            }
            if (cfg["mapf"]["region_capacity"]) {
                config.mapf_config.region_capacity = cfg["mapf"]["region_capacity"].as<int>();
            }
            if (cfg["mapf"]["max_obstacle_volume_percent"]) {
                config.mapf_config.max_obstacle_volume_percent = cfg["mapf"]["max_obstacle_volume_percent"].as<double>();
            }
        }

        // Load guided planner configuration
        if (cfg["guided_planner_method"]) {
            config.guided_planner_method = cfg["guided_planner_method"].as<std::string>();
        }

        if (cfg["guided_planner"]) {
            const YAML::Node& gp = cfg["guided_planner"];
            if (gp["time_per_robot"]) {
                config.guided_planner_config.time_per_robot = gp["time_per_robot"].as<double>();
            }
            if (gp["debug"]) {
                config.guided_planner_config.debug = gp["debug"].as<bool>();
            }
            if (gp["goal_threshold"]) {
                config.guided_planner_config.goal_threshold = gp["goal_threshold"].as<double>();
            }
            if (gp["num_free_volume_samples"]) {
                config.guided_planner_config.num_free_volume_samples = gp["num_free_volume_samples"].as<int>();
            }
            if (gp["num_region_expansions"]) {
                config.guided_planner_config.num_region_expansions = gp["num_region_expansions"].as<int>();
            }
            if (gp["num_tree_expansions"]) {
                config.guided_planner_config.num_tree_expansions = gp["num_tree_expansions"].as<int>();
            }
            if (gp["prob_abandon_lead_early"]) {
                config.guided_planner_config.prob_abandon_lead_early = gp["prob_abandon_lead_early"].as<double>();
            }
            if (gp["prob_shortest_path"]) {
                config.guided_planner_config.prob_shortest_path = gp["prob_shortest_path"].as<double>();
            }
            if (gp["use_regional_nn"]) {
                config.guided_planner_config.use_regional_nn = gp["use_regional_nn"].as<bool>();
            }

            // Parse DB-RRT specific config
            if (gp["db_rrt"]) {
                const YAML::Node& db = gp["db_rrt"];
                if (db["motions_file"]) {
                    config.db_rrt_config.motions_file = db["motions_file"].as<std::string>();
                }
                if (db["models_base_path"]) {
                    config.db_rrt_config.models_base_path = db["models_base_path"].as<std::string>();
                }
                if (db["timelimit"]) {
                    config.db_rrt_config.timelimit = db["timelimit"].as<double>();
                }
                if (db["max_expands"]) {
                    config.db_rrt_config.max_expands = db["max_expands"].as<int>();
                }
                if (db["goal_region"]) {
                    config.db_rrt_config.goal_region = db["goal_region"].as<double>();
                }
                if (db["delta"]) {
                    config.db_rrt_config.delta = db["delta"].as<double>();
                }
                if (db["goal_bias"]) {
                    config.db_rrt_config.goal_bias = db["goal_bias"].as<double>();
                }
                if (db["max_motions"]) {
                    config.db_rrt_config.max_motions = db["max_motions"].as<int>();
                }
                if (db["seed"]) {
                    config.db_rrt_config.seed = db["seed"].as<int>();
                }
                if (db["do_optimization"]) {
                    config.db_rrt_config.do_optimization = db["do_optimization"].as<bool>();
                }
                if (db["use_nigh_nn"]) {
                    config.db_rrt_config.use_nigh_nn = db["use_nigh_nn"].as<bool>();
                }
                if (db["debug"]) {
                    config.db_rrt_config.debug = db["debug"].as<bool>();
                }
                if (db["solver_id"]) {
                    config.db_rrt_config.solver_id = db["solver_id"].as<int>();
                }
                if (db["use_region_guidance"]) {
                    config.db_rrt_config.use_region_guidance = db["use_region_guidance"].as<bool>();
                }
                if (db["region_bias"]) {
                    config.db_rrt_config.region_bias = db["region_bias"].as<double>();
                }
            }

            // Parse Composite DB-RRT config (for joint multi-robot planning)
            if (gp["composite_dbrrt"]) {
                const YAML::Node& cd = gp["composite_dbrrt"];
                if (cd["time_limit"]) {
                    config.composite_dbrrt_config.time_limit = cd["time_limit"].as<double>();
                }
                if (cd["delta"]) {
                    config.composite_dbrrt_config.delta = cd["delta"].as<double>();
                }
                if (cd["goal_region"]) {
                    config.composite_dbrrt_config.goal_region = cd["goal_region"].as<double>();
                }
                if (cd["max_motions"]) {
                    config.composite_dbrrt_config.max_motions = cd["max_motions"].as<size_t>();
                }
                if (cd["expansions_per_iter"]) {
                    config.composite_dbrrt_config.expansions_per_iter = cd["expansions_per_iter"].as<unsigned int>();
                }
                if (cd["goal_threshold"]) {
                    config.composite_dbrrt_config.goal_threshold = cd["goal_threshold"].as<double>();
                }
                if (cd["goal_bias"]) {
                    config.composite_dbrrt_config.goal_bias = cd["goal_bias"].as<double>();
                }
                if (cd["require_all_move"]) {
                    config.composite_dbrrt_config.require_all_move = cd["require_all_move"].as<bool>();
                }
                if (cd["models_base_path"]) {
                    config.composite_dbrrt_config.models_base_path = cd["models_base_path"].as<std::string>();
                }
                if (cd["seed"]) {
                    config.composite_dbrrt_config.seed = cd["seed"].as<int>();
                }
                if (cd["debug"]) {
                    config.composite_dbrrt_config.debug = cd["debug"].as<bool>();
                }
                // Parse motion files map
                if (cd["motion_files"]) {
                    for (const auto& mf : cd["motion_files"]) {
                        std::string robot_type = mf.first.as<std::string>();
                        std::string file_path = mf.second.as<std::string>();
                        config.composite_dbrrt_config.motion_files[robot_type] = file_path;
                    }
                }
            }
        }

        // Set coupled RRT config if needed
        config.coupled_rrt_config.goal_threshold = 0.5;
        config.coupled_rrt_config.min_control_duration = 1;
        config.coupled_rrt_config.max_control_duration = 10;
        config.coupled_rrt_config.use_geometric = true;

        // Parse collision resolution configuration
        if (cfg["collision_resolution"]) {
            const YAML::Node& cr = cfg["collision_resolution"];
            // New hierarchical refinement parameters
            if (cr["max_refinement_levels"]) {
                config.collision_resolution_config.max_refinement_levels = cr["max_refinement_levels"].as<int>();
            }
            // Legacy support: map old name to new
            if (cr["max_decomposition_attempts"]) {
                config.collision_resolution_config.max_refinement_levels = cr["max_decomposition_attempts"].as<int>();
            }
            if (cr["max_expansion_layers"]) {
                config.collision_resolution_config.max_expansion_layers = cr["max_expansion_layers"].as<int>();
            }
            if (cr["max_composite_attempts"]) {
                config.collision_resolution_config.max_composite_attempts = cr["max_composite_attempts"].as<int>();
            }
            if (cr["composite_uses_full_problem"]) {
                config.collision_resolution_config.composite_uses_full_problem = cr["composite_uses_full_problem"].as<bool>();
            }
            if (cr["decomposition_subdivision_factor"]) {
                config.collision_resolution_config.decomposition_subdivision_factor = cr["decomposition_subdivision_factor"].as<double>();
            }
            if (cr["recheck_from_prior_segment"]) {
                config.collision_resolution_config.recheck_from_prior_segment = cr["recheck_from_prior_segment"].as<bool>();
            }
            if (cr["escalation_frequency"]) {
                config.collision_resolution_config.escalation_frequency = cr["escalation_frequency"].as<int>();
            }
        }

        std::cout << "  Max total planning time: " << (config.max_total_time > 0 ? std::to_string(config.max_total_time) + "s" : "unlimited") << std::endl;
        std::cout << "  Decomposition region length: " << config.decomposition_region_length << std::endl;
        std::cout << "  Decomposition resolution: [" << config.decomposition_resolution[0] << ", "
                  << config.decomposition_resolution[1] << ", " << config.decomposition_resolution[2] << "]" << std::endl;
        std::cout << "  Decomposition output dir: " << (config.decomposition_output_dir.empty() ? "(disabled)" : config.decomposition_output_dir) << std::endl;
        std::cout << "  Segment timesteps: " << config.segment_timesteps << std::endl;
        std::cout << "  MAPF method: " << config.mapf_config.method << std::endl;
        std::cout << "  MAPF region capacity: " << config.mapf_config.region_capacity << std::endl;
        std::cout << "  Guided planner method: " << config.guided_planner_method << std::endl;
        std::cout << "  Guided planner time per robot: " << config.guided_planner_config.time_per_robot << "s" << std::endl;
        std::cout << "  DB-RRT motions file: " << config.db_rrt_config.motions_file << std::endl;
        std::cout << "  DB-RRT models path: " << config.db_rrt_config.models_base_path << std::endl;
        std::cout << "  MAPF max obstacle volume: " << (config.mapf_config.max_obstacle_volume_percent * 100.0) << "%" << std::endl;

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

        // Write the final plan to the output files
        YAML::Node output;
        output["success"] = result.success;
        output["planning_time"] = result.planning_time;

        // Add failure reason (empty string if successful)
        if (!result.failure_reason.empty()) {
            output["failure_reason"] = result.failure_reason;
        }

        // Add resolution statistics
        YAML::Node resolution_stats;
        resolution_stats["total_collisions_encountered"] = result.resolution_stats.total_collisions_encountered;
        resolution_stats["total_collisions_resolved"] = result.resolution_stats.total_collisions_resolved;

        YAML::Node strategy_attempts;
        strategy_attempts["decomposition_refinement"] = result.resolution_stats.decomposition_refinement_attempts;
        strategy_attempts["subproblem_expansion"] = result.resolution_stats.subproblem_expansion_attempts;
        strategy_attempts["composite_planner"] = result.resolution_stats.composite_planner_attempts;
        resolution_stats["strategy_attempts"] = strategy_attempts;

        YAML::Node strategy_successes;
        strategy_successes["decomposition_refinement"] = result.resolution_stats.decomposition_refinement_successes;
        strategy_successes["subproblem_expansion"] = result.resolution_stats.subproblem_expansion_successes;
        strategy_successes["composite_planner"] = result.resolution_stats.composite_planner_successes;
        resolution_stats["strategy_successes"] = strategy_successes;

        // Add per-collision resolution log
        YAML::Node collision_log_node;
        for (const auto& entry : result.resolution_stats.collision_log) {
            YAML::Node entry_node;
            entry_node["collision_number"] = entry.collision_number;
            entry_node["robot_1"] = static_cast<int>(entry.robot_1);
            entry_node["robot_2"] = static_cast<int>(entry.robot_2);
            entry_node["timestep"] = entry.timestep;
            entry_node["resolved"] = entry.resolved;
            entry_node["outcome"] = entry.outcome;

            YAML::Node attempts_node;
            for (const auto& attempt : entry.attempts) {
                YAML::Node attempt_node;
                attempt_node["strategy"] = attempt.strategy;
                if (attempt.expansion_layer >= 0) {
                    attempt_node["expansion_layer"] = attempt.expansion_layer;
                }
                if (attempt.refinement_level >= 0) {
                    attempt_node["refinement_level"] = attempt.refinement_level;
                }
                if (attempt.attempt_number >= 0) {
                    attempt_node["attempt_number"] = attempt.attempt_number;
                }
                attempt_node["planning_succeeded"] = attempt.planning_succeeded;
                attempt_node["collision_resolved"] = attempt.collision_resolved;
                attempts_node.push_back(attempt_node);
            }
            entry_node["attempts"] = attempts_node;

            collision_log_node.push_back(entry_node);
        }
        resolution_stats["collision_log"] = collision_log_node;

        output["resolution_stats"] = resolution_stats;

        // Export debug data (decomposition, segments, collisions, etc.) - always export, even on failure
        planner.exportDebugData(output);

        if (result.success) {
            YAML::Node result_node;
            const auto& guided_paths = planner.getGuidedPaths();
            const auto& robots = planner.getRobots();

            // For each robot (include all, even failed ones)
            for (size_t robot_idx = 0; robot_idx < guided_paths.size(); ++robot_idx) {
                YAML::Node robot_data;
                YAML::Node states_node;

                const auto& guided_result = guided_paths[robot_idx];

                // Only write states if this robot's planning succeeded
                if (guided_result.success && guided_result.path) {
                    auto path = guided_result.path;
                    auto si = robots[robot_idx]->getSpaceInformation();

                    // Extract all states from the path
                    for (size_t i = 0; i < path->getStateCount(); ++i) {
                        const auto state = path->getState(i);

                        // Convert OMPL state to vector of doubles
                        std::vector<double> state_vals;
                        si->getStateSpace()->copyToReals(state_vals, state);

                        // Write state values to YAML
                        YAML::Node state_node;
                        for (double val : state_vals) {
                            state_node.push_back(val);
                        }
                        states_node.push_back(state_node);
                    }
                }
                // If failed, states_node remains empty

                robot_data["states"] = states_node;
                result_node.push_back(robot_data);
            }

            output["result"] = result_node;
        }

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
