// Pinocchio/Crocoddyl must be included before Boost headers
#include <dynoplan/optimization/ocp.hpp>

#include "mr_syclop.h"

#include <iostream>
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

    // Create guided planner
    auto guided_planner = createGuidedPlannerWithDBRRT(
        config_.guided_planner_method,
        config_.guided_planner_config,
        config_.db_rrt_config,
        collision_manager_,
        dynobench_obstacles_);

    // Clear previous results
    guided_planning_results_.clear();

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

    // Find maximum segment count across all robots
    size_t max_segment_count = 0;
    for (const auto& robot_segments : path_segments_) {
        max_segment_count = std::max(max_segment_count, robot_segments.size());
    }

    if (max_segment_count == 0) {
        return false;  // No segments to check
    }

    // Allocate states for each robot (one per robot for current timestep)
    std::vector<ob::State*> current_states(robots_.size(), nullptr);
    for (size_t i = 0; i < robots_.size(); ++i) {
        current_states[i] = robots_[i]->getSpaceInformation()->getStateSpace()->allocState();
    }

    // Iterate over segment indices (NEW: segment-based approach)
    for (size_t seg_idx = 0; seg_idx < max_segment_count; ++seg_idx) {

        // Find the minimum duration among robots that have this segment
        int min_duration = INT_MAX;
        bool any_robot_has_segment = false;

        for (size_t robot_idx = 0; robot_idx < robots_.size(); ++robot_idx) {
            if (seg_idx < path_segments_[robot_idx].size()) {
                any_robot_has_segment = true;
                const auto& segment = path_segments_[robot_idx][seg_idx];
                int duration = segment.end_timestep - segment.start_timestep;
                min_duration = std::min(min_duration, duration);
            }
        }

        if (!any_robot_has_segment) continue;

        // Check each relative timestep within this segment index
        for (int rel_t = 0; rel_t < min_duration; ++rel_t) {

            // Propagate each robot to relative timestep rel_t within segment seg_idx
            for (size_t robot_idx = 0; robot_idx < robots_.size(); ++robot_idx) {
                if (seg_idx < path_segments_[robot_idx].size()) {
                    // Robot has this segment - propagate to timestep within this segment
                    const auto& segment = path_segments_[robot_idx][seg_idx];
                    int absolute_timestep = segment.start_timestep + rel_t;
                    propagateToTimestep(robot_idx, seg_idx, absolute_timestep, current_states[robot_idx]);
                } else if (!path_segments_[robot_idx].empty()) {
                    // Robot finished - use goal state (end of last segment)
                    const auto& last_segment = path_segments_[robot_idx].back();
                    robots_[robot_idx]->getSpaceInformation()->copyState(
                        current_states[robot_idx], last_segment.end_state);
                }
                // else: robot has no segments, skip
            }

            // Calculate absolute timestep for collision reporting
            // (use first robot that has this segment as reference)
            int absolute_timestep = rel_t;
            for (size_t robot_idx = 0; robot_idx < robots_.size(); ++robot_idx) {
                if (seg_idx < path_segments_[robot_idx].size()) {
                    absolute_timestep = path_segments_[robot_idx][seg_idx].start_timestep + rel_t;
                    break;
                }
            }

            // Note: Robot-environment collisions are guaranteed to be avoided by the
            // guided planner's state validity checker. We only check robot-robot
            // collisions here since individual planners don't coordinate with each other.

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
                            collision.timestep = absolute_timestep;
                            collision.part_index_1 = part_i;
                            collision.part_index_2 = part_j;

                            // Both robots are now at the same segment index
                            if (seg_idx < path_segments_[i].size()) {
                                collision.segment_index_1 = seg_idx;
                            } else if (!path_segments_[i].empty()) {
                                collision.segment_index_1 = path_segments_[i].size() - 1;
                            }

                            if (seg_idx < path_segments_[j].size()) {
                                collision.segment_index_2 = seg_idx;
                            } else if (!path_segments_[j].empty()) {
                                collision.segment_index_2 = path_segments_[j].size() - 1;
                            }

                            segment_collisions_.push_back(collision);

#ifdef DBG_PRINTS
                            std::cout << "  Robot-robot collision: Robots " << i << " and " << j
                                      << " at segment " << seg_idx
                                      << " (timestep " << absolute_timestep << ")" << std::endl;
#endif
                        }
                    }
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

bool MRSyCLoPPlanner::resolveCollisions()
{
    int total_attempts = 0;

    while (!segment_collisions_.empty() &&
           total_attempts < config_.collision_resolution_config.max_total_resolution_attempts) {
        total_attempts++;

        SegmentCollision collision = segment_collisions_[0];

#ifdef DBG_PRINTS
        std::cout << "Resolving collision " << total_attempts << ": Robots "
                  << collision.robot_index_1 << " and " << collision.robot_index_2
                  << " at timestep " << collision.timestep << std::endl;
#endif

        bool resolved = resolveCollisionWithStrategies(collision);

        if (!resolved) {
            std::cerr << "Failed to resolve collision after all strategy attempts" << std::endl;
            return false;  // Planning failed
        }
    }

    // Check if we hit the attempt limit without resolving all collisions
    if (!segment_collisions_.empty()) {
        std::cerr << "Max resolution attempts reached with " << segment_collisions_.size()
                  << " collisions remaining" << std::endl;
        return false;
    }

#ifdef DBG_PRINTS
    std::cout << "All collisions resolved successfully" << std::endl;
#endif
    return true;
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

        // Compute guided low-level paths
        computeGuidedPaths();

        // Segment guided paths
        segmentGuidedPaths();

        // Check segments for collisions
        bool collisions_found = checkSegmentsForCollisions();

        // If collisions are found, decide how to resolve them
        if (collisions_found) {
            bool collisions_resolved = resolveCollisions();
            if (!collisions_resolved) {
                std::cerr << "Planning failed: could not resolve all collisions" << std::endl;
                result.success = false;
            } else {
                result.success = true;
            }
        } else {
            result.success = true;
        }

    } catch (const std::exception& e) {
        std::cerr << "Planning failed: " << e.what() << std::endl;
        result.success = false;
    }

    auto end_time = std::chrono::steady_clock::now();
    result.planning_time = std::chrono::duration<double>(end_time - start_time).count();

    return result;
}

// ============================================================================
// Collision Resolution - Modular Strategy System
// ============================================================================

// Helper to check if a collision between the same robot pair at the same timestep persists
// A collision is considered "the same" only if it involves the same two robots AND same timestep
// If the collision is at a different timestep, it's a new collision that gets fresh strategy attempts
bool MRSyCLoPPlanner::collisionPersistsForRobots(size_t robot_1, size_t robot_2, int timestep) const
{
    for (const auto& coll : segment_collisions_) {
        if (coll.type == SegmentCollision::ROBOT_ROBOT && coll.timestep == timestep) {
            if ((coll.robot_index_1 == robot_1 && coll.robot_index_2 == robot_2) ||
                (coll.robot_index_1 == robot_2 && coll.robot_index_2 == robot_1)) {
                return true;
            }
        }
    }
    return false;
}

bool MRSyCLoPPlanner::resolveCollisionWithStrategies(const SegmentCollision& collision)
{
    const auto& config = config_.collision_resolution_config;

    // Track the collision we're trying to resolve
    size_t robot_1 = collision.robot_index_1;
    size_t robot_2 = collision.robot_index_2;
    int timestep = collision.timestep;

    // Strategy 1: Decomposition Refinement
    if (config.max_decomposition_attempts > 0) {
#ifdef DBG_PRINTS
        std::cout << "  Trying decomposition refinement (max "
                  << config.max_decomposition_attempts << " levels)..." << std::endl;
#endif

        if (resolveWithDecompositionRefinement(collision, config.max_decomposition_attempts)) {
            // Strategy completed without error - check if same collision persists
            if (!collisionPersistsForRobots(robot_1, robot_2, timestep)) {
#ifdef DBG_PRINTS
                std::cout << "  Decomposition refinement resolved the collision" << std::endl;
#endif
                return true;  // Collision actually resolved (or moved to different timestep)
            }
#ifdef DBG_PRINTS
            std::cout << "  Decomposition refinement completed but collision persists at same timestep, escalating..." << std::endl;
#endif
            // Same collision still exists - fall through to next strategy
        }
    }

    // Strategy 2: Subproblem Expansion
    if (config.max_subproblem_expansion_attempts > 0) {
#ifdef DBG_PRINTS
        std::cout << "  Trying subproblem expansion (max "
                  << config.max_subproblem_expansion_attempts << " attempts)..." << std::endl;
#endif

        if (resolveWithSubproblemExpansion(collision, config.max_subproblem_expansion_attempts)) {
            // Strategy completed without error - check if same collision persists
            if (!collisionPersistsForRobots(robot_1, robot_2, timestep)) {
#ifdef DBG_PRINTS
                std::cout << "  Subproblem expansion resolved the collision" << std::endl;
#endif
                return true;  // Collision actually resolved (or moved to different timestep)
            }
#ifdef DBG_PRINTS
            std::cout << "  Subproblem expansion completed but collision persists at same timestep, escalating..." << std::endl;
#endif
            // Same collision still exists - fall through to next strategy
        }
    }

    // Strategy 3: Composite Planner
    if (config.max_composite_attempts > 0) {
#ifdef DBG_PRINTS
        std::cout << "  Trying composite planner (max "
                  << config.max_composite_attempts << " attempts)..." << std::endl;
#endif

        if (resolveWithCompositePlanner(collision, config.max_composite_attempts)) {
            // Strategy completed without error - check if same collision persists
            if (!collisionPersistsForRobots(robot_1, robot_2, timestep)) {
#ifdef DBG_PRINTS
                std::cout << "  Composite planner resolved the collision" << std::endl;
#endif
                return true;  // Collision actually resolved (or moved to different timestep)
            }
#ifdef DBG_PRINTS
            std::cout << "  Composite planner completed but collision persists at same timestep" << std::endl;
#endif
        }
    }

    // All strategies failed or same collision still persists at same timestep
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
    return std::make_shared<GridDecompositionImpl>(
        static_cast<int>(subdivision_factor),
        decomp_->getDimension(),
        local_bounds);
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

        // Find entry to collision region (scan backwards)
        int entry_timestep = 0;
        size_t entry_segment = 0;
        bool found_entry = false;

        int collision_ts = collision.timestep;
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

        int max_timestep = 0;
        if (!path_segments_[robot_idx].empty()) {
            max_timestep = path_segments_[robot_idx].back().end_timestep;
        }

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
            // Robot's goal is in collision region
            exit_timestep = max_timestep;
            exit_segment = path_segments_[robot_idx].size();
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
                    std::cout << "    Found collision at timestep " << timestep << std::endl;
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
// Strategy 1: Decomposition Refinement
// ============================================================================

bool MRSyCLoPPlanner::resolveWithDecompositionRefinement(
    const SegmentCollision& collision,
    int max_attempts)
{
    // Extract collision state and locate region
    size_t robot_1 = collision.robot_index_1;
    size_t robot_2 = collision.robot_index_2;

    ob::State* state_1 = robots_[robot_1]->getSpaceInformation()->getStateSpace()->allocState();
    ob::State* state_2 = robots_[robot_2]->getSpaceInformation()->getStateSpace()->allocState();

    // Find the correct segments for the collision timestep (segments may have been updated)
    const PathSegment* seg_1 = findSegmentAtTimestep(robot_1, collision.timestep);
    const PathSegment* seg_2 = findSegmentAtTimestep(robot_2, collision.timestep);

    if (!seg_1 || !seg_2) {
        robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(state_1);
        robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(state_2);
        return false;
    }

    propagateToTimestep(robot_1, seg_1->segment_index, collision.timestep, state_1);
    propagateToTimestep(robot_2, seg_2->segment_index, collision.timestep, state_2);

    int collision_region = decomp_->locateRegion(state_1);

#ifdef DBG_PRINTS
    std::cout << "    Collision in region " << collision_region << std::endl;
#endif

    // Iterative refinement loop
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
#ifdef DBG_PRINTS
        std::cout << "    Refinement attempt " << attempt << "/" << max_attempts << std::endl;
#endif

        double subdivision_factor = std::pow(config_.collision_resolution_config.decomposition_subdivision_factor, attempt);

        // Create local decomposition
        oc::DecompositionPtr local_decomp = createLocalDecomposition(collision_region, subdivision_factor);

        // Extract replanning bounds
        PathUpdateInfo update_info_1, update_info_2;
        if (!extractReplanningBounds(collision, collision_region, update_info_1, update_info_2)) {
            robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(state_1);
            robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(state_2);
            return false;
        }

        // Validate that entry/exit states are within local decomposition bounds
        std::vector<ob::State*> local_starts = {update_info_1.entry_state, update_info_2.entry_state};
        std::vector<ob::State*> local_goals = {update_info_1.exit_state, update_info_2.exit_state};

        bool states_in_bounds = true;
        for (const auto* state : local_starts) {
            if (local_decomp->locateRegion(state) < 0) {
                states_in_bounds = false;
#ifdef DBG_PRINTS
                std::cout << "      Entry state outside local decomposition bounds" << std::endl;
#endif
                break;
            }
        }
        if (states_in_bounds) {
            for (const auto* state : local_goals) {
                if (local_decomp->locateRegion(state) < 0) {
                    states_in_bounds = false;
#ifdef DBG_PRINTS
                    std::cout << "      Exit state outside local decomposition bounds" << std::endl;
#endif
                    break;
                }
            }
        }

        if (!states_in_bounds) {
#ifdef DBG_PRINTS
            std::cout << "      States outside bounds, trying next refinement level" << std::endl;
#endif
            // Free states
            robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(update_info_1.entry_state);
            robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(update_info_1.exit_state);
            robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(update_info_2.entry_state);
            robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(update_info_2.exit_state);
            continue;
        }

        // MAPF replanning
        auto mapf_solver = createMAPFSolver(
            config_.mapf_config.method,
            config_.mapf_config.region_capacity,
            config_.planning_time_limit);

        auto local_high_level_paths = mapf_solver->solve(
            local_decomp, local_starts, local_goals,
            obstacles_, config_.mapf_config.max_obstacle_volume_percent);

        if (local_high_level_paths.empty() ||
            local_high_level_paths[0].empty() ||
            local_high_level_paths[1].empty()) {
#ifdef DBG_PRINTS
            std::cout << "      MAPF failed, trying next refinement level" << std::endl;
#endif
            // Free states
            robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(update_info_1.entry_state);
            robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(update_info_1.exit_state);
            robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(update_info_2.entry_state);
            robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(update_info_2.exit_state);
            continue;
        }

        // Guided planning
        auto guided_planner = createGuidedPlannerWithDBRRT(
            config_.guided_planner_method,
            config_.guided_planner_config,
            config_.db_rrt_config,
            collision_manager_,
            dynobench_obstacles_);

        std::vector<size_t> robot_indices = {robot_1, robot_2};
        std::vector<mr_syclop::GuidedPlanningResult> local_results;

        bool both_succeeded = true;
        for (size_t i = 0; i < robot_indices.size(); ++i) {
            size_t robot_idx = robot_indices[i];
            mr_syclop::GuidedPlanningResult result = guided_planner->solve(
                robots_[robot_idx],
                local_decomp,
                local_starts[i],
                local_goals[i],
                local_high_level_paths[i],
                robot_idx);

            local_results.push_back(result);

            if (!result.success) {
                both_succeeded = false;
                break;
            }
        }

        if (!both_succeeded) {
#ifdef DBG_PRINTS
            std::cout << "      Guided planning failed, trying next refinement level" << std::endl;
#endif
            // Free states
            robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(update_info_1.entry_state);
            robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(update_info_1.exit_state);
            robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(update_info_2.entry_state);
            robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(update_info_2.exit_state);
            continue;
        }

        // Success! Integrate refined paths
#ifdef DBG_PRINTS
        std::cout << "      Success at refinement level " << attempt << std::endl;
#endif

        integrateRefinedPaths(robot_indices, local_results, update_info_1, update_info_2);

        // Re-check collisions
        recheckCollisionsFromTimestep(collision.timestep);

        // Free states
        robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(update_info_1.entry_state);
        robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(update_info_1.exit_state);
        robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(update_info_2.entry_state);
        robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(update_info_2.exit_state);
        robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(state_1);
        robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(state_2);

        return true;
    }

    // Max attempts reached
    robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(state_1);
    robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(state_2);
    return false;
}

// ============================================================================
// Strategy 2: Subproblem Expansion (Stub)
// ============================================================================

bool MRSyCLoPPlanner::resolveWithSubproblemExpansion(
    const SegmentCollision& collision,
    int max_attempts)
{
    // Extract collision state and locate region
    size_t robot_1 = collision.robot_index_1;
    size_t robot_2 = collision.robot_index_2;

    ob::State* state_1 = robots_[robot_1]->getSpaceInformation()->getStateSpace()->allocState();
    ob::State* state_2 = robots_[robot_2]->getSpaceInformation()->getStateSpace()->allocState();

    // Find the correct segments for the collision timestep (segments may have been updated)
    const PathSegment* seg_1 = findSegmentAtTimestep(robot_1, collision.timestep);
    const PathSegment* seg_2 = findSegmentAtTimestep(robot_2, collision.timestep);

    if (!seg_1 || !seg_2) {
        robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(state_1);
        robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(state_2);
        return false;
    }

    propagateToTimestep(robot_1, seg_1->segment_index, collision.timestep, state_1);
    propagateToTimestep(robot_2, seg_2->segment_index, collision.timestep, state_2);

    int collision_region = decomp_->locateRegion(state_1);

#ifdef DBG_PRINTS
    std::cout << "    Collision in region " << collision_region << std::endl;
#endif

    // Get expanded region (collision cell + 8 neighbors)
    std::vector<int> expanded_regions = getExpandedRegion(collision_region, 1);

#ifdef DBG_PRINTS
    std::cout << "    Expanded to " << expanded_regions.size() << " regions" << std::endl;
#endif

    // Extract replanning bounds
    PathUpdateInfo update_info_1, update_info_2;
    if (!extractReplanningBounds(collision, collision_region, update_info_1, update_info_2)) {
        robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(state_1);
        robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(state_2);
        return false;
    }

    // Iterative expansion loop
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
#ifdef DBG_PRINTS
        std::cout << "    Expansion attempt " << attempt << "/" << max_attempts << std::endl;
#endif

        oc::DecompositionPtr planning_decomp;

        if (attempt == 1) {
            // Attempt 1: Use global decomposition (no cost modification for now - simpler approach)
#ifdef DBG_PRINTS
            std::cout << "      Using global decomposition" << std::endl;
#endif
            planning_decomp = decomp_;
        } else {
            // Attempt 2+: Decompose all 9 cells
            double subdivision_factor = std::pow(
                config_.collision_resolution_config.decomposition_subdivision_factor,
                attempt - 1);

#ifdef DBG_PRINTS
            std::cout << "      Creating multi-cell decomposition with factor "
                      << subdivision_factor << std::endl;
#endif

            planning_decomp = createMultiCellDecomposition(expanded_regions, subdivision_factor);
        }

        // MAPF replanning
        auto mapf_solver = createMAPFSolver(
            config_.mapf_config.method,
            config_.mapf_config.region_capacity,
            config_.planning_time_limit);

        std::vector<ob::State*> local_starts = {update_info_1.entry_state,
                                                 update_info_2.entry_state};
        std::vector<ob::State*> local_goals = {update_info_1.exit_state,
                                                update_info_2.exit_state};

        auto local_high_level_paths = mapf_solver->solve(
            planning_decomp, local_starts, local_goals,
            obstacles_, config_.mapf_config.max_obstacle_volume_percent);

        if (local_high_level_paths.empty() ||
            local_high_level_paths[0].empty() ||
            local_high_level_paths[1].empty()) {
#ifdef DBG_PRINTS
            std::cout << "      MAPF failed, trying next expansion attempt" << std::endl;
#endif
            continue;
        }

        // Guided planning
        auto guided_planner = createGuidedPlannerWithDBRRT(
            config_.guided_planner_method,
            config_.guided_planner_config,
            config_.db_rrt_config,
            collision_manager_,
            dynobench_obstacles_);

        std::vector<size_t> robot_indices = {robot_1, robot_2};
        std::vector<mr_syclop::GuidedPlanningResult> local_results;

        bool both_succeeded = true;
        for (size_t i = 0; i < robot_indices.size(); ++i) {
            size_t robot_idx = robot_indices[i];
            mr_syclop::GuidedPlanningResult result = guided_planner->solve(
                robots_[robot_idx],
                planning_decomp,
                local_starts[i],
                local_goals[i],
                local_high_level_paths[i],
                robot_idx);

            local_results.push_back(result);

            if (!result.success) {
                both_succeeded = false;
                break;
            }
        }

        if (!both_succeeded) {
#ifdef DBG_PRINTS
            std::cout << "      Guided planning failed, trying next expansion attempt" << std::endl;
#endif
            continue;
        }

        // Success! Integrate refined paths
#ifdef DBG_PRINTS
        std::cout << "      Success at expansion attempt " << attempt << std::endl;
#endif

        integrateRefinedPaths(robot_indices, local_results, update_info_1, update_info_2);

        // Re-check collisions
        recheckCollisionsFromTimestep(collision.timestep);

        // Free states
        robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(update_info_1.entry_state);
        robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(update_info_1.exit_state);
        robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(update_info_2.entry_state);
        robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(update_info_2.exit_state);
        robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(state_1);
        robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(state_2);

        return true;
    }

    // Max attempts reached
    robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(update_info_1.entry_state);
    robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(update_info_1.exit_state);
    robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(update_info_2.entry_state);
    robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(update_info_2.exit_state);
    robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(state_1);
    robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(state_2);

    return false;
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

    return std::make_shared<GridDecompositionImpl>(
        grid_length,
        decomp_->getDimension(),
        expanded_bounds);
}

// ============================================================================
// Strategy 3: Composite Planner (Stub)
// ============================================================================

bool MRSyCLoPPlanner::resolveWithCompositePlanner(
    const SegmentCollision& collision,
    int max_attempts)
{
    // Extract collision state and locate region
    size_t robot_1 = collision.robot_index_1;
    size_t robot_2 = collision.robot_index_2;

    ob::State* state_1 = robots_[robot_1]->getSpaceInformation()->getStateSpace()->allocState();
    ob::State* state_2 = robots_[robot_2]->getSpaceInformation()->getStateSpace()->allocState();

    // Find the correct segments for the collision timestep
    const PathSegment* seg_1 = findSegmentAtTimestep(robot_1, collision.timestep);
    const PathSegment* seg_2 = findSegmentAtTimestep(robot_2, collision.timestep);

    if (!seg_1 || !seg_2) {
        robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(state_1);
        robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(state_2);
        return false;
    }

    propagateToTimestep(robot_1, seg_1->segment_index, collision.timestep, state_1);
    propagateToTimestep(robot_2, seg_2->segment_index, collision.timestep, state_2);

    int collision_region = decomp_->locateRegion(state_1);

#ifdef DBG_PRINTS
    std::cout << "    Composite planner: Collision in region " << collision_region << std::endl;
#endif

    // Extract replanning bounds
    PathUpdateInfo update_info_1, update_info_2;
    if (!extractReplanningBounds(collision, collision_region, update_info_1, update_info_2)) {
        robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(state_1);
        robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(state_2);
        return false;
    }

    // Get expanded region bounds for the subproblem
    std::vector<int> expanded_regions = getExpandedRegion(collision_region, 1);
    std::vector<double> subproblem_env_min, subproblem_env_max;
    computeExpandedBounds(expanded_regions, subproblem_env_min, subproblem_env_max);

#ifdef DBG_PRINTS
    std::cout << "    Subproblem bounds: [" << subproblem_env_min[0] << ", " << subproblem_env_min[1]
              << "] to [" << subproblem_env_max[0] << ", " << subproblem_env_max[1] << "]" << std::endl;
#endif

    // Extract start/goal positions from entry/exit states
    std::vector<size_t> robot_indices = {robot_1, robot_2};
    std::vector<std::vector<double>> subproblem_starts(2);
    std::vector<std::vector<double>> subproblem_goals(2);

    // Robot 1 start/goal
    auto rvs_1 = update_info_1.entry_state->as<ob::RealVectorStateSpace::StateType>();
    auto rvg_1 = update_info_1.exit_state->as<ob::RealVectorStateSpace::StateType>();
    size_t dim_1 = robots_[robot_1]->getSpaceInformation()->getStateSpace()->getDimension();
    for (size_t d = 0; d < dim_1; ++d) {
        subproblem_starts[0].push_back(rvs_1->values[d]);
        subproblem_goals[0].push_back(rvg_1->values[d]);
    }

    // Robot 2 start/goal
    auto rvs_2 = update_info_2.entry_state->as<ob::RealVectorStateSpace::StateType>();
    auto rvg_2 = update_info_2.exit_state->as<ob::RealVectorStateSpace::StateType>();
    size_t dim_2 = robots_[robot_2]->getSpaceInformation()->getStateSpace()->getDimension();
    for (size_t d = 0; d < dim_2; ++d) {
        subproblem_starts[1].push_back(rvs_2->values[d]);
        subproblem_goals[1].push_back(rvg_2->values[d]);
    }

    // Attempt composite planning
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
#ifdef DBG_PRINTS
        std::cout << "    Composite planner attempt " << attempt << "/" << max_attempts << std::endl;
#endif

        PlanningResult result = useCompositePlanner(
            robot_indices,
            subproblem_starts,
            subproblem_goals,
            subproblem_env_min,
            subproblem_env_max);

        if (result.solved && result.path) {
#ifdef DBG_PRINTS
            std::cout << "    Composite planner succeeded" << std::endl;
#endif

            // Extract individual paths from the compound path
            std::vector<std::shared_ptr<oc::PathControl>> individual_paths;
            if (!extractIndividualPaths(result.path, individual_paths)) {
                std::cerr << "    Failed to extract individual paths from composite result" << std::endl;
                continue;
            }

            // Convert to GuidedPlanningResult format for integration
            std::vector<mr_syclop::GuidedPlanningResult> local_results;
            for (size_t i = 0; i < robot_indices.size(); ++i) {
                mr_syclop::GuidedPlanningResult guided_result;
                guided_result.success = true;
                guided_result.path = individual_paths[i];
                local_results.push_back(guided_result);
            }

            // Integrate refined paths
            integrateRefinedPaths(robot_indices, local_results, update_info_1, update_info_2);

            // Re-check collisions
            recheckCollisionsFromTimestep(collision.timestep);

            // Free states
            robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(update_info_1.entry_state);
            robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(update_info_1.exit_state);
            robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(update_info_2.entry_state);
            robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(update_info_2.exit_state);
            robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(state_1);
            robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(state_2);

            return true;
        }

#ifdef DBG_PRINTS
        std::cout << "    Composite planner attempt " << attempt << " failed" << std::endl;
#endif
    }

    // Max attempts reached - free states
    robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(update_info_1.entry_state);
    robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(update_info_1.exit_state);
    robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(update_info_2.entry_state);
    robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(update_info_2.exit_state);
    robots_[robot_1]->getSpaceInformation()->getStateSpace()->freeState(state_1);
    robots_[robot_2]->getSpaceInformation()->getStateSpace()->freeState(state_2);

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
        }

        // Set coupled RRT config if needed
        config.coupled_rrt_config.goal_threshold = 0.5;
        config.coupled_rrt_config.min_control_duration = 1;
        config.coupled_rrt_config.max_control_duration = 10;

        std::cout << "  Decomposition region length: " << config.decomposition_region_length << std::endl;
        std::cout << "  Decomposition resolution: [" << config.decomposition_resolution[0] << ", "
                  << config.decomposition_resolution[1] << ", " << config.decomposition_resolution[2] << "]" << std::endl;
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

            // Export debug data (decomposition, segments, collisions, etc.)
            planner.exportDebugData(output);
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
