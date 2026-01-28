/**
 * @file k_arc.cpp
 * @brief K-ARC: Kinodynamic Adaptive Robot Coordination - Implementation
 *
 * Implements the K-ARC algorithm for multi-robot kinodynamic planning.
 * Builds on existing infrastructure from ARC (initial kinematic planning),
 * DB-RRT solver (kinodynamic segment planning), and CompositeDBRRT
 * (joint multi-robot kinodynamic planning).
 */

// Pinocchio/Crocoddyl must be included before Boost/OMPL headers
#include <dynoplan/optimization/ocp.hpp>

#include "k_arc.h"
#include "robots.h"
#include "fclStateValidityChecker.hpp"
#include "robotStatePropagator.hpp"

// OMPL - Geometric Planning (for initial kinematic paths)
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/util/RandomNumbers.h>

// Forward declaration for loadDBRRTConfig (defined in db_rrt_solver.cpp,
// but db_rrt_solver.h cannot be included here due to dbrrt.hpp lacking include guards)
namespace mr_syclop {
    DBRRTConfig loadDBRRTConfig(const YAML::Node& node);
}

// Standard library
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>

// YAML for config loading
#include <yaml-cpp/yaml.h>

// ============================================================================
// Simple geometric propagator (for RRTConnect with control-based robots)
// ============================================================================

namespace {

class GeometricPropagator : public oc::StatePropagator {
public:
    GeometricPropagator(const oc::SpaceInformationPtr& si)
        : oc::StatePropagator(si) {}

    void propagate(const ob::State* state, const oc::Control* /*control*/,
                  double /*duration*/, ob::State* result) const override {
        si_->getStateSpace()->copyState(result, state);
    }
};

} // anonymous namespace

// ============================================================================
// Constructor / Destructor
// ============================================================================

KARCPlanner::KARCPlanner(const KARCConfig& config)
    : config_(config)
    , position_bounds_(2)
{
    if (config_.seed >= 0) {
        rng_.seed(config_.seed);
    } else {
        rng_.seed(std::random_device{}());
    }
}

KARCPlanner::~KARCPlanner()
{
    cleanup();
}

// ============================================================================
// Main Planning Function (Algorithm 1 from K-ARC paper)
// ============================================================================

KARCResult KARCPlanner::plan(const PlanningProblem& problem)
{
    std::cout << "K-ARC: Starting Kinodynamic Adaptive Robot Coordination..." << std::endl;

    planning_start_time_ = std::chrono::steady_clock::now();
    problem_ = problem;
    result_ = KARCResult();

    try {
        // Setup
        setupEnvironment(problem);
        setupRobots(problem);
        setupDynobenchObstacles();

        std::cout << "K-ARC: Planning for " << robots_.size() << " robots, "
                  << config_.num_segments << " segments" << std::endl;

        // Phase 1: Compute initial kinematic paths (individual RRTConnect)
        std::cout << "K-ARC: Phase 1 - Computing initial kinematic paths..." << std::endl;
        computeInitialKinematicPaths();

        if (isTimeoutExceeded()) {
            result_.failure_reason = "timeout_kinematic_paths";
            return result_;
        }

        // Phase 2: Segment kinematic paths into waypoints
        std::cout << "K-ARC: Phase 2 - Segmenting kinematic paths..." << std::endl;
        segmentKinematicPaths();

        // Initialize achieved states to start states
        achieved_states_.resize(robots_.size());
        for (size_t i = 0; i < robots_.size(); ++i) {
            achieved_states_[i] = omplStateToEigen(start_states_[i], robots_[i]);
        }

        // Initialize segment trajectories storage
        segment_trajectories_.resize(robots_.size());
        for (size_t i = 0; i < robots_.size(); ++i) {
            segment_trajectories_[i].resize(config_.num_segments);
        }

        result_.num_segments = config_.num_segments;

        // Phase 3-5: For each segment, plan kinodynamic + detect/resolve conflicts
        for (int seg = 0; seg < config_.num_segments; ++seg) {
            std::cout << "\nK-ARC: === Segment " << seg << "/" << config_.num_segments - 1
                      << " ===" << std::endl;

            if (isTimeoutExceeded()) {
                result_.failure_reason = "timeout_segment_" + std::to_string(seg);
                return result_;
            }

            // Phase 3: Compute kinodynamic trajectories for this segment
            std::cout << "K-ARC: Phase 3 - Computing kinodynamic segments..." << std::endl;
            bool kinodynamic_ok = computeKinodynamicSegments(seg);
            if (!kinodynamic_ok) {
                result_.failure_reason = "kinodynamic_planning_failed_segment_" + std::to_string(seg);
                std::cout << "K-ARC: Failed to compute kinodynamic segments for segment "
                          << seg << std::endl;
                return result_;
            }

            // Phase 4: Check for inter-robot conflicts in this segment
            std::cout << "K-ARC: Phase 4 - Checking for conflicts..." << std::endl;
            auto conflicts = findSegmentConflicts(seg);

            // Phase 5: Resolve conflicts if any
            if (!conflicts.empty()) {
                std::cout << "K-ARC: Found " << conflicts.size()
                          << " conflict(s) in segment " << seg << std::endl;
                result_.num_conflicts_found += conflicts.size();

                std::cout << "K-ARC: Phase 5 - Resolving conflicts..." << std::endl;
                bool resolved = resolveConflicts(seg, conflicts);

                if (!resolved) {
                    result_.failure_reason = "conflict_resolution_failed_segment_" + std::to_string(seg);
                    std::cout << "K-ARC: Failed to resolve conflicts in segment "
                              << seg << std::endl;
                    return result_;
                }
            } else {
                std::cout << "K-ARC: No conflicts in segment " << seg << std::endl;
            }

            // Update achieved states to the end of this segment
            for (size_t i = 0; i < robots_.size(); ++i) {
                const auto& traj = segment_trajectories_[i][seg];
                if (!traj.states.empty()) {
                    achieved_states_[i] = traj.states.back();
                }
            }
        }

        // Phase 6: Concatenate all segments into final trajectories
        std::cout << "\nK-ARC: Phase 6 - Concatenating segments..." << std::endl;
        concatenateSegments();

        result_.solved = true;
        result_.trajectories = final_trajectories_;

    } catch (const std::exception& e) {
        std::cerr << "K-ARC: Exception: " << e.what() << std::endl;
        result_.failure_reason = std::string("exception: ") + e.what();
    }

    auto end_time = std::chrono::steady_clock::now();
    result_.planning_time = std::chrono::duration<double>(end_time - planning_start_time_).count();

    std::cout << "\nK-ARC: Planning " << (result_.solved ? "SUCCEEDED" : "FAILED")
              << " in " << result_.planning_time << "s" << std::endl;
    std::cout << "K-ARC: Conflicts found: " << result_.num_conflicts_found
              << ", resolved: " << result_.num_conflicts_resolved << std::endl;

    return result_;
}

// ============================================================================
// Setup Methods
// ============================================================================

void KARCPlanner::setupEnvironment(const PlanningProblem& problem)
{
    position_bounds_.setLow(0, problem.env_min[0]);
    position_bounds_.setLow(1, problem.env_min[1]);
    position_bounds_.setHigh(0, problem.env_max[0]);
    position_bounds_.setHigh(1, problem.env_max[1]);

    col_mng_environment_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
    col_mng_environment_->registerObjects(problem.obstacles);
    col_mng_environment_->setup();
}

void KARCPlanner::setupRobots(const PlanningProblem& problem)
{
    robots_.clear();
    start_states_.clear();
    goal_states_.clear();

    for (size_t i = 0; i < problem.robots.size(); ++i) {
        const auto& robot_spec = problem.robots[i];

        // Create robot with position bounds
        auto robot = create_robot(robot_spec.type, position_bounds_);
        auto si = robot->getSpaceInformation();

        // Setup validity checker (environment obstacles only)
        si->setStateValidityChecker(
            std::make_shared<fclStateValidityChecker>(si, col_mng_environment_, robot));

        // Setup propagator (needed for Robot SpaceInformation which is control-based)
        si->setStatePropagator(std::make_shared<RobotStatePropagator>(si, robot));
        si->setup();

        robots_.push_back(robot);

        // Create start state
        ob::State* start = si->getStateSpace()->allocState();
        si->getStateSpace()->copyFromReals(start, robot_spec.start);
        start_states_.push_back(start);

        // Create goal state
        ob::State* goal = si->getStateSpace()->allocState();
        si->getStateSpace()->copyFromReals(goal, robot_spec.goal);
        goal_states_.push_back(goal);
    }
}

void KARCPlanner::setupDynobenchObstacles()
{
    dynobench_obstacles_.clear();
    dynobench_obstacles_.reserve(problem_.obstacles.size());

    for (const auto* obs : problem_.obstacles) {
        const auto* geom = obs->collisionGeometry().get();
        const auto& translation = obs->getTranslation();

        if (const auto* box = dynamic_cast<const fcl::Boxf*>(geom)) {
            dynobench::Obstacle dyno_obs;
            dyno_obs.type = "box";
            dyno_obs.size.resize(2);
            dyno_obs.size(0) = box->side[0];
            dyno_obs.size(1) = box->side[1];
            dyno_obs.center.resize(2);
            dyno_obs.center(0) = static_cast<double>(translation[0]);
            dyno_obs.center(1) = static_cast<double>(translation[1]);
            dynobench_obstacles_.push_back(dyno_obs);
        }
    }

    if (config_.debug) {
        std::cout << "K-ARC: Converted " << dynobench_obstacles_.size()
                  << " obstacles to dynobench format" << std::endl;
    }
}

void KARCPlanner::cleanup()
{
    for (size_t i = 0; i < robots_.size(); ++i) {
        if (i < start_states_.size() && start_states_[i]) {
            robots_[i]->getSpaceInformation()->getStateSpace()->freeState(start_states_[i]);
        }
        if (i < goal_states_.size() && goal_states_[i]) {
            robots_[i]->getSpaceInformation()->getStateSpace()->freeState(goal_states_[i]);
        }
    }

    robots_.clear();
    start_states_.clear();
    goal_states_.clear();
    kinematic_paths_.clear();
    waypoints_.clear();
    segment_trajectories_.clear();
    achieved_states_.clear();
    final_trajectories_.clear();
    col_mng_environment_.reset();
    dynobench_obstacles_.clear();
}

// ============================================================================
// Phase 1: Initial Kinematic Paths (adapted from arc.cpp)
// ============================================================================

void KARCPlanner::computeInitialKinematicPaths()
{
    kinematic_paths_.clear();
    kinematic_paths_.resize(robots_.size());

    double time_per_robot = config_.initial_planning_time / robots_.size();

    for (size_t i = 0; i < robots_.size(); ++i) {
        std::cout << "  Planning kinematic path for robot " << i << "..." << std::endl;

        auto robot = robots_[i];
        auto si = robot->getSpaceInformation();

        // Create problem definition for geometric planning
        auto pdef = std::make_shared<ob::ProblemDefinition>(si);
        pdef->addStartState(start_states_[i]);

        auto goal = std::make_shared<ob::GoalState>(si);
        goal->setState(goal_states_[i]);
        goal->setThreshold(config_.goal_threshold);
        pdef->setGoal(goal);

        // Use RRTConnect for fast individual paths
        auto planner = std::make_shared<og::RRTConnect>(si);
        planner->setProblemDefinition(pdef);
        planner->setup();

        ob::PlannerStatus solved = planner->solve(
            ob::timedPlannerTerminationCondition(time_per_robot));

        if (solved) {
            auto path = std::dynamic_pointer_cast<og::PathGeometric>(
                pdef->getSolutionPath());
            if (path) {
                path->interpolate();
                kinematic_paths_[i] = std::make_shared<og::PathGeometric>(*path);
                std::cout << "    Robot " << i << ": " << path->getStateCount()
                          << " states" << std::endl;
            } else {
                throw std::runtime_error(
                    "K-ARC: Failed to get kinematic path for robot " + std::to_string(i));
            }
        } else {
            throw std::runtime_error(
                "K-ARC: RRTConnect failed for robot " + std::to_string(i));
        }
    }
}

// ============================================================================
// Phase 2: Path Segmentation
// ============================================================================

void KARCPlanner::segmentKinematicPaths()
{
    int m = config_.num_segments;
    waypoints_.resize(robots_.size());

    for (size_t i = 0; i < robots_.size(); ++i) {
        auto& path = kinematic_paths_[i];
        if (!path || path->getStateCount() == 0) {
            throw std::runtime_error(
                "K-ARC: No kinematic path for robot " + std::to_string(i));
        }

        size_t num_states = path->getStateCount();
        waypoints_[i].resize(m);

        // Extract m waypoints at equal intervals along the path
        // waypoints_[i][j] = state at position (j+1)/m along the path
        for (int j = 0; j < m; ++j) {
            size_t state_idx;
            if (j == m - 1) {
                // Last waypoint is the goal
                state_idx = num_states - 1;
            } else {
                // Intermediate waypoints at equal fractions
                double fraction = static_cast<double>(j + 1) / m;
                state_idx = static_cast<size_t>(fraction * (num_states - 1));
            }

            waypoints_[i][j] = omplStateToEigen(
                path->getState(state_idx), robots_[i]);
        }

        if (config_.debug) {
            std::cout << "  Robot " << i << " waypoints:" << std::endl;
            for (int j = 0; j < m; ++j) {
                std::cout << "    Segment " << j << " goal: "
                          << waypoints_[i][j].head(2).transpose() << std::endl;
            }
        }
    }
}

// ============================================================================
// Phase 3: Per-Segment Kinodynamic Planning
// ============================================================================

bool KARCPlanner::computeKinodynamicSegments(int segment_idx)
{
    bool is_final_segment = (segment_idx == config_.num_segments - 1);

    for (size_t i = 0; i < robots_.size(); ++i) {
        // Start state: achieved state from previous segment (or initial start)
        Eigen::VectorXd start = achieved_states_[i];

        // Goal state: waypoint for this segment
        Eigen::VectorXd goal = waypoints_[i][segment_idx];

        // Adjust tolerance for final vs intermediate segments
        double original_goal_region = config_.db_rrt_config.goal_region;
        if (is_final_segment) {
            config_.db_rrt_config.goal_region = config_.final_goal_tolerance;
        } else {
            config_.db_rrt_config.goal_region = config_.waypoint_tolerance;
        }

        if (config_.debug) {
            std::cout << "  Robot " << i << ": start="
                      << start.head(std::min((int)start.size(), 2)).transpose()
                      << " -> goal=" << goal.head(std::min((int)goal.size(), 2)).transpose()
                      << std::endl;
        }

        auto [success, traj] = planRobotSegment(
            i, start, goal, config_.segment_planning_time);

        // Restore original goal region
        config_.db_rrt_config.goal_region = original_goal_region;

        if (!success || traj.states.empty()) {
            std::cerr << "K-ARC: DB-RRT failed for robot " << i
                      << " in segment " << segment_idx << std::endl;
            return false;
        }

        segment_trajectories_[i][segment_idx] = traj;

        if (config_.debug) {
            std::cout << "  Robot " << i << ": trajectory with "
                      << traj.states.size() << " states" << std::endl;
        }
    }

    return true;
}

std::pair<bool, dynobench::Trajectory> KARCPlanner::planRobotSegment(
    size_t robot_idx,
    const Eigen::VectorXd& start_state,
    const Eigen::VectorXd& goal_state,
    double time_budget)
{
    dynobench::Trajectory traj_out;

    try {
        auto robot = robots_[robot_idx];

        // Get robot type and bounds
        std::string robot_type = getRobotType(robot);
        std::vector<double> min_bounds, max_bounds;
        getPositionBounds(robot, min_bounds, max_bounds);

        // Create dynobench problem
        auto problem = std::make_shared<dynobench::Problem>();
        problem->robotType = robot_type;
        problem->robotTypes.push_back(robot_type);
        problem->models_base_path = config_.db_rrt_config.models_base_path.empty()
            ? config_.models_base_path : config_.db_rrt_config.models_base_path;
        problem->start = start_state;
        problem->goal = goal_state;

        size_t pos_dim = min_bounds.size();
        problem->p_lb.resize(pos_dim);
        problem->p_ub.resize(pos_dim);
        for (size_t i = 0; i < pos_dim; ++i) {
            problem->p_lb(i) = min_bounds[i];
            problem->p_ub(i) = max_bounds[i];
        }
        problem->obstacles = dynobench_obstacles_;

        // Create dynobench robot model
        std::string model_file = problem->models_base_path + "/" + robot_type + ".yaml";
        auto dynobench_robot = dynobench::robot_factory(
            model_file.c_str(),
            Eigen::Map<Eigen::VectorXd>(min_bounds.data(), min_bounds.size()),
            Eigen::Map<Eigen::VectorXd>(max_bounds.data(), max_bounds.size()));

        if (!dynobench_robot) {
            std::cerr << "K-ARC: Failed to create dynobench robot model" << std::endl;
            return {false, traj_out};
        }

        dynobench::load_env(*dynobench_robot, *problem);

        // Load motion primitives
        std::string motions_file = config_.db_rrt_config.motions_file.empty()
            ? config_.motions_file : config_.db_rrt_config.motions_file;

        if (motions_file.empty()) {
            motions_file = problem->models_base_path + "/../dynomotions/" + robot_type + "_default.msgpack";
        }

        std::vector<dynoplan::Motion> motions;
        dynoplan::load_motion_primitives_new(
            motions_file,
            *dynobench_robot,
            motions,
            config_.db_rrt_config.max_motions,
            false, false, true);

        if (motions.empty()) {
            std::cerr << "K-ARC: No motion primitives loaded" << std::endl;
            return {false, traj_out};
        }

        auto dynobench_robot_shared = std::shared_ptr<dynobench::Model_robot>(
            std::move(dynobench_robot));

        // Configure DB-RRT
        dynoplan::Options_dbrrt options;
        options.timelimit = time_budget * 1000;  // ms
        options.max_expands = config_.db_rrt_config.max_expands;
        options.goal_region = config_.db_rrt_config.goal_region;
        options.delta = config_.db_rrt_config.delta;
        options.goal_bias = config_.db_rrt_config.goal_bias;
        options.max_motions = config_.db_rrt_config.max_motions;
        options.seed = config_.db_rrt_config.seed;
        options.do_optimization = config_.db_rrt_config.do_optimization;
        options.use_nigh_nn = config_.db_rrt_config.use_nigh_nn;
        options.debug = config_.db_rrt_config.debug;
        options.motionsFile = motions_file;
        options.motions_ptr = &motions;

        dynoplan::Options_trajopt options_trajopt;
        options_trajopt.solver_id = config_.db_rrt_config.solver_id;

        dynobench::Info_out out_info;
        dynoplan::idbrrt(*problem, dynobench_robot_shared, options, options_trajopt,
                         traj_out, out_info);

        return {out_info.solved, traj_out};

    } catch (const std::exception& e) {
        std::cerr << "K-ARC: planRobotSegment exception: " << e.what() << std::endl;
        return {false, traj_out};
    }
}

// ============================================================================
// Phase 4: Per-Segment Conflict Detection
// ============================================================================

std::vector<KARCConflict> KARCPlanner::findSegmentConflicts(int segment_idx)
{
    std::vector<KARCConflict> conflicts;

    // Find the maximum number of timesteps in this segment across all robots
    size_t max_timesteps = 0;
    for (size_t i = 0; i < robots_.size(); ++i) {
        const auto& traj = segment_trajectories_[i][segment_idx];
        max_timesteps = std::max(max_timesteps, traj.states.size());
    }

    if (max_timesteps == 0) return conflicts;

    // Check each timestep for robot-robot collisions
    for (size_t t = 0; t < max_timesteps; ++t) {
        for (size_t i = 0; i < robots_.size(); ++i) {
            for (size_t j = i + 1; j < robots_.size(); ++j) {
                const auto& traj_i = segment_trajectories_[i][segment_idx];
                const auto& traj_j = segment_trajectories_[j][segment_idx];

                if (traj_i.states.empty() || traj_j.states.empty()) continue;

                // Get state at timestep t (clamp to last state if shorter)
                size_t idx_i = std::min(t, traj_i.states.size() - 1);
                size_t idx_j = std::min(t, traj_j.states.size() - 1);

                const Eigen::VectorXd& state_i = traj_i.states[idx_i];
                const Eigen::VectorXd& state_j = traj_j.states[idx_j];

                if (checkTwoRobotCollision(i, state_i, j, state_j)) {
                    KARCConflict conflict;
                    conflict.robot_i = i;
                    conflict.robot_j = j;
                    conflict.segment_index = segment_idx;
                    conflict.timestep = static_cast<int>(t);
                    conflicts.push_back(conflict);

                    if (config_.debug) {
                        std::cout << "  Conflict: robots " << i << " and " << j
                                  << " at timestep " << t << std::endl;
                    }

                    // Only report first conflict per robot pair per segment
                    goto next_pair;
                }
            }
            next_pair:;
        }
    }

    return conflicts;
}

bool KARCPlanner::checkTwoRobotCollision(
    size_t robot_i, const Eigen::VectorXd& state_i,
    size_t robot_j, const Eigen::VectorXd& state_j) const
{
    auto r_i = robots_[robot_i];
    auto r_j = robots_[robot_j];
    auto si_i = r_i->getSpaceInformation();
    auto si_j = r_j->getSpaceInformation();

    // Convert Eigen states to OMPL states for FCL transform
    ob::State* ompl_i = si_i->allocState();
    ob::State* ompl_j = si_j->allocState();

    std::vector<double> vec_i(state_i.data(), state_i.data() + state_i.size());
    std::vector<double> vec_j(state_j.data(), state_j.data() + state_j.size());
    si_i->getStateSpace()->copyFromReals(ompl_i, vec_i);
    si_j->getStateSpace()->copyFromReals(ompl_j, vec_j);

    bool collision = false;

    for (size_t p1 = 0; p1 < r_i->numParts() && !collision; ++p1) {
        for (size_t p2 = 0; p2 < r_j->numParts() && !collision; ++p2) {
            const auto& transform_i = r_i->getTransform(ompl_i, p1);
            const auto& transform_j = r_j->getTransform(ompl_j, p2);

            fcl::CollisionObjectf co_i(r_i->getCollisionGeometry(p1));
            co_i.setTranslation(transform_i.translation());
            co_i.setRotation(transform_i.rotation());
            co_i.computeAABB();

            fcl::CollisionObjectf co_j(r_j->getCollisionGeometry(p2));
            co_j.setTranslation(transform_j.translation());
            co_j.setRotation(transform_j.rotation());
            co_j.computeAABB();

            fcl::CollisionRequestf request;
            fcl::CollisionResultf result;
            fcl::collide(&co_i, &co_j, request, result);

            if (result.isCollision()) {
                collision = true;
            }
        }
    }

    si_i->getStateSpace()->freeState(ompl_i);
    si_j->getStateSpace()->freeState(ompl_j);

    return collision;
}

// ============================================================================
// Phase 5: Conflict Resolution (Algorithm 2 from K-ARC paper)
// ============================================================================

bool KARCPlanner::resolveConflicts(
    int segment_idx,
    const std::vector<KARCConflict>& conflicts)
{
    // Collect unique robot pairs involved in conflicts
    std::set<std::pair<size_t, size_t>> conflict_pairs;
    for (const auto& c : conflicts) {
        conflict_pairs.insert({std::min(c.robot_i, c.robot_j),
                              std::max(c.robot_i, c.robot_j)});
    }

    for (const auto& [ri, rj] : conflict_pairs) {
        std::cout << "K-ARC: Resolving conflict between robots " << ri
                  << " and " << rj << " in segment " << segment_idx << std::endl;

        bool resolved = false;

        for (int escalation = 0; escalation <= config_.max_solver_escalations; ++escalation) {
            if (isTimeoutExceeded()) {
                std::cout << "K-ARC: Timeout during conflict resolution" << std::endl;
                return false;
            }

            if (escalation == 0) {
                // Level 1: Prioritized DB-RRT
                std::cout << "  Level 1: Trying prioritized DB-RRT..." << std::endl;
                if (tryPrioritizedDBRRT(segment_idx, ri, rj)) {
                    result_.methods_used.push_back("PrioritizedDBRRT");
                    result_.num_conflicts_resolved++;
                    resolved = true;
                    break;
                }

                // Try the other priority order
                std::cout << "  Level 1: Trying reverse priority..." << std::endl;
                if (tryPrioritizedDBRRT(segment_idx, rj, ri)) {
                    result_.methods_used.push_back("PrioritizedDBRRT_reverse");
                    result_.num_conflicts_resolved++;
                    resolved = true;
                    break;
                }

                // Level 2: Composite DB-RRT
                std::cout << "  Level 2: Trying composite DB-RRT..." << std::endl;
                if (tryCompositeDBRRT(segment_idx, {ri, rj})) {
                    result_.methods_used.push_back("CompositeDBRRT");
                    result_.num_conflicts_resolved++;
                    resolved = true;
                    break;
                }
            } else {
                // Subproblem adaptation: expand to adjacent segment boundaries
                std::cout << "  Escalation " << escalation
                          << ": Expanding subproblem..." << std::endl;
                if (tryExpandedSubproblem(segment_idx, {ri, rj}, escalation)) {
                    result_.methods_used.push_back(
                        "ExpandedSubproblem_level" + std::to_string(escalation));
                    result_.num_conflicts_resolved++;
                    resolved = true;
                    break;
                }
            }
        }

        if (!resolved) {
            std::cout << "K-ARC: Failed to resolve conflict between robots "
                      << ri << " and " << rj << std::endl;
            return false;
        }

        // Re-check for new conflicts after resolution
        auto new_conflicts = findSegmentConflicts(segment_idx);
        if (new_conflicts.empty()) {
            std::cout << "K-ARC: All conflicts resolved in segment "
                      << segment_idx << std::endl;
            return true;
        }
    }

    // Final check
    auto remaining = findSegmentConflicts(segment_idx);
    return remaining.empty();
}

bool KARCPlanner::tryPrioritizedDBRRT(
    int segment_idx,
    size_t robot_high_priority,
    size_t robot_low_priority)
{
    // High-priority robot keeps its current trajectory.
    // Low-priority robot replans with high-priority robot as dynamic obstacle.
    //
    // For now, we implement this by replanning the low-priority robot
    // and then checking if the result is collision-free with the
    // high-priority robot's trajectory.

    // Get the start and goal for the low-priority robot in this segment
    Eigen::VectorXd start_lp;
    if (segment_idx == 0) {
        start_lp = omplStateToEigen(start_states_[robot_low_priority],
                                     robots_[robot_low_priority]);
    } else {
        // Start from end of previous segment
        const auto& prev_traj = segment_trajectories_[robot_low_priority][segment_idx - 1];
        if (!prev_traj.states.empty()) {
            start_lp = prev_traj.states.back();
        } else {
            start_lp = achieved_states_[robot_low_priority];
        }
    }

    Eigen::VectorXd goal_lp = waypoints_[robot_low_priority][segment_idx];

    // Replan low-priority robot with shorter timeout
    auto [success, new_traj] = planRobotSegment(
        robot_low_priority, start_lp, goal_lp, config_.prioritized_timeout);

    if (!success || new_traj.states.empty()) {
        return false;
    }

    // Check if the new trajectory is collision-free with high-priority robot
    const auto& hp_traj = segment_trajectories_[robot_high_priority][segment_idx];
    size_t max_t = std::max(new_traj.states.size(), hp_traj.states.size());

    for (size_t t = 0; t < max_t; ++t) {
        size_t idx_hp = std::min(t, hp_traj.states.size() - 1);
        size_t idx_lp = std::min(t, new_traj.states.size() - 1);

        if (checkTwoRobotCollision(robot_high_priority, hp_traj.states[idx_hp],
                                    robot_low_priority, new_traj.states[idx_lp])) {
            // Still colliding - prioritized approach failed
            return false;
        }
    }

    // Success - update the low-priority robot's trajectory
    segment_trajectories_[robot_low_priority][segment_idx] = new_traj;
    return true;
}

bool KARCPlanner::tryCompositeDBRRT(
    int segment_idx,
    const std::vector<size_t>& robot_indices)
{
    // Use CompositeDBRRTPlanner for joint planning of conflicting robots

    // Build the composite planning problem
    CompositeDBRRTProblem composite_problem;
    composite_problem.env_min = problem_.env_min;
    composite_problem.env_max = problem_.env_max;
    composite_problem.obstacles = problem_.obstacles;
    composite_problem.dynobench_obstacles = dynobench_obstacles_;

    for (size_t robot_idx : robot_indices) {
        CompositeRobotSpec spec;
        spec.type = problem_.robots[robot_idx].type;

        // Start: achieved state or segment start
        Eigen::VectorXd start;
        if (segment_idx == 0) {
            start = omplStateToEigen(start_states_[robot_idx], robots_[robot_idx]);
        } else {
            const auto& prev_traj = segment_trajectories_[robot_idx][segment_idx - 1];
            if (!prev_traj.states.empty()) {
                start = prev_traj.states.back();
            } else {
                start = achieved_states_[robot_idx];
            }
        }

        // Goal: waypoint for this segment
        Eigen::VectorXd goal = waypoints_[robot_idx][segment_idx];

        spec.start = std::vector<double>(start.data(), start.data() + start.size());
        spec.goal = std::vector<double>(goal.data(), goal.data() + goal.size());

        composite_problem.robots.push_back(spec);
    }

    // Configure and run composite planner
    CompositeDBRRTConfig composite_config = config_.composite_dbrrt_config;
    composite_config.time_limit = config_.composite_timeout;
    if (!config_.motions_file.empty()) {
        // Set motion files for all robot types
        for (size_t robot_idx : robot_indices) {
            std::string rtype = problem_.robots[robot_idx].type;
            if (composite_config.motion_files.find(rtype) == composite_config.motion_files.end()) {
                composite_config.motion_files[rtype] = config_.motions_file;
            }
        }
    }
    if (composite_config.models_base_path.empty()) {
        composite_config.models_base_path = config_.models_base_path;
    }

    CompositeDBRRTPlanner composite_planner(composite_config);
    CompositeDBRRTResult composite_result = composite_planner.plan(composite_problem);

    if (!composite_result.solved ||
        composite_result.trajectories.size() != robot_indices.size()) {
        return false;
    }

    // Update segment trajectories for the resolved robots
    for (size_t i = 0; i < robot_indices.size(); ++i) {
        segment_trajectories_[robot_indices[i]][segment_idx] =
            composite_result.trajectories[i];
    }

    return true;
}

bool KARCPlanner::tryExpandedSubproblem(
    int segment_idx,
    const std::vector<size_t>& robot_indices,
    int escalation_level)
{
    // Per the K-ARC paper (Algorithm 2):
    // Expand the subproblem to use previous segment's start and/or
    // next segment's goal, giving robots more room to maneuver.

    // Calculate expanded segment range
    int start_seg = std::max(0, segment_idx - escalation_level);
    int end_seg = std::min(config_.num_segments - 1, segment_idx + escalation_level);

    std::cout << "    Expanded range: segments [" << start_seg << ", " << end_seg << "]"
              << std::endl;

    // Build composite problem spanning multiple segments
    CompositeDBRRTProblem composite_problem;
    composite_problem.env_min = problem_.env_min;
    composite_problem.env_max = problem_.env_max;
    composite_problem.obstacles = problem_.obstacles;
    composite_problem.dynobench_obstacles = dynobench_obstacles_;

    for (size_t robot_idx : robot_indices) {
        CompositeRobotSpec spec;
        spec.type = problem_.robots[robot_idx].type;

        // Start from expanded start segment
        Eigen::VectorXd start;
        if (start_seg == 0) {
            start = omplStateToEigen(start_states_[robot_idx], robots_[robot_idx]);
        } else {
            const auto& prev_traj = segment_trajectories_[robot_idx][start_seg - 1];
            if (!prev_traj.states.empty()) {
                start = prev_traj.states.back();
            } else {
                start = achieved_states_[robot_idx];
            }
        }

        // Goal is the expanded end segment's waypoint
        Eigen::VectorXd goal = waypoints_[robot_idx][end_seg];

        spec.start = std::vector<double>(start.data(), start.data() + start.size());
        spec.goal = std::vector<double>(goal.data(), goal.data() + goal.size());

        composite_problem.robots.push_back(spec);
    }

    // Run composite planner with extended timeout
    CompositeDBRRTConfig composite_config = config_.composite_dbrrt_config;
    composite_config.time_limit = config_.composite_timeout * (1 + escalation_level);
    if (!config_.motions_file.empty()) {
        for (size_t robot_idx : robot_indices) {
            std::string rtype = problem_.robots[robot_idx].type;
            if (composite_config.motion_files.find(rtype) == composite_config.motion_files.end()) {
                composite_config.motion_files[rtype] = config_.motions_file;
            }
        }
    }
    if (composite_config.models_base_path.empty()) {
        composite_config.models_base_path = config_.models_base_path;
    }

    CompositeDBRRTPlanner composite_planner(composite_config);
    CompositeDBRRTResult composite_result = composite_planner.plan(composite_problem);

    if (!composite_result.solved ||
        composite_result.trajectories.size() != robot_indices.size()) {
        return false;
    }

    // The composite planner gives us trajectories spanning the expanded range.
    // We need to re-segment these into the individual segment slots.
    // For simplicity, we place the full trajectory into the current segment
    // and clear the expanded segments (they'll be part of the merged trajectory).
    for (size_t i = 0; i < robot_indices.size(); ++i) {
        size_t ridx = robot_indices[i];
        const auto& full_traj = composite_result.trajectories[i];

        if (full_traj.states.empty()) continue;

        // Calculate how to split the trajectory across the expanded segments
        int num_expanded_segments = end_seg - start_seg + 1;
        size_t states_per_seg = full_traj.states.size() / num_expanded_segments;

        for (int s = start_seg; s <= end_seg; ++s) {
            dynobench::Trajectory seg_traj;
            size_t seg_start = (s - start_seg) * states_per_seg;
            size_t seg_end;
            if (s == end_seg) {
                seg_end = full_traj.states.size();
            } else {
                seg_end = seg_start + states_per_seg;
            }

            for (size_t st = seg_start; st < seg_end; ++st) {
                seg_traj.states.push_back(full_traj.states[st]);
                if (st < full_traj.actions.size()) {
                    seg_traj.actions.push_back(full_traj.actions[st]);
                }
            }

            segment_trajectories_[ridx][s] = seg_traj;
        }
    }

    return true;
}

// ============================================================================
// Phase 6: Solution Construction
// ============================================================================

void KARCPlanner::concatenateSegments()
{
    final_trajectories_.resize(robots_.size());

    for (size_t i = 0; i < robots_.size(); ++i) {
        dynobench::Trajectory& full_traj = final_trajectories_[i];
        full_traj.states.clear();
        full_traj.actions.clear();

        for (int seg = 0; seg < config_.num_segments; ++seg) {
            const auto& seg_traj = segment_trajectories_[i][seg];

            if (seg_traj.states.empty()) continue;

            if (seg == 0 && full_traj.states.empty()) {
                // First segment: include all states
                full_traj.states.insert(full_traj.states.end(),
                    seg_traj.states.begin(), seg_traj.states.end());
                full_traj.actions.insert(full_traj.actions.end(),
                    seg_traj.actions.begin(), seg_traj.actions.end());
            } else {
                // Subsequent segments: skip first state (overlap)
                if (seg_traj.states.size() > 1) {
                    full_traj.states.insert(full_traj.states.end(),
                        seg_traj.states.begin() + 1, seg_traj.states.end());
                }
                full_traj.actions.insert(full_traj.actions.end(),
                    seg_traj.actions.begin(), seg_traj.actions.end());
            }
        }

        std::cout << "K-ARC: Robot " << i << " final trajectory: "
                  << full_traj.states.size() << " states, "
                  << full_traj.actions.size() << " actions" << std::endl;
    }
}

// ============================================================================
// State Conversion Helpers
// ============================================================================

Eigen::VectorXd KARCPlanner::omplStateToEigen(
    const ob::State* state,
    const std::shared_ptr<Robot>& robot) const
{
    auto si = robot->getSpaceInformation();
    auto state_space = si->getStateSpace();

    std::vector<double> values;

    if (auto compound = state_space->as<ob::CompoundStateSpace>()) {
        const auto* cs = state->as<ob::CompoundState>();
        for (size_t i = 0; i < compound->getSubspaceCount(); ++i) {
            auto subspace = compound->getSubspace(i);
            const auto* sub = cs->as<ob::State>(i);

            if (subspace->getType() == ob::STATE_SPACE_SO2) {
                values.push_back(sub->as<ob::SO2StateSpace::StateType>()->value);
            } else if (subspace->getType() == ob::STATE_SPACE_REAL_VECTOR) {
                auto rv = subspace->as<ob::RealVectorStateSpace>();
                const auto* rvs = sub->as<ob::RealVectorStateSpace::StateType>();
                for (size_t j = 0; j < rv->getDimension(); ++j) {
                    values.push_back(rvs->values[j]);
                }
            }
        }
    } else if (auto rv = state_space->as<ob::RealVectorStateSpace>()) {
        const auto* rvs = state->as<ob::RealVectorStateSpace::StateType>();
        for (size_t i = 0; i < rv->getDimension(); ++i) {
            values.push_back(rvs->values[i]);
        }
    }

    return Eigen::Map<Eigen::VectorXd>(values.data(), values.size());
}

void KARCPlanner::eigenToOmplState(
    const Eigen::VectorXd& eigen_state,
    ob::State* ompl_state,
    const std::shared_ptr<Robot>& robot) const
{
    auto si = robot->getSpaceInformation();
    auto state_space = si->getStateSpace();

    size_t idx = 0;

    if (auto compound = state_space->as<ob::CompoundStateSpace>()) {
        auto* cs = ompl_state->as<ob::CompoundState>();
        for (size_t i = 0; i < compound->getSubspaceCount(); ++i) {
            auto subspace = compound->getSubspace(i);
            auto* sub = cs->as<ob::State>(i);

            if (subspace->getType() == ob::STATE_SPACE_SO2) {
                sub->as<ob::SO2StateSpace::StateType>()->value = eigen_state(idx++);
            } else if (subspace->getType() == ob::STATE_SPACE_REAL_VECTOR) {
                auto rv = subspace->as<ob::RealVectorStateSpace>();
                auto* rvs = sub->as<ob::RealVectorStateSpace::StateType>();
                for (size_t j = 0; j < rv->getDimension(); ++j) {
                    rvs->values[j] = eigen_state(idx++);
                }
            }
        }
    } else if (auto rv = state_space->as<ob::RealVectorStateSpace>()) {
        auto* rvs = ompl_state->as<ob::RealVectorStateSpace::StateType>();
        for (size_t i = 0; i < rv->getDimension(); ++i) {
            rvs->values[i] = eigen_state(idx++);
        }
    }
}

void KARCPlanner::getPositionBounds(
    const std::shared_ptr<Robot>& robot,
    std::vector<double>& min_bounds,
    std::vector<double>& max_bounds) const
{
    auto si = robot->getSpaceInformation();
    auto state_space = si->getStateSpace();

    min_bounds.clear();
    max_bounds.clear();

    if (auto compound = state_space->as<ob::CompoundStateSpace>()) {
        for (size_t i = 0; i < compound->getSubspaceCount(); ++i) {
            auto subspace = compound->getSubspace(i);
            if (subspace->getType() == ob::STATE_SPACE_REAL_VECTOR) {
                auto rv = subspace->as<ob::RealVectorStateSpace>();
                ob::RealVectorBounds bounds = rv->getBounds();
                for (size_t j = 0; j < bounds.low.size(); ++j) {
                    min_bounds.push_back(bounds.low[j]);
                    max_bounds.push_back(bounds.high[j]);
                }
            }
        }
    } else if (auto rv = state_space->as<ob::RealVectorStateSpace>()) {
        ob::RealVectorBounds bounds = rv->getBounds();
        for (size_t i = 0; i < bounds.low.size(); ++i) {
            min_bounds.push_back(bounds.low[i]);
            max_bounds.push_back(bounds.high[i]);
        }
    }
}

std::string KARCPlanner::getRobotType(const std::shared_ptr<Robot>& robot) const
{
    if (robot) {
        auto si = robot->getSpaceInformation();
        size_t state_dim = si->getStateDimension();

        if (state_dim == 3) return "unicycle1_v0";
        if (state_dim == 4) return "integrator2_2d_v0";
        if (state_dim == 8) return "quad2d_v0";
        if (state_dim == 13) return "quad3d_v0";
    }
    return "unicycle1_v0";
}

// ============================================================================
// Trajectory Conversion
// ============================================================================

std::shared_ptr<oc::PathControl> KARCPlanner::convertDynobenchTrajectory(
    const dynobench::Trajectory& traj,
    const std::shared_ptr<Robot>& robot) const
{
    auto si = robot->getSpaceInformation();
    auto path = std::make_shared<oc::PathControl>(si);
    auto state_space = si->getStateSpace();

    for (size_t i = 0; i < traj.states.size(); ++i) {
        ob::State* state = si->allocState();

        // Convert Eigen state to OMPL state
        size_t idx = 0;
        if (auto compound = state_space->as<ob::CompoundStateSpace>()) {
            auto* cs = state->as<ob::CompoundState>();
            for (size_t s = 0; s < compound->getSubspaceCount(); ++s) {
                auto subspace = compound->getSubspace(s);
                auto* sub = cs->as<ob::State>(s);

                if (subspace->getType() == ob::STATE_SPACE_SO2) {
                    sub->as<ob::SO2StateSpace::StateType>()->value = traj.states[i](idx++);
                } else if (subspace->getType() == ob::STATE_SPACE_REAL_VECTOR) {
                    auto rv = subspace->as<ob::RealVectorStateSpace>();
                    auto* rvs = sub->as<ob::RealVectorStateSpace::StateType>();
                    for (size_t j = 0; j < rv->getDimension(); ++j) {
                        rvs->values[j] = traj.states[i](idx++);
                    }
                }
            }
        } else if (auto rv = state_space->as<ob::RealVectorStateSpace>()) {
            auto* rvs = state->as<ob::RealVectorStateSpace::StateType>();
            for (size_t j = 0; j < rv->getDimension(); ++j) {
                rvs->values[j] = traj.states[i](j);
            }
        }

        if (i < traj.actions.size()) {
            oc::Control* control = si->allocControl();
            auto* rvc = control->as<oc::RealVectorControlSpace::ControlType>();
            for (size_t j = 0; j < traj.actions[i].size() &&
                              j < si->getControlSpace()->getDimension(); ++j) {
                rvc->values[j] = traj.actions[i](j);
            }
            double duration = 0.1;
            path->append(state, control, duration);
        } else {
            path->append(state);
        }
    }

    return path;
}

// ============================================================================
// Timing
// ============================================================================

bool KARCPlanner::isTimeoutExceeded() const
{
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - planning_start_time_).count();
    return elapsed >= config_.total_time_limit;
}

// ============================================================================
// Configuration Loading
// ============================================================================

KARCConfig loadKARCConfig(const YAML::Node& node)
{
    KARCConfig config;

    if (node["num_segments"]) config.num_segments = node["num_segments"].as<int>();
    if (node["initial_planning_time"]) config.initial_planning_time = node["initial_planning_time"].as<double>();
    if (node["goal_threshold"]) config.goal_threshold = node["goal_threshold"].as<double>();
    if (node["segment_planning_time"]) config.segment_planning_time = node["segment_planning_time"].as<double>();
    if (node["waypoint_tolerance"]) config.waypoint_tolerance = node["waypoint_tolerance"].as<double>();
    if (node["final_goal_tolerance"]) config.final_goal_tolerance = node["final_goal_tolerance"].as<double>();
    if (node["max_solver_escalations"]) config.max_solver_escalations = node["max_solver_escalations"].as<int>();
    if (node["prioritized_timeout"]) config.prioritized_timeout = node["prioritized_timeout"].as<double>();
    if (node["composite_timeout"]) config.composite_timeout = node["composite_timeout"].as<double>();
    if (node["total_time_limit"]) config.total_time_limit = node["total_time_limit"].as<double>();
    if (node["models_base_path"]) config.models_base_path = node["models_base_path"].as<std::string>();
    if (node["motions_file"]) config.motions_file = node["motions_file"].as<std::string>();
    if (node["seed"]) config.seed = node["seed"].as<int>();
    if (node["debug"]) config.debug = node["debug"].as<bool>();

    // Load DB-RRT sub-config
    if (node["db_rrt"]) {
        config.db_rrt_config = mr_syclop::loadDBRRTConfig(node["db_rrt"]);
    }

    // Load composite DB-RRT sub-config
    if (node["composite_dbrrt"]) {
        config.composite_dbrrt_config = loadCompositeDBRRTConfig(node["composite_dbrrt"]);
    }

    return config;
}
