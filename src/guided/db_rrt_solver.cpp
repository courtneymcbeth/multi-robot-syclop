/**
 * @file db_rrt_solver.cpp
 * @brief Implementation of DB-RRT solver for mr_syclop
 */

// Pinocchio/Crocoddyl must be included before Boost/OMPL headers
#include <dynoplan/optimization/ocp.hpp>

#include "db_rrt_solver.h"
#include "robots.h"
#include "decomposition.h"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <random>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace mr_syclop {

DBRRTSolver::DBRRTSolver(
    const GuidedPlannerConfig& config,
    const DBRRTConfig& db_config,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager,
    const std::vector<dynobench::Obstacle>& dynobench_obstacles)
    : GuidedPlanner(config), db_config_(db_config), collision_manager_(collision_manager),
      dynobench_obstacles_(dynobench_obstacles) {

    if (config_.debug) {
        std::cout << "[DBRRTSolver] Initialized with:" << std::endl;
        std::cout << "  Motions file: " << db_config_.motions_file << std::endl;
        std::cout << "  Models path: " << db_config_.models_base_path << std::endl;
        std::cout << "  Time limit: " << db_config_.timelimit << "s" << std::endl;
        std::cout << "  Max expands: " << db_config_.max_expands << std::endl;
        std::cout << "  Precomputed obstacles: " << dynobench_obstacles_.size() << std::endl;
    }
}

GuidedPlanningResult DBRRTSolver::solve(
    std::shared_ptr<::Robot> robot,
    oc::DecompositionPtr decomp,
    ob::State* start_state,
    ob::State* goal_state,
    const std::vector<int>& region_path,
    size_t robot_index) {

    GuidedPlanningResult result;
    result.robot_index = robot_index;
    result.success = false;

    auto start_time = std::chrono::high_resolution_clock::now();

    // Use SYCLOP-style region guidance if enabled and we have a region path
    if (db_config_.use_region_guidance && db_config_.use_waypoint_planning &&
        !region_path.empty() && region_path.size() > 1) {
        if (config_.debug) {
            std::cout << "[DBRRTSolver] Using SYCLOP-style region guidance for robot "
                      << robot_index << std::endl;
            std::cout << "  Region path: ";
            for (int r : region_path) std::cout << r << " ";
            std::cout << std::endl;
        }
        return solveWithRegionGuidance(robot, decomp, start_state, goal_state,
                                       region_path, robot_index);
    }

    try {
        if (config_.debug) {
            std::cout << "[DBRRTSolver] Planning for robot " << robot_index << std::endl;
            std::cout << "  Region path length: " << region_path.size() << std::endl;
        }

        // 1. Create dynobench problem
        if (config_.debug) {
            std::cout << "[DBRRTSolver] Step 1: Creating dynobench problem..." << std::endl;
        }
        auto problem = createDynobenchProblem(robot, start_state, goal_state, decomp);
        if (!problem) {
            std::cerr << "[DBRRTSolver] Failed to create dynobench problem" << std::endl;
            return result;
        }

        // 2. Get robot type and determine motion primitives file
        std::string robot_type = getRobotType(robot);
        std::string motions_file = db_config_.motions_file;
        if (config_.debug) {
            std::cout << "[DBRRTSolver] Robot type: " << robot_type << std::endl;
            std::cout << "[DBRRTSolver] Motions file from config: " << motions_file << std::endl;
        }

        // Auto-detect motions file if not specified
        if (motions_file.empty()) {
            // Construct default path: models_base_path/../dynomotions/robot_type*.msgpack
            std::string motions_dir = db_config_.models_base_path + "/../dynomotions/";
            motions_file = motions_dir + robot_type + "_default.msgpack";
            if (config_.debug) {
                std::cout << "[DBRRTSolver] Auto-detected motions file: " << motions_file << std::endl;
            }
        }

        // 3. Create dynobench robot model
        // Note: robot_factory expects position-only bounds (translation_invariance dimensions)
        std::vector<double> min_bounds, max_bounds;
        getPositionBounds(robot, min_bounds, max_bounds);

        std::string model_file = db_config_.models_base_path + "/" + robot_type + ".yaml";
        auto dynobench_robot = dynobench::robot_factory(
            model_file.c_str(),
            Eigen::Map<Eigen::VectorXd>(min_bounds.data(), min_bounds.size()),
            Eigen::Map<Eigen::VectorXd>(max_bounds.data(), max_bounds.size()));

        if (!dynobench_robot) {
            std::cerr << "[DBRRTSolver] Failed to create dynobench robot model" << std::endl;
            return result;
        }

        // 4. Load environment into robot
        dynobench::load_env(*dynobench_robot, *problem);

        // 5. Load motion primitives
        std::vector<dynoplan::Motion> motions;
        if (!loadMotionPrimitives(dynobench_robot, motions)) {
            std::cerr << "[DBRRTSolver] Failed to load motion primitives" << std::endl;
            return result;
        }

        if (config_.debug) {
            std::cout << "[DBRRTSolver] Loaded " << motions.size() << " motion primitives" << std::endl;
        }

        // Diagnostic: Analyze motion primitive angular coverage
        if (config_.debug && !motions.empty()) {
            std::vector<double> start_angles;
            start_angles.reserve(motions.size());
            for (const auto& m : motions) {
                if (m.traj.states.size() > 0 && m.traj.states[0].size() >= 3) {
                    start_angles.push_back(m.traj.states[0](2));  // theta for unicycle
                }
            }
            if (!start_angles.empty()) {
                std::sort(start_angles.begin(), start_angles.end());
                double min_angle = start_angles.front();
                double max_angle = start_angles.back();

                // Find largest angular gap
                double max_gap = 0.0;
                for (size_t i = 1; i < start_angles.size(); ++i) {
                    double gap = start_angles[i] - start_angles[i-1];
                    max_gap = std::max(max_gap, gap);
                }
                // Also check wrap-around gap
                double wrap_gap = (2 * M_PI) - (max_angle - min_angle);
                max_gap = std::max(max_gap, wrap_gap);

                std::cout << "[DBRRTSolver] Motion primitive angular coverage:" << std::endl;
                std::cout << "  Angle range: [" << min_angle << ", " << max_angle << "] rad" << std::endl;
                std::cout << "  Largest angular gap: " << max_gap << " rad (" << (max_gap * 180 / M_PI) << " deg)" << std::endl;
                std::cout << "  Recommended min delta: " << (0.5 * max_gap / 2 + 0.1) << " (half-gap * weight + margin)" << std::endl;
                std::cout << "  Current delta: " << db_config_.delta << std::endl;

                // Warn if delta might be too small
                double min_recommended_delta = 0.5 * max_gap / 2 + 0.1;
                if (db_config_.delta < min_recommended_delta) {
                    std::cerr << "[DBRRTSolver] WARNING: delta=" << db_config_.delta
                              << " may be too small for motion primitive coverage. "
                              << "Consider delta >= " << min_recommended_delta << std::endl;
                }
            }
        }

        // Convert unique_ptr to shared_ptr for idbrrt call
        auto dynobench_robot_shared = std::shared_ptr<dynobench::Model_robot>(std::move(dynobench_robot));

        // 6. Configure DB-RRT options
        dynoplan::Options_dbrrt options_dbrrt;
        options_dbrrt.timelimit = db_config_.timelimit * 1000; // Convert to milliseconds
        options_dbrrt.max_expands = db_config_.max_expands;
        options_dbrrt.goal_region = db_config_.goal_region;
        options_dbrrt.delta = db_config_.delta;
        options_dbrrt.goal_bias = db_config_.goal_bias;
        options_dbrrt.max_motions = db_config_.max_motions;
        options_dbrrt.seed = db_config_.seed;
        options_dbrrt.do_optimization = db_config_.do_optimization;
        options_dbrrt.use_nigh_nn = db_config_.use_nigh_nn;
        options_dbrrt.debug = db_config_.debug;
        options_dbrrt.motionsFile = motions_file;
        options_dbrrt.motions_ptr = &motions;

        // Use region guidance if enabled
        if (db_config_.use_region_guidance && !region_path.empty()) {
            // TODO: Implement custom sampling function that biases towards regions
            // For now, we can use goal_bias to bias towards the goal
            options_dbrrt.goal_bias = std::max(options_dbrrt.goal_bias, db_config_.region_bias);
        }

        // 7. Configure trajectory optimization options
        dynoplan::Options_trajopt options_trajopt;
        options_trajopt.solver_id = db_config_.solver_id;

        // 8. Run DB-RRT planner
        if (config_.debug) {
            std::cout << "[DBRRTSolver] Running idbrrt planner..." << std::endl;
        }

        // Diagnostic: Check start state against motion primitives
        if (config_.debug) {
            Eigen::VectorXd canonical_start(problem->start.size());
            dynobench_robot_shared->canonical_state(problem->start, canonical_start);
            std::cout << "[DBRRTSolver] Start state: " << problem->start.transpose() << std::endl;
            std::cout << "[DBRRTSolver] Canonical start: " << canonical_start.transpose() << std::endl;

            // Count how many motions are within delta of start's canonical state
            int motions_in_range = 0;
            double closest_dist = std::numeric_limits<double>::max();
            for (const auto& m : motions) {
                if (m.traj.states.size() > 0) {
                    double d = dynobench_robot_shared->distance(m.traj.states[0], canonical_start);
                    if (d <= db_config_.delta) {
                        motions_in_range++;
                    }
                    closest_dist = std::min(closest_dist, d);
                }
            }
            std::cout << "[DBRRTSolver] Motions within delta=" << db_config_.delta
                      << " of start: " << motions_in_range << "/" << motions.size() << std::endl;
            std::cout << "[DBRRTSolver] Closest motion distance: " << closest_dist << std::endl;

            if (motions_in_range == 0) {
                std::cerr << "[DBRRTSolver] WARNING: No motion primitives within delta of start state!" << std::endl;
                std::cerr << "[DBRRTSolver] This will cause num_nn_motions=0. Consider increasing delta to >= "
                          << closest_dist << std::endl;
            }
        }

        dynobench::Trajectory traj_out;
        dynobench::Info_out out_info;

        dynoplan::idbrrt(*problem, dynobench_robot_shared, options_dbrrt, options_trajopt, traj_out, out_info);

        // 9. Process results
        if (out_info.solved) {
            if (config_.debug) {
                std::cout << "[DBRRTSolver] Solution found!" << std::endl;
                std::cout << "  Cost: " << out_info.cost << std::endl;
            }

            // Convert trajectory to OMPL format
            result.path = convertTrajectory(traj_out, robot);
            result.success = (result.path != nullptr);

            if (!result.success) {
                std::cerr << "[DBRRTSolver] Failed to convert trajectory" << std::endl;
            }
        } else {
            if (config_.debug) {
                std::cout << "[DBRRTSolver] No solution found" << std::endl;
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "[DBRRTSolver] Exception: " << e.what() << std::endl;
        result.success = false;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    result.planning_time = std::chrono::duration<double>(end_time - start_time).count();

    if (config_.debug) {
        std::cout << "[DBRRTSolver] Planning finished in " << result.planning_time << "s" << std::endl;
    }

    return result;
}

std::vector<double> DBRRTSolver::omplStateToDynobench(
    const ob::State* state,
    const std::shared_ptr<::Robot>& robot) {

    if (!state || !robot) {
        return {};
    }

    auto si = robot->getSpaceInformation();
    if (!si) {
        return {};
    }

    auto state_space = si->getStateSpace();
    if (!state_space) {
        return {};
    }

    std::vector<double> dynobench_state;

    if (auto compound = state_space->as<ob::CompoundStateSpace>()) {
        // Handle compound state space
        const auto* compound_state = state->as<ob::CompoundState>();
        if (!compound_state) {
            return dynobench_state;
        }

        for (size_t i = 0; i < compound->getSubspaceCount(); ++i) {
            auto subspace = compound->getSubspace(i);
            const auto* substate = compound_state->as<ob::State>(i);

            if (!substate) {
                continue;
            }

            // Check SO2 first since it may be a subclass of RealVectorStateSpace in OMPL
            if (subspace->getType() == ob::STATE_SPACE_SO2) {
                const auto* so2_state = substate->as<ob::SO2StateSpace::StateType>();
                if (!so2_state) {
                    continue;
                }
                dynobench_state.push_back(so2_state->value);
            } else if (subspace->getType() == ob::STATE_SPACE_REAL_VECTOR) {
                auto rv_space = subspace->as<ob::RealVectorStateSpace>();
                const auto* rv_state = substate->as<ob::RealVectorStateSpace::StateType>();
                if (!rv_state) {
                    continue;
                }
                for (size_t j = 0; j < rv_space->getDimension(); ++j) {
                    dynobench_state.push_back(rv_state->values[j]);
                }
            }
        }
    } else if (auto rv_space = state_space->as<ob::RealVectorStateSpace>()) {
        // Handle simple real vector state space
        const auto* rv_state = state->as<ob::RealVectorStateSpace::StateType>();
        if (!rv_state) {
            return dynobench_state;
        }
        for (size_t i = 0; i < rv_space->getDimension(); ++i) {
            dynobench_state.push_back(rv_state->values[i]);
        }
    }

    return dynobench_state;
}

void DBRRTSolver::dynobenchStateToOmpl(
    const std::vector<double>& state,
    ob::State* ompl_state,
    const std::shared_ptr<::Robot>& robot) {

    auto si = robot->getSpaceInformation();
    auto state_space = si->getStateSpace();

    size_t idx = 0;

    if (auto compound = state_space->as<ob::CompoundStateSpace>()) {
        auto* compound_state = ompl_state->as<ob::CompoundState>();

        for (size_t i = 0; i < compound->getSubspaceCount(); ++i) {
            auto subspace = compound->getSubspace(i);
            auto* substate = compound_state->as<ob::State>(i);

            // Check SO2 first (by type), matching omplStateToDynobench order
            if (subspace->getType() == ob::STATE_SPACE_SO2) {
                auto* so2_state = substate->as<ob::SO2StateSpace::StateType>();
                so2_state->value = state[idx++];
            } else if (subspace->getType() == ob::STATE_SPACE_REAL_VECTOR) {
                auto rv_space = subspace->as<ob::RealVectorStateSpace>();
                auto* rv_state = substate->as<ob::RealVectorStateSpace::StateType>();
                for (size_t j = 0; j < rv_space->getDimension(); ++j) {
                    rv_state->values[j] = state[idx++];
                }
            }
        }
    } else if (auto rv_space = state_space->as<ob::RealVectorStateSpace>()) {
        auto* rv_state = ompl_state->as<ob::RealVectorStateSpace::StateType>();
        for (size_t i = 0; i < rv_space->getDimension(); ++i) {
            rv_state->values[i] = state[idx++];
        }
    }
}

std::shared_ptr<dynobench::Problem> DBRRTSolver::createDynobenchProblem(
    const std::shared_ptr<::Robot>& robot,
    const ob::State* start_state,
    const ob::State* goal_state,
    const oc::DecompositionPtr& decomp) {

    auto problem = std::make_shared<dynobench::Problem>();

    // Set robot type (both singular and plural forms for compatibility)
    problem->robotType = getRobotType(robot);
    problem->robotTypes.push_back(problem->robotType);  // Required for trajectory_optimization

    // Set models base path
    problem->models_base_path = db_config_.models_base_path;

    // Convert start and goal states
    auto start_vec = omplStateToDynobench(start_state, robot);
    auto goal_vec = omplStateToDynobench(goal_state, robot);

    problem->start = Eigen::Map<Eigen::VectorXd>(start_vec.data(), start_vec.size());
    problem->goal = Eigen::Map<Eigen::VectorXd>(goal_vec.data(), goal_vec.size());

    // Get position bounds from robot's state space
    // For unicycle (SE2), this is just the x,y position bounds, not theta
    std::vector<double> min_bounds, max_bounds;
    getPositionBounds(robot, min_bounds, max_bounds);

    // Set position bounds (only x,y for unicycle, or x,y,z for 3D robots)
    size_t pos_dim = min_bounds.size();
    problem->p_lb.resize(pos_dim);
    problem->p_ub.resize(pos_dim);

    for (size_t i = 0; i < pos_dim; ++i) {
        problem->p_lb(i) = min_bounds[i];
        problem->p_ub(i) = max_bounds[i];
    }

    // Add precomputed obstacles
    problem->obstacles = dynobench_obstacles_;

    return problem;
}

std::shared_ptr<oc::PathControl> DBRRTSolver::convertTrajectory(
    const dynobench::Trajectory& traj,
    const std::shared_ptr<::Robot>& robot) {

    auto si = robot->getSpaceInformation();

    // Create path control
    auto path = std::make_shared<oc::PathControl>(si);

    // Convert states and controls
    for (size_t i = 0; i < traj.states.size(); ++i) {
        // Convert state
        ob::State* state = si->allocState();
        std::vector<double> state_vec(traj.states[i].data(),
                                       traj.states[i].data() + traj.states[i].size());
        dynobenchStateToOmpl(state_vec, state, robot);

        if (i < traj.actions.size()) {
            // Convert control
            oc::Control* control = si->allocControl();
            auto* rv_control = control->as<oc::RealVectorControlSpace::ControlType>();

            for (size_t j = 0; j < traj.actions[i].size() && j < si->getControlSpace()->getDimension(); ++j) {
                rv_control->values[j] = traj.actions[i](j);
            }

            // Compute duration (assuming fixed timestep if not provided)
            double duration = 0.1; // Default timestep
            // TODO: Get actual duration from trajectory if available

            path->append(state, control, duration);
        } else {
            // Last state, no control
            path->append(state);
        }
    }

    return path;
}

bool DBRRTSolver::loadMotionPrimitives(
    const std::unique_ptr<dynobench::Model_robot>& dynobench_robot,
    std::vector<dynoplan::Motion>& motions) {

    std::string robot_type = getRobotType(nullptr); // Get from config
    std::string motions_file = db_config_.motions_file;

    // Note: Motion caching disabled because dynoplan::Motion contains unique_ptr
    // which is not copyable. Motions must be reloaded each time.

    try {
        dynoplan::load_motion_primitives_new(
            motions_file,
            *dynobench_robot,
            motions,
            db_config_.max_motions,
            false,  // cut_actions
            false,  // shuffle
            true    // compute_col - required for collision checking with invariance_reuse_col_shape
        );

        return !motions.empty();
    } catch (const std::exception& e) {
        std::cerr << "[DBRRTSolver] Failed to load motion primitives: " << e.what() << std::endl;
        return false;
    }
}

std::string DBRRTSolver::getRobotType(const std::shared_ptr<::Robot>& robot) {
    // TODO: Determine robot type from Robot object
    // For now, return a default based on configuration or detection
    // This should match dynobench robot types: unicycle1_v0, quad2d_v0, etc.

    // Simple heuristic: check state space dimension
    if (robot) {
        auto si = robot->getSpaceInformation();
        size_t state_dim = si->getStateDimension();

        if (state_dim == 3) {
            return "unicycle1_v0";
        } else if (state_dim == 4) {
            return "integrator2_2d_v0";
        } else if (state_dim == 8) {
            return "quad2d_v0";
        } else if (state_dim == 13) {
            return "quad3d_v0";
        }
    }

    // Default fallback
    return "unicycle1_v0";
}

void DBRRTSolver::getStateBounds(
    const std::shared_ptr<::Robot>& robot,
    std::vector<double>& min_bounds,
    std::vector<double>& max_bounds) {

    auto si = robot->getSpaceInformation();
    auto state_space = si->getStateSpace();

    min_bounds.clear();
    max_bounds.clear();

    if (auto compound = state_space->as<ob::CompoundStateSpace>()) {
        // Handle compound state space (e.g., SE2 = R^2 + SO2)
        for (size_t i = 0; i < compound->getSubspaceCount(); ++i) {
            auto subspace = compound->getSubspace(i);
            if (subspace->getType() == ob::STATE_SPACE_REAL_VECTOR) {
                auto rv_space = subspace->as<ob::RealVectorStateSpace>();
                ob::RealVectorBounds bounds = rv_space->getBounds();
                for (size_t j = 0; j < bounds.low.size(); ++j) {
                    min_bounds.push_back(bounds.low[j]);
                    max_bounds.push_back(bounds.high[j]);
                }
            } else if (subspace->getType() == ob::STATE_SPACE_SO2) {
                // SO2 has bounds [-pi, pi]
                min_bounds.push_back(-M_PI);
                max_bounds.push_back(M_PI);
            }
        }
    } else if (auto rv_space = state_space->as<ob::RealVectorStateSpace>()) {
        ob::RealVectorBounds bounds = rv_space->getBounds();
        for (size_t i = 0; i < bounds.low.size(); ++i) {
            min_bounds.push_back(bounds.low[i]);
            max_bounds.push_back(bounds.high[i]);
        }
    }
}

void DBRRTSolver::getPositionBounds(
    const std::shared_ptr<::Robot>& robot,
    std::vector<double>& min_bounds,
    std::vector<double>& max_bounds) {

    auto si = robot->getSpaceInformation();
    auto state_space = si->getStateSpace();

    min_bounds.clear();
    max_bounds.clear();

    // For dynobench, we only need position bounds (translation_invariance dimensions)
    // This means only the RealVector part (x,y for 2D, x,y,z for 3D), not SO2/SO3
    if (auto compound = state_space->as<ob::CompoundStateSpace>()) {
        // Handle compound state space (e.g., SE2 = R^2 + SO2)
        // Only extract bounds from RealVector subspaces (position), skip SO2 (orientation)
        for (size_t i = 0; i < compound->getSubspaceCount(); ++i) {
            auto subspace = compound->getSubspace(i);
            if (subspace->getType() == ob::STATE_SPACE_REAL_VECTOR) {
                auto rv_space = subspace->as<ob::RealVectorStateSpace>();
                ob::RealVectorBounds bounds = rv_space->getBounds();
                for (size_t j = 0; j < bounds.low.size(); ++j) {
                    min_bounds.push_back(bounds.low[j]);
                    max_bounds.push_back(bounds.high[j]);
                }
            }
            // Skip SO2/SO3 subspaces for position bounds
        }
    } else if (auto rv_space = state_space->as<ob::RealVectorStateSpace>()) {
        ob::RealVectorBounds bounds = rv_space->getBounds();
        for (size_t i = 0; i < bounds.low.size(); ++i) {
            min_bounds.push_back(bounds.low[i]);
            max_bounds.push_back(bounds.high[i]);
        }
    }
}

std::function<void(Eigen::Ref<Eigen::VectorXd>)> DBRRTSolver::createRegionGuidedSampler(
    const std::vector<int>& region_path,
    const oc::DecompositionPtr& decomp) {

    // Create a region-biased sampler that samples from regions along the lead path
    // with probability prob_follow_lead, otherwise samples uniformly

    return [this, region_path, decomp](Eigen::Ref<Eigen::VectorXd> sample) {
        static std::mt19937 gen(std::random_device{}());
        static std::uniform_real_distribution<> dis(0.0, 1.0);

        if (dis(gen) < db_config_.prob_follow_lead && !region_path.empty()) {
            // Sample from a region along the lead path
            std::uniform_int_distribution<> region_dis(0, region_path.size() - 1);
            int region_idx = region_dis(gen);
            int region_id = region_path[region_idx];

            Eigen::VectorXd region_sample = sampleInRegion(region_id, decomp);
            // Only copy the position dimensions (2D for now)
            for (int i = 0; i < std::min((int)region_sample.size(), (int)sample.size()); ++i) {
                sample(i) = region_sample(i);
            }
        }
        // Otherwise, the sample remains as uniform random (set by caller)
    };
}

// =============================================================================
// SYCLOP-style Region Guidance Implementation
// =============================================================================

GuidedPlanningResult DBRRTSolver::solveWithRegionGuidance(
    std::shared_ptr<::Robot> robot,
    oc::DecompositionPtr decomp,
    ob::State* start_state,
    ob::State* goal_state,
    const std::vector<int>& region_path,
    size_t robot_index) {

    GuidedPlanningResult result;
    result.robot_index = robot_index;
    result.success = false;

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        // Convert OMPL states to Eigen vectors
        auto start_vec = omplStateToDynobench(start_state, robot);
        auto goal_vec = omplStateToDynobench(goal_state, robot);

        Eigen::VectorXd current_state = Eigen::Map<Eigen::VectorXd>(
            start_vec.data(), start_vec.size());
        Eigen::VectorXd final_goal = Eigen::Map<Eigen::VectorXd>(
            goal_vec.data(), goal_vec.size());

        if (config_.debug) {
            std::cout << "[DBRRTSolver] SYCLOP-style planning:" << std::endl;
            std::cout << "  Start: " << current_state.transpose() << std::endl;
            std::cout << "  Goal: " << final_goal.transpose() << std::endl;
            std::cout << "  Lead path length: " << region_path.size() << std::endl;
        }

        // Track trajectory segments
        std::vector<dynobench::Trajectory> trajectory_segments;

        // Time budget management - reserve some time for fallback direct planning
        double total_time_budget = db_config_.timelimit;
        double region_guided_budget = total_time_budget * 0.7;  // Use 70% for region-guided
        double remaining_time = region_guided_budget;
        double time_per_segment = region_guided_budget / std::max((size_t)1, region_path.size());

        // Current position in the lead (region path)
        size_t lead_index = 0;
        int consecutive_failures = 0;
        const int max_consecutive_failures = 3;

        // Random generator for probabilistic lead following
        std::mt19937 gen(db_config_.seed >= 0 ? db_config_.seed : std::random_device{}());
        std::uniform_real_distribution<> prob_dis(0.0, 1.0);

        // Main SYCLOP-style loop: follow the lead through regions
        while (lead_index < region_path.size() && remaining_time > 0 &&
               consecutive_failures < max_consecutive_failures) {
            int current_region = region_path[lead_index];
            bool is_final_segment = (lead_index == region_path.size() - 1);

            // Determine waypoint goal for this segment
            Eigen::VectorXd waypoint_goal;
            if (is_final_segment) {
                // Last segment: go to the actual goal
                waypoint_goal = final_goal;
            } else {
                // Intermediate segment: create waypoint in next region
                // Bias waypoint toward the final goal direction
                int next_region = region_path[lead_index + 1];
                waypoint_goal = createWaypointGoal(next_region, current_state,
                                                   final_goal, decomp);
            }

            if (config_.debug) {
                std::cout << "[DBRRTSolver] Segment " << lead_index << "/"
                          << region_path.size() - 1 << ":" << std::endl;
                std::cout << "  From region " << current_region << " to "
                          << (is_final_segment ? "GOAL" : std::to_string(region_path[lead_index + 1]))
                          << std::endl;
                std::cout << "  Current: " << current_state.head(2).transpose() << std::endl;
                std::cout << "  Waypoint: " << waypoint_goal.head(2).transpose() << std::endl;
            }

            // Plan segment to waypoint
            double segment_time = std::min(time_per_segment * 2.0, remaining_time);
            auto [segment_success, segment_traj] = planSegmentToWaypoint(
                robot, current_state, waypoint_goal, decomp, segment_time);

            if (segment_success && !segment_traj.states.empty()) {
                trajectory_segments.push_back(segment_traj);
                consecutive_failures = 0;  // Reset failure counter

                // Update current state to end of this segment
                current_state = segment_traj.states.back();

                if (config_.debug) {
                    std::cout << "  Segment succeeded, " << segment_traj.states.size()
                              << " states" << std::endl;
                    std::cout << "  New position: " << current_state.head(2).transpose()
                              << std::endl;
                }

                // Check if we've reached close enough to waypoint to advance lead
                double dist_to_waypoint = (current_state.head(2) - waypoint_goal.head(2)).norm();
                if (dist_to_waypoint < db_config_.waypoint_tolerance || is_final_segment) {
                    lead_index++;
                }

                // Probabilistic early termination if we're close to final goal
                double dist_to_goal = (current_state.head(2) - final_goal.head(2)).norm();
                if (dist_to_goal < db_config_.goal_region) {
                    if (config_.debug) {
                        std::cout << "  Reached goal region early!" << std::endl;
                    }
                    break;
                }

                // Probabilistic shortcut: try going directly to goal
                if (!is_final_segment && prob_dis(gen) < db_config_.prob_abandon_lead_early) {
                    if (config_.debug) {
                        std::cout << "  Attempting shortcut to goal..." << std::endl;
                    }
                    auto [shortcut_success, shortcut_traj] = planSegmentToWaypoint(
                        robot, current_state, final_goal, decomp, segment_time / 2.0);

                    if (shortcut_success && !shortcut_traj.states.empty()) {
                        double shortcut_dist = (shortcut_traj.states.back().head(2) -
                                                final_goal.head(2)).norm();
                        if (shortcut_dist < db_config_.goal_region) {
                            trajectory_segments.push_back(shortcut_traj);
                            current_state = shortcut_traj.states.back();
                            if (config_.debug) {
                                std::cout << "  Shortcut succeeded!" << std::endl;
                            }
                            break;
                        }
                    }
                }

            } else {
                consecutive_failures++;
                if (config_.debug) {
                    std::cout << "  Segment failed (failure " << consecutive_failures
                              << "/" << max_consecutive_failures << "), trying next region..."
                              << std::endl;
                }
                // Segment failed, try advancing anyway or retry with different waypoint
                lead_index++;
            }

            // Update remaining time
            auto current_time = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(current_time - start_time).count();
            remaining_time = region_guided_budget - elapsed;
        }

        // Check if region-guided planning succeeded
        bool region_guided_success = false;
        if (!trajectory_segments.empty()) {
            dynobench::Trajectory full_traj = concatenateTrajectories(trajectory_segments);

            // Check if we reached the goal
            double final_dist = (full_traj.states.back().head(2) - final_goal.head(2)).norm();
            region_guided_success = (final_dist < db_config_.goal_region);

            if (config_.debug) {
                std::cout << "[DBRRTSolver] Region-guided trajectory:" << std::endl;
                std::cout << "  Total states: " << full_traj.states.size() << std::endl;
                std::cout << "  Final distance to goal: " << final_dist << std::endl;
                std::cout << "  Success: " << region_guided_success << std::endl;
            }

            if (region_guided_success) {
                result.success = true;
                result.path = convertTrajectory(full_traj, robot);
            }
        }

        // If region-guided planning failed, fall back to direct planning
        if (!region_guided_success) {
            auto current_time = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(current_time - start_time).count();
            double fallback_time = total_time_budget - elapsed;

            if (fallback_time > 1.0) {  // Only try fallback if we have at least 1 second
                if (config_.debug) {
                    std::cout << "[DBRRTSolver] Region-guided planning failed, "
                              << "falling back to direct planning with " << fallback_time
                              << "s remaining" << std::endl;
                }

                // Try direct planning from original start to goal
                auto [direct_success, direct_traj] = planSegmentToWaypoint(
                    robot, Eigen::Map<Eigen::VectorXd>(start_vec.data(), start_vec.size()),
                    final_goal, decomp, fallback_time);

                if (direct_success && !direct_traj.states.empty()) {
                    double final_dist = (direct_traj.states.back().head(2) -
                                         final_goal.head(2)).norm();
                    if (final_dist < db_config_.goal_region) {
                        result.success = true;
                        result.path = convertTrajectory(direct_traj, robot);
                        if (config_.debug) {
                            std::cout << "[DBRRTSolver] Direct fallback succeeded!" << std::endl;
                        }
                    }
                }
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "[DBRRTSolver] Exception in region-guided planning: " << e.what()
                  << std::endl;
        result.success = false;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    result.planning_time = std::chrono::duration<double>(end_time - start_time).count();

    return result;
}

Eigen::VectorXd DBRRTSolver::getRegionCenter(int region_id,
                                             const oc::DecompositionPtr& decomp) {
    // Cast to our GridDecompositionImpl to access region bounds
    auto grid_decomp = std::dynamic_pointer_cast<GridDecompositionImpl>(decomp);
    if (!grid_decomp) {
        std::cerr << "[DBRRTSolver] Warning: Could not cast to GridDecompositionImpl"
                  << std::endl;
        return Eigen::VectorXd::Zero(2);
    }

    const auto& bounds = grid_decomp->getRegionBoundsPublic(region_id);

    Eigen::VectorXd center(bounds.low.size());
    for (size_t i = 0; i < bounds.low.size(); ++i) {
        center(i) = (bounds.low[i] + bounds.high[i]) / 2.0;
    }
    return center;
}

Eigen::VectorXd DBRRTSolver::sampleInRegion(int region_id,
                                            const oc::DecompositionPtr& decomp) {
    auto grid_decomp = std::dynamic_pointer_cast<GridDecompositionImpl>(decomp);
    if (!grid_decomp) {
        return Eigen::VectorXd::Zero(2);
    }

    const auto& bounds = grid_decomp->getRegionBoundsPublic(region_id);

    static std::mt19937 gen(std::random_device{}());

    Eigen::VectorXd sample(bounds.low.size());
    for (size_t i = 0; i < bounds.low.size(); ++i) {
        std::uniform_real_distribution<> dis(bounds.low[i], bounds.high[i]);
        sample(i) = dis(gen);
    }
    return sample;
}

bool DBRRTSolver::isStateInRegion(const Eigen::VectorXd& state, int region_id,
                                  const oc::DecompositionPtr& decomp) {
    auto grid_decomp = std::dynamic_pointer_cast<GridDecompositionImpl>(decomp);
    if (!grid_decomp) {
        return false;
    }

    const auto& bounds = grid_decomp->getRegionBoundsPublic(region_id);

    // Check if position (first 2 dimensions) is within bounds
    for (size_t i = 0; i < std::min((size_t)2, bounds.low.size()); ++i) {
        if (state(i) < bounds.low[i] || state(i) > bounds.high[i]) {
            return false;
        }
    }
    return true;
}

size_t DBRRTSolver::findCurrentLeadIndex(const Eigen::VectorXd& state,
                                         const std::vector<int>& region_path,
                                         const oc::DecompositionPtr& decomp) {
    // Find which region in the lead path the state is closest to
    double min_dist = std::numeric_limits<double>::max();
    size_t best_idx = 0;

    for (size_t i = 0; i < region_path.size(); ++i) {
        Eigen::VectorXd center = getRegionCenter(region_path[i], decomp);
        double dist = (state.head(center.size()) - center).norm();
        if (dist < min_dist) {
            min_dist = dist;
            best_idx = i;
        }
    }
    return best_idx;
}

Eigen::VectorXd DBRRTSolver::createWaypointGoal(int region_id,
                                                const Eigen::VectorXd& current_state,
                                                const Eigen::VectorXd& final_goal,
                                                const oc::DecompositionPtr& decomp) {
    // Get region center
    Eigen::VectorXd center = getRegionCenter(region_id, decomp);

    // Create waypoint: region center position with interpolated orientation toward goal
    Eigen::VectorXd waypoint = current_state;  // Copy full state

    // Set position to region center
    waypoint.head(center.size()) = center;

    // For unicycle (state = [x, y, theta]), compute heading toward final goal
    if (waypoint.size() >= 3) {
        Eigen::VectorXd goal_pos = final_goal.head(2);
        Eigen::VectorXd waypoint_pos = center.head(2);
        double dx = goal_pos(0) - waypoint_pos(0);
        double dy = goal_pos(1) - waypoint_pos(1);
        waypoint(2) = std::atan2(dy, dx);  // theta pointing toward goal
    }

    // For higher-dimensional states (with velocity), keep velocities small
    for (int i = 3; i < waypoint.size(); ++i) {
        waypoint(i) = 0.0;  // Zero velocity at waypoints
    }

    return waypoint;
}

std::pair<bool, dynobench::Trajectory> DBRRTSolver::planSegmentToWaypoint(
    const std::shared_ptr<::Robot>& robot,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& waypoint,
    const oc::DecompositionPtr& decomp,
    double time_budget) {

    dynobench::Trajectory traj_out;

    try {
        // Create a sub-problem for this segment
        auto problem = std::make_shared<dynobench::Problem>();
        problem->robotType = getRobotType(robot);
        problem->robotTypes.push_back(problem->robotType);
        problem->models_base_path = db_config_.models_base_path;
        problem->start = start;
        problem->goal = waypoint;

        // Get position bounds
        std::vector<double> min_bounds, max_bounds;
        getPositionBounds(robot, min_bounds, max_bounds);

        size_t pos_dim = min_bounds.size();
        problem->p_lb.resize(pos_dim);
        problem->p_ub.resize(pos_dim);
        for (size_t i = 0; i < pos_dim; ++i) {
            problem->p_lb(i) = min_bounds[i];
            problem->p_ub(i) = max_bounds[i];
        }

        problem->obstacles = dynobench_obstacles_;

        // Create robot model
        std::string model_file = db_config_.models_base_path + "/" + problem->robotType + ".yaml";
        auto dynobench_robot = dynobench::robot_factory(
            model_file.c_str(),
            Eigen::Map<Eigen::VectorXd>(min_bounds.data(), min_bounds.size()),
            Eigen::Map<Eigen::VectorXd>(max_bounds.data(), max_bounds.size()));

        if (!dynobench_robot) {
            return {false, traj_out};
        }

        dynobench::load_env(*dynobench_robot, *problem);

        // Load motion primitives
        std::vector<dynoplan::Motion> motions;
        if (!loadMotionPrimitives(dynobench_robot, motions)) {
            return {false, traj_out};
        }

        auto dynobench_robot_shared = std::shared_ptr<dynobench::Model_robot>(
            std::move(dynobench_robot));

        // Configure DB-RRT for this segment
        dynoplan::Options_dbrrt options;
        options.timelimit = time_budget * 1000;  // Convert to ms
        options.max_expands = db_config_.max_expands / 4;  // Fewer expands per segment
        options.goal_region = db_config_.waypoint_tolerance;  // Use waypoint tolerance
        options.delta = db_config_.delta;
        options.goal_bias = 0.2;  // Higher goal bias for directed planning
        options.max_motions = db_config_.max_motions;
        options.seed = db_config_.seed;
        options.do_optimization = false;  // No optimization for intermediate segments
        options.use_nigh_nn = db_config_.use_nigh_nn;
        options.debug = false;  // Reduce debug output for segments
        options.motionsFile = db_config_.motions_file;
        options.motions_ptr = &motions;

        dynoplan::Options_trajopt options_trajopt;
        options_trajopt.solver_id = db_config_.solver_id;

        dynobench::Info_out out_info;

        // Run DB-RRT
        dynoplan::idbrrt(*problem, dynobench_robot_shared, options, options_trajopt,
                         traj_out, out_info);

        return {out_info.solved, traj_out};

    } catch (const std::exception& e) {
        std::cerr << "[DBRRTSolver] Segment planning exception: " << e.what() << std::endl;
        return {false, traj_out};
    }
}

dynobench::Trajectory DBRRTSolver::concatenateTrajectories(
    const std::vector<dynobench::Trajectory>& segments) {

    dynobench::Trajectory result;

    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];

        if (seg.states.empty()) continue;

        if (i == 0) {
            // First segment: add all states
            result.states.insert(result.states.end(),
                                 seg.states.begin(), seg.states.end());
            result.actions.insert(result.actions.end(),
                                  seg.actions.begin(), seg.actions.end());
        } else {
            // Subsequent segments: skip first state (overlap with previous end)
            if (seg.states.size() > 1) {
                result.states.insert(result.states.end(),
                                     seg.states.begin() + 1, seg.states.end());
            }
            result.actions.insert(result.actions.end(),
                                  seg.actions.begin(), seg.actions.end());
        }
    }

    return result;
}

// Load configuration from YAML
DBRRTConfig loadDBRRTConfig(const YAML::Node& node) {
    DBRRTConfig config;

    if (node["motions_file"]) {
        config.motions_file = node["motions_file"].as<std::string>();
    }
    if (node["models_base_path"]) {
        config.models_base_path = node["models_base_path"].as<std::string>();
    }
    if (node["timelimit"]) {
        config.timelimit = node["timelimit"].as<double>();
    }
    if (node["max_expands"]) {
        config.max_expands = node["max_expands"].as<int>();
    }
    if (node["goal_region"]) {
        config.goal_region = node["goal_region"].as<double>();
    }
    if (node["delta"]) {
        config.delta = node["delta"].as<double>();
    }
    if (node["goal_bias"]) {
        config.goal_bias = node["goal_bias"].as<double>();
    }
    if (node["max_motions"]) {
        config.max_motions = node["max_motions"].as<int>();
    }
    if (node["seed"]) {
        config.seed = node["seed"].as<int>();
    }
    if (node["do_optimization"]) {
        config.do_optimization = node["do_optimization"].as<bool>();
    }
    if (node["use_nigh_nn"]) {
        config.use_nigh_nn = node["use_nigh_nn"].as<bool>();
    }
    if (node["debug"]) {
        config.debug = node["debug"].as<bool>();
    }
    if (node["solver_id"]) {
        config.solver_id = node["solver_id"].as<int>();
    }
    if (node["use_region_guidance"]) {
        config.use_region_guidance = node["use_region_guidance"].as<bool>();
    }
    if (node["region_bias"]) {
        config.region_bias = node["region_bias"].as<double>();
    }

    // SYCLOP-style lead following parameters
    if (node["num_region_expansions"]) {
        config.num_region_expansions = node["num_region_expansions"].as<int>();
    }
    if (node["prob_follow_lead"]) {
        config.prob_follow_lead = node["prob_follow_lead"].as<double>();
    }
    if (node["prob_abandon_lead_early"]) {
        config.prob_abandon_lead_early = node["prob_abandon_lead_early"].as<double>();
    }
    if (node["use_waypoint_planning"]) {
        config.use_waypoint_planning = node["use_waypoint_planning"].as<bool>();
    }
    if (node["waypoint_tolerance"]) {
        config.waypoint_tolerance = node["waypoint_tolerance"].as<double>();
    }

    return config;
}

} // namespace mr_syclop
