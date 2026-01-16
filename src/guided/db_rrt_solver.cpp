/**
 * @file db_rrt_solver.cpp
 * @brief Implementation of DB-RRT solver for mr_syclop
 */

// Pinocchio/Crocoddyl must be included before Boost/OMPL headers
#include <dynoplan/optimization/ocp.hpp>

#include "db_rrt_solver.h"
#include "robots.h"

#include <chrono>
#include <iostream>
#include <stdexcept>

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

    try {
        if (config_.debug) {
            std::cout << "[DBRRTSolver] Planning for robot " << robot_index << std::endl;
            std::cout << "  Region path length: " << region_path.size() << std::endl;
        }

        // 1. Create dynobench problem
        std::cout << "[DBRRTSolver] Step 1: Creating dynobench problem..." << std::endl;
        auto problem = createDynobenchProblem(robot, start_state, goal_state, decomp);
        if (!problem) {
            std::cerr << "[DBRRTSolver] Failed to create dynobench problem" << std::endl;
            return result;
        }
        std::cout << "[DBRRTSolver] Step 1: Done" << std::endl;

        // 2. Get robot type and determine motion primitives file
        std::cout << "[DBRRTSolver] Step 2: Getting robot type..." << std::endl;
        std::string robot_type = getRobotType(robot);
        std::string motions_file = db_config_.motions_file;
        std::cout << "[DBRRTSolver] Robot type: " << robot_type << std::endl;
        std::cout << "[DBRRTSolver] Motions file from config: " << motions_file << std::endl;

        // Auto-detect motions file if not specified
        if (motions_file.empty()) {
            // Construct default path: models_base_path/../dynomotions/robot_type*.msgpack
            std::string motions_dir = db_config_.models_base_path + "/../dynomotions/";
            motions_file = motions_dir + robot_type + "_default.msgpack";
            std::cout << "[DBRRTSolver] Auto-detected motions file: " << motions_file << std::endl;
        }

        // 3. Create dynobench robot model
        // Note: robot_factory expects position-only bounds (translation_invariance dimensions)
        std::cout << "[DBRRTSolver] Step 3: Creating dynobench robot model..." << std::endl;
        std::vector<double> min_bounds, max_bounds;
        getPositionBounds(robot, min_bounds, max_bounds);
        std::cout << "[DBRRTSolver] Got position bounds, min size: " << min_bounds.size() << ", max size: " << max_bounds.size() << std::endl;

        std::string model_file = db_config_.models_base_path + "/" + robot_type + ".yaml";
        std::cout << "[DBRRTSolver] Model file: " << model_file << std::endl;
        auto dynobench_robot = dynobench::robot_factory(
            model_file.c_str(),
            Eigen::Map<Eigen::VectorXd>(min_bounds.data(), min_bounds.size()),
            Eigen::Map<Eigen::VectorXd>(max_bounds.data(), max_bounds.size()));

        if (!dynobench_robot) {
            std::cerr << "[DBRRTSolver] Failed to create dynobench robot model" << std::endl;
            return result;
        }
        std::cout << "[DBRRTSolver] Step 3: Done" << std::endl;

        // 4. Load environment into robot
        std::cout << "[DBRRTSolver] Step 4: Loading environment..." << std::endl;
        dynobench::load_env(*dynobench_robot, *problem);
        std::cout << "[DBRRTSolver] Step 4: Done" << std::endl;

        // 5. Load motion primitives
        std::cout << "[DBRRTSolver] Step 5: Loading motion primitives from " << motions_file << "..." << std::endl;
        std::vector<dynoplan::Motion> motions;
        if (!loadMotionPrimitives(dynobench_robot, motions)) {
            std::cerr << "[DBRRTSolver] Failed to load motion primitives" << std::endl;
            return result;
        }

        if (config_.debug) {
            std::cout << "[DBRRTSolver] Loaded " << motions.size() << " motion primitives" << std::endl;
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

        dynobench::Trajectory traj_out;
        dynobench::Info_out out_info;

        std::cout << "[DBRRTSolver] Calling idbrrt..." << std::endl;
        std::cout.flush();
        dynoplan::idbrrt(*problem, dynobench_robot_shared, options_dbrrt, options_trajopt, traj_out, out_info);
        std::cout << "[DBRRTSolver] idbrrt returned" << std::endl;
        std::cout.flush();

        std::cout << "[DBRRTSolver] idbrrt solved=" << out_info.solved << std::endl;
        std::cout.flush();

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

        std::cout << "[DBRRTSolver] Processing complete, cleaning up..." << std::endl;
        std::cout.flush();

    } catch (const std::exception& e) {
        std::cerr << "[DBRRTSolver] Exception: " << e.what() << std::endl;
        result.success = false;
    }

    std::cout << "[DBRRTSolver] Exiting solve(), success=" << result.success << std::endl;
    std::cout.flush();

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

    std::cout << "[DBRRTSolver] omplStateToDynobench: entering..." << std::endl;

    if (!state) {
        std::cerr << "[DBRRTSolver] omplStateToDynobench: ERROR - state is null!" << std::endl;
        return {};
    }

    if (!robot) {
        std::cerr << "[DBRRTSolver] omplStateToDynobench: ERROR - robot is null!" << std::endl;
        return {};
    }

    auto si = robot->getSpaceInformation();
    if (!si) {
        std::cerr << "[DBRRTSolver] omplStateToDynobench: ERROR - space information is null!" << std::endl;
        return {};
    }

    auto state_space = si->getStateSpace();
    if (!state_space) {
        std::cerr << "[DBRRTSolver] omplStateToDynobench: ERROR - state space is null!" << std::endl;
        return {};
    }

    std::cout << "[DBRRTSolver] omplStateToDynobench: state space name = " << state_space->getName() << std::endl;

    std::vector<double> dynobench_state;

    if (auto compound = state_space->as<ob::CompoundStateSpace>()) {
        // Handle compound state space
        std::cout << "[DBRRTSolver] omplStateToDynobench: compound state space with " << compound->getSubspaceCount() << " subspaces" << std::endl;
        std::cout << "[DBRRTSolver] omplStateToDynobench: casting state to CompoundState..." << std::endl;
        const auto* compound_state = state->as<ob::CompoundState>();
        if (!compound_state) {
            std::cerr << "[DBRRTSolver] omplStateToDynobench: ERROR - failed to cast to CompoundState!" << std::endl;
            return dynobench_state;
        }
        std::cout << "[DBRRTSolver] omplStateToDynobench: compound_state ok" << std::endl;

        for (size_t i = 0; i < compound->getSubspaceCount(); ++i) {
            std::cout << "[DBRRTSolver] omplStateToDynobench: processing subspace " << i << std::endl;
            auto subspace = compound->getSubspace(i);
            std::cout << "[DBRRTSolver] omplStateToDynobench: subspace type = " << subspace->getName() << std::endl;
            std::cout << "[DBRRTSolver] omplStateToDynobench: getting substate..." << std::endl;
            const auto* substate = compound_state->as<ob::State>(i);

            if (!substate) {
                std::cerr << "[DBRRTSolver] Error: null substate at index " << i << std::endl;
                continue;
            }
            std::cout << "[DBRRTSolver] omplStateToDynobench: substate ok" << std::endl;

            // Check SO2 first since it may be a subclass of RealVectorStateSpace in OMPL
            if (subspace->getType() == ob::STATE_SPACE_SO2) {
                std::cout << "[DBRRTSolver] omplStateToDynobench: SO2StateSpace (by type)" << std::endl;
                const auto* so2_state = substate->as<ob::SO2StateSpace::StateType>();
                if (!so2_state) {
                    std::cerr << "[DBRRTSolver] Error: failed to cast substate to SO2StateSpace" << std::endl;
                    continue;
                }
                dynobench_state.push_back(so2_state->value);
            } else if (subspace->getType() == ob::STATE_SPACE_REAL_VECTOR) {
                auto rv_space = subspace->as<ob::RealVectorStateSpace>();
                std::cout << "[DBRRTSolver] omplStateToDynobench: RealVectorStateSpace dim = " << rv_space->getDimension() << std::endl;
                const auto* rv_state = substate->as<ob::RealVectorStateSpace::StateType>();
                if (!rv_state) {
                    std::cerr << "[DBRRTSolver] Error: failed to cast substate to RealVectorStateSpace" << std::endl;
                    continue;
                }
                for (size_t j = 0; j < rv_space->getDimension(); ++j) {
                    dynobench_state.push_back(rv_state->values[j]);
                }
            } else {
                std::cerr << "[DBRRTSolver] Warning: unknown subspace type at index " << i << std::endl;
            }
        }
    } else if (auto rv_space = state_space->as<ob::RealVectorStateSpace>()) {
        // Handle simple real vector state space
        const auto* rv_state = state->as<ob::RealVectorStateSpace::StateType>();
        if (!rv_state) {
            std::cerr << "[DBRRTSolver] Error: failed to cast state to RealVectorStateSpace" << std::endl;
            return dynobench_state;
        }
        for (size_t i = 0; i < rv_space->getDimension(); ++i) {
            dynobench_state.push_back(rv_state->values[i]);
        }
    } else {
        std::cerr << "[DBRRTSolver] Error: unsupported state space type" << std::endl;
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

    std::cout << "[DBRRTSolver] createDynobenchProblem: creating problem..." << std::endl;
    auto problem = std::make_shared<dynobench::Problem>();

    // Set robot type (both singular and plural forms for compatibility)
    std::cout << "[DBRRTSolver] createDynobenchProblem: getting robot type..." << std::endl;
    problem->robotType = getRobotType(robot);
    problem->robotTypes.push_back(problem->robotType);  // Required for trajectory_optimization
    std::cout << "[DBRRTSolver] createDynobenchProblem: robot type = " << problem->robotType << std::endl;

    // Set models base path
    problem->models_base_path = db_config_.models_base_path;

    // Convert start and goal states
    std::cout << "[DBRRTSolver] createDynobenchProblem: converting start state..." << std::endl;
    auto start_vec = omplStateToDynobench(start_state, robot);
    std::cout << "[DBRRTSolver] createDynobenchProblem: start_vec size = " << start_vec.size() << std::endl;

    std::cout << "[DBRRTSolver] createDynobenchProblem: converting goal state..." << std::endl;
    auto goal_vec = omplStateToDynobench(goal_state, robot);
    std::cout << "[DBRRTSolver] createDynobenchProblem: goal_vec size = " << goal_vec.size() << std::endl;

    problem->start = Eigen::Map<Eigen::VectorXd>(start_vec.data(), start_vec.size());
    problem->goal = Eigen::Map<Eigen::VectorXd>(goal_vec.data(), goal_vec.size());

    // Get position bounds from robot's state space
    // For unicycle (SE2), this is just the x,y position bounds, not theta
    std::cout << "[DBRRTSolver] createDynobenchProblem: getting bounds..." << std::endl;
    std::vector<double> min_bounds, max_bounds;
    getPositionBounds(robot, min_bounds, max_bounds);
    std::cout << "[DBRRTSolver] createDynobenchProblem: position bounds size = " << min_bounds.size() << std::endl;

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
    std::cout << "[DBRRTSolver] createDynobenchProblem: added " << problem->obstacles.size() << " obstacles" << std::endl;

    std::cout << "[DBRRTSolver] createDynobenchProblem: done" << std::endl;

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
            true    // compute_col
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

    // TODO: Implement region-guided sampling
    // This should bias samples towards regions in the path

    return [](Eigen::Ref<Eigen::VectorXd> sample) {
        // Default: uniform random sampling
        // Override with region-biased sampling
    };
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

    return config;
}

} // namespace mr_syclop
