/**
 * @file composite_dbrrt.cpp
 * @brief Implementation of Composite DB-RRT Planner for Multi-Robot Motion Planning
 */

// Pinocchio/Crocoddyl must be included before Boost/OMPL headers
#include <dynoplan/optimization/ocp.hpp>

#include "composite_dbrrt.h"

#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <cmath>

#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>

#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

#include "dynoplan/nigh_custom_spaces.hpp"
#include "dynobench/general_utils.hpp"

namespace po = boost::program_options;

// =============================================================================
// CompositeDBRRTTree Implementation
// =============================================================================

CompositeDBRRTTree::CompositeDBRRTTree(const std::vector<DBRRTNode*>& root_nodes) {
    root_ = std::make_unique<CompositeDBRRTVertex>(root_nodes);
    vertex_map_[root_nodes] = root_.get();
}

void CompositeDBRRTTree::addVertex(
    const std::vector<DBRRTNode*>& nodes,
    CompositeDBRRTVertex* parent)
{
    auto new_vertex = std::make_unique<CompositeDBRRTVertex>(nodes);
    new_vertex->parent = parent;

    if (parent) {
        // Compute cost as sum of individual node costs
        double cost = 0.0;
        for (auto* node : nodes) {
            cost += node->cost;
        }
        new_vertex->g_score = cost;
    }

    vertex_map_[nodes] = new_vertex.get();
    vertices_.push_back(std::move(new_vertex));
}

bool CompositeDBRRTTree::contains(const std::vector<DBRRTNode*>& nodes) const {
    return vertex_map_.find(nodes) != vertex_map_.end();
}

CompositeDBRRTVertex* CompositeDBRRTTree::getVertex(
    const std::vector<DBRRTNode*>& nodes) const
{
    auto it = vertex_map_.find(nodes);
    if (it != vertex_map_.end()) {
        return it->second;
    }
    return nullptr;
}

// =============================================================================
// IndividualDBRRT Implementation
// =============================================================================

IndividualDBRRT::IndividualDBRRT(
    std::shared_ptr<Robot> robot,
    std::shared_ptr<dynobench::Model_robot> dynobench_robot,
    const Config& config)
    : robot_(robot)
    , dynobench_robot_(dynobench_robot)
    , config_(config)
    , root_(nullptr)
{
    // Initialize RNG
    if (config_.seed >= 0) {
        rng_ = std::mt19937(config_.seed);
    } else {
        rng_ = std::mt19937(std::random_device{}());
    }

    // Allocate working memory
    offset_.resize(dynobench_robot_->get_offset_dim());
    aux_last_state_.resize(dynobench_robot_->nx);
}

void IndividualDBRRT::initialize(const Eigen::VectorXd& start_state) {
    // Create root node
    auto root_node = std::make_unique<DBRRTNode>();
    root_node->state = start_state;
    root_node->parent = nullptr;
    root_node->used_motion = -1;
    root_node->cost = 0.0;
    root_node->id = DBRRTNode::getNextId();

    root_ = root_node.get();
    nodes_.push_back(std::move(root_node));

    // Initialize NN structure for tree
    tree_nn_ = std::make_unique<ompl::NearestNeighborsGNATNoThreadSafety<DBRRTNode*>>();
    tree_nn_->setDistanceFunction([this](const DBRRTNode* a, const DBRRTNode* b) {
        return dynobench_robot_->distance(a->state, b->state);
    });
    tree_nn_->add(root_);
}

bool IndividualDBRRT::loadMotions(const std::string& motions_file) {
    try {
        dynoplan::load_motion_primitives_new(
            motions_file,
            *dynobench_robot_,
            motions_,
            config_.max_motions,
            false,  // cut_actions
            false,  // shuffle
            true    // compute_col
        );

        if (motions_.empty()) {
            std::cerr << "[IndividualDBRRT] No motions loaded from " << motions_file << std::endl;
            return false;
        }

        // Build NN structure for motion primitives
        motion_nn_.reset(dynoplan::nigh_factory2<dynoplan::Motion*>(
            dynobench_robot_->name, dynobench_robot_));

        for (auto& motion : motions_) {
            motion_nn_->add(&motion);
        }

        // Create expander
        expander_ = std::make_unique<dynoplan::Expander>(
            dynobench_robot_.get(),
            motion_nn_.get(),
            config_.delta);

        if (config_.seed >= 0) {
            expander_->seed(config_.seed);
        }

        // Allocate trajectory wrapper with max motion size
        size_t max_traj_size = 0;
        for (const auto& m : motions_) {
            max_traj_size = std::max(max_traj_size, m.traj.states.size());
        }
        traj_wrapper_.allocate_size(max_traj_size, dynobench_robot_->nx, dynobench_robot_->nu);

        std::cout << "[IndividualDBRRT] Loaded " << motions_.size()
                  << " motion primitives" << std::endl;

        return true;

    } catch (const std::exception& e) {
        std::cerr << "[IndividualDBRRT] Failed to load motions: " << e.what() << std::endl;
        return false;
    }
}

DBRRTNode* IndividualDBRRT::addNode(
    const Eigen::VectorXd& state,
    DBRRTNode* parent,
    int motion_idx)
{
    auto new_node = std::make_unique<DBRRTNode>();
    new_node->state = state;
    new_node->parent = parent;
    new_node->used_motion = motion_idx;
    new_node->id = DBRRTNode::getNextId();

    // Compute cost
    if (parent && motion_idx >= 0 && motion_idx < static_cast<int>(motions_.size())) {
        new_node->cost = parent->cost + motions_[motion_idx].cost;
    } else if (parent) {
        new_node->cost = parent->cost + dynobench_robot_->distance(parent->state, state);
    }

    DBRRTNode* node_ptr = new_node.get();
    nodes_.push_back(std::move(new_node));
    tree_nn_->add(node_ptr);

    return node_ptr;
}

DBRRTNode* IndividualDBRRT::nearest(const Eigen::VectorXd& state) {
    // Create temporary node for query
    DBRRTNode query_node;
    query_node.state = state;
    return tree_nn_->nearest(&query_node);
}

std::vector<dynoplan::LazyTraj> IndividualDBRRT::getApplicableMotions(DBRRTNode* from_node) {
    std::vector<dynoplan::LazyTraj> lazy_trajs;
    expander_->expand_lazy(from_node->state, lazy_trajs);
    return lazy_trajs;
}

bool IndividualDBRRT::validateMotion(
    dynoplan::LazyTraj& lazy_traj,
    Eigen::VectorXd& end_state)
{
    traj_wrapper_.set_size(lazy_traj.motion->traj.states.size());

    dynoplan::Time_benchmark time_bench;  // Dummy for timing

    bool valid = dynoplan::check_lazy_trajectory(
        lazy_traj,
        *dynobench_robot_,
        time_bench,
        traj_wrapper_,
        aux_last_state_,
        nullptr,  // check_state
        nullptr,  // num_valid_states
        true      // forward
    );

    if (valid) {
        end_state = traj_wrapper_.get_state(traj_wrapper_.get_size() - 1);
    }

    return valid;
}

DBRRTNode* IndividualDBRRT::expandToward(
    DBRRTNode* near_node,
    const Eigen::VectorXd& target)
{
    // Get applicable motions from this node
    std::vector<dynoplan::LazyTraj> lazy_trajs = getApplicableMotions(near_node);

    if (lazy_trajs.empty()) {
        return nullptr;
    }

    // Find best motion toward target
    double min_distance = std::numeric_limits<double>::max();
    int best_motion_idx = -1;
    Eigen::VectorXd best_end_state;

    for (auto& lazy_traj : lazy_trajs) {
        Eigen::VectorXd end_state;

        // Validate motion (obstacle collision check happens here)
        if (!validateMotion(lazy_traj, end_state)) {
            continue;
        }

        double dist = dynobench_robot_->distance(end_state, target);

        if (dist < min_distance) {
            min_distance = dist;
            best_motion_idx = lazy_traj.motion->idx;
            best_end_state = end_state;
        }
    }

    if (best_motion_idx < 0) {
        return nullptr;  // All motions were invalid
    }

    // Check if new node is sufficiently different from existing nodes
    DBRRTNode* existing = nearest(best_end_state);
    if (existing &&
        dynobench_robot_->distance(existing->state, best_end_state) < config_.delta / 2.0) {
        return nullptr;  // Too close to existing node
    }

    // Add new node to tree
    return addNode(best_end_state, near_node, best_motion_idx);
}

bool IndividualDBRRT::isAtGoal(
    const Eigen::VectorXd& state,
    const Eigen::VectorXd& goal) const
{
    return dynobench_robot_->distance(state, goal) < config_.goal_region;
}

// =============================================================================
// CompositeDBRRTPlanner Implementation
// =============================================================================

CompositeDBRRTPlanner::CompositeDBRRTPlanner(const CompositeDBRRTConfig& config)
    : config_(config)
    , position_bounds_(2)
{
    // Initialize RNG
    if (config_.seed >= 0) {
        rng_ = std::mt19937(config_.seed);
    } else {
        rng_ = std::mt19937(std::random_device{}());
    }
}

CompositeDBRRTPlanner::~CompositeDBRRTPlanner() {
    cleanup();
}

void CompositeDBRRTPlanner::cleanup() {
    individual_trees_.clear();
    composite_tree_.reset();
    robots_.clear();
    dynobench_robots_.clear();
    env_collision_manager_.reset();
    start_states_.clear();
    goal_states_.clear();
    num_expansions_ = 0;
    solution_found_ = false;
    goal_vertex_ = nullptr;
}

void CompositeDBRRTPlanner::setupEnvironment(const CompositeDBRRTProblem& problem) {
    // Set up position bounds
    position_bounds_.setLow(0, problem.env_min[0]);
    position_bounds_.setLow(1, problem.env_min[1]);
    position_bounds_.setHigh(0, problem.env_max[0]);
    position_bounds_.setHigh(1, problem.env_max[1]);

    // Create FCL collision manager
    env_collision_manager_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();

    for (auto* obstacle : problem.obstacles) {
        env_collision_manager_->registerObject(obstacle);
    }

    if (!problem.obstacles.empty()) {
        env_collision_manager_->setup();
    }
}

void CompositeDBRRTPlanner::setupRobots(const CompositeDBRRTProblem& problem) {
    for (const auto& robot_spec : problem.robots) {
        // Create OMPL robot
        auto robot = create_robot(robot_spec.type, position_bounds_);
        robots_.push_back(robot);

        // Create dynobench robot model
        std::string model_file = config_.models_base_path + "/" + robot_spec.type + ".yaml";

        Eigen::VectorXd p_lb(2), p_ub(2);
        p_lb << problem.env_min[0], problem.env_min[1];
        p_ub << problem.env_max[0], problem.env_max[1];

        auto dynobench_robot = dynobench::robot_factory(
            model_file.c_str(), p_lb, p_ub);

        if (!dynobench_robot) {
            throw std::runtime_error("Failed to create dynobench robot: " + robot_spec.type);
        }

        // Load environment into dynobench robot
        dynobench::Problem dyn_problem;
        dyn_problem.obstacles = problem.dynobench_obstacles;
        dynobench::load_env(*dynobench_robot, dyn_problem);

        dynobench_robots_.push_back(std::move(dynobench_robot));

        // Store start and goal states
        Eigen::VectorXd start = Eigen::Map<const Eigen::VectorXd>(
            robot_spec.start.data(), robot_spec.start.size());
        Eigen::VectorXd goal = Eigen::Map<const Eigen::VectorXd>(
            robot_spec.goal.data(), robot_spec.goal.size());

        start_states_.push_back(start);
        goal_states_.push_back(goal);

        if (config_.debug) {
            std::cout << "[CompositeDBRRT] Robot " << robots_.size() - 1
                      << " (" << robot_spec.type << ")" << std::endl;
            std::cout << "  Start: " << start.transpose() << std::endl;
            std::cout << "  Goal:  " << goal.transpose() << std::endl;
        }
    }
}

void CompositeDBRRTPlanner::setupIndividualTrees(const CompositeDBRRTProblem& problem) {
    for (size_t i = 0; i < robots_.size(); ++i) {
        // Create individual tree configuration
        IndividualDBRRT::Config tree_config;
        tree_config.delta = config_.delta;
        tree_config.goal_region = config_.goal_region;
        tree_config.max_motions = config_.max_motions;
        tree_config.seed = config_.seed >= 0 ? config_.seed + i : -1;

        // Create individual tree
        auto tree = std::make_unique<IndividualDBRRT>(
            robots_[i],
            dynobench_robots_[i],
            tree_config);

        // Load motion primitives
        std::string robot_type = problem.robots[i].type;
        std::string motions_file;

        if (config_.motion_files.count(robot_type)) {
            motions_file = config_.motion_files.at(robot_type);
        } else {
            // Default path
            motions_file = config_.models_base_path + "/../dynomotions/" +
                           robot_type + "_default.msgpack";
        }

        if (!tree->loadMotions(motions_file)) {
            throw std::runtime_error("Failed to load motions for robot " + std::to_string(i));
        }

        // Initialize tree with start state
        tree->initialize(start_states_[i]);

        individual_trees_.push_back(std::move(tree));
    }
}

void CompositeDBRRTPlanner::setupCompositeTree() {
    // Create composite tree root from individual tree roots
    std::vector<DBRRTNode*> root_nodes;
    for (auto& tree : individual_trees_) {
        root_nodes.push_back(tree->getRoot());
    }
    composite_tree_ = std::make_unique<CompositeDBRRTTree>(root_nodes);
}

CompositeDBRRTResult CompositeDBRRTPlanner::plan(const CompositeDBRRTProblem& problem) {
    // Clean up any previous planning state
    cleanup();

    CompositeDBRRTResult result;
    result.solved = false;
    result.planning_time = 0.0;

    try {
        if (config_.debug) {
            std::cout << "[CompositeDBRRT] Starting planning..." << std::endl;
            std::cout << "  Robots: " << problem.robots.size() << std::endl;
            std::cout << "  Time limit: " << config_.time_limit << "s" << std::endl;
        }

        // Setup
        setupEnvironment(problem);
        setupRobots(problem);
        setupIndividualTrees(problem);
        setupCompositeTree();

        // Solve
        auto start_time = std::chrono::steady_clock::now();
        bool solved = solve(config_.time_limit);
        auto end_time = std::chrono::steady_clock::now();

        result.planning_time = std::chrono::duration<double>(end_time - start_time).count();
        result.solved = solved;

        if (solved && goal_vertex_) {
            result.trajectories = extractSolution(goal_vertex_);
        }

        if (config_.debug) {
            std::cout << "[CompositeDBRRT] Planning finished" << std::endl;
            std::cout << "  Solved: " << (solved ? "yes" : "no") << std::endl;
            std::cout << "  Time: " << result.planning_time << "s" << std::endl;
            std::cout << "  Expansions: " << num_expansions_ << std::endl;
            std::cout << "  Composite tree size: " << composite_tree_->size() << std::endl;
            for (size_t i = 0; i < individual_trees_.size(); ++i) {
                std::cout << "  Robot " << i << " tree size: "
                          << individual_trees_[i]->size() << std::endl;
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "[CompositeDBRRT] Planning failed: " << e.what() << std::endl;
        result.solved = false;
    }

    return result;
}

bool CompositeDBRRTPlanner::solve(double max_time) {
    auto start_time = std::chrono::high_resolution_clock::now();
    solution_found_ = false;
    num_expansions_ = 0;

    // Check if start is already at goal
    if (checkGoalReached()) {
        solution_found_ = true;
        return true;
    }

    unsigned int iteration = 0;
    while (true) {
        auto current_time = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(current_time - start_time).count();

        if (elapsed > max_time) {
            if (config_.debug) {
                std::cout << "[CompositeDBRRT] Time limit reached" << std::endl;
            }
            break;
        }

        // Expansion phase
        for (unsigned int i = 0; i < config_.expansions_per_iter; ++i) {
            expandCompositeTree();
        }

        // Check if goal reached
        if (checkGoalReached()) {
            solution_found_ = true;
            if (config_.debug) {
                std::cout << "[CompositeDBRRT] Solution found!" << std::endl;
                std::cout << "  Iterations: " << iteration << std::endl;
                std::cout << "  Time: " << elapsed << "s" << std::endl;
            }
            return true;
        }

        iteration++;

        // Progress update
        if (config_.debug && iteration % 100 == 0) {
            std::cout << "[CompositeDBRRT] Iteration " << iteration
                      << ", tree size: " << composite_tree_->size()
                      << ", time: " << elapsed << "s" << std::endl;
        }
    }

    return false;
}

void CompositeDBRRTPlanner::expandCompositeTree() {
    num_expansions_++;

    // 1. Sample random configuration for all robots
    std::vector<Eigen::VectorXd> q_rand = sampleRandomConfiguration();

    // 2. Find nearest composite vertex in tree
    CompositeDBRRTVertex* v_near = nearestCompositeVertex(q_rand);

    // 3. Tensor product expansion
    std::vector<DBRRTNode*> new_nodes = tensorProductExpansion(v_near, q_rand);

    // 4. Check movement requirement
    bool any_moved = false;
    bool all_moved = true;
    for (size_t i = 0; i < new_nodes.size(); ++i) {
        if (new_nodes[i] != v_near->robot_nodes[i]) {
            any_moved = true;
        } else {
            all_moved = false;
        }
    }

    if (config_.require_all_move) {
        if (!all_moved) return;
    } else {
        if (!any_moved) return;
    }

    // 5. Check if new composite vertex
    if (composite_tree_->contains(new_nodes)) {
        return;
    }

    // 6. Validate robot-robot collisions
    if (!isCompositeMotionValid(v_near->robot_nodes, new_nodes)) {
        return;
    }

    // 7. Add new composite vertex to tree
    composite_tree_->addVertex(new_nodes, v_near);
}

std::vector<Eigen::VectorXd> CompositeDBRRTPlanner::sampleRandomConfiguration() {
    std::vector<Eigen::VectorXd> config;
    config.reserve(individual_trees_.size());

    std::uniform_real_distribution<> dis(0.0, 1.0);

    for (size_t i = 0; i < individual_trees_.size(); ++i) {
        Eigen::VectorXd state(dynobench_robots_[i]->nx);

        // Goal bias
        if (dis(rng_) < config_.goal_bias) {
            state = goal_states_[i];
        } else {
            // Uniform random sampling
            dynobench_robots_[i]->sample_uniform(state);
        }

        config.push_back(state);
    }

    return config;
}

CompositeDBRRTVertex* CompositeDBRRTPlanner::nearestCompositeVertex(
    const std::vector<Eigen::VectorXd>& config)
{
    double min_dist = std::numeric_limits<double>::infinity();
    CompositeDBRRTVertex* nearest = composite_tree_->getRoot();

    // Check root
    double dist = compositeDistance(composite_tree_->getRoot()->robot_nodes, config);
    if (dist < min_dist) {
        min_dist = dist;
        nearest = composite_tree_->getRoot();
    }

    // Check all other vertices
    for (const auto& vertex_ptr : composite_tree_->getVertices()) {
        dist = compositeDistance(vertex_ptr->robot_nodes, config);
        if (dist < min_dist) {
            min_dist = dist;
            nearest = vertex_ptr.get();
        }
    }

    return nearest;
}

std::vector<DBRRTNode*> CompositeDBRRTPlanner::tensorProductExpansion(
    CompositeDBRRTVertex* v_near,
    const std::vector<Eigen::VectorXd>& q_rand)
{
    std::vector<DBRRTNode*> new_nodes;
    new_nodes.reserve(individual_trees_.size());

    for (size_t i = 0; i < individual_trees_.size(); ++i) {
        DBRRTNode* current_node = v_near->robot_nodes[i];
        const Eigen::VectorXd& target = q_rand[i];

        // Try to expand individual tree toward target using motion primitives
        DBRRTNode* expanded_node = individual_trees_[i]->expandToward(current_node, target);

        if (expanded_node && expanded_node != current_node) {
            // Expansion succeeded - new node added to individual tree
            new_nodes.push_back(expanded_node);
        } else {
            // No valid expansion - keep current node
            new_nodes.push_back(current_node);
        }
    }

    return new_nodes;
}

bool CompositeDBRRTPlanner::isCompositeMotionValid(
    const std::vector<DBRRTNode*>& from_nodes,
    const std::vector<DBRRTNode*>& to_nodes)
{
    size_t num_robots = from_nodes.size();

    // Check final state for robot-robot collisions
    std::vector<Eigen::VectorXd> to_states;
    to_states.reserve(num_robots);
    for (size_t i = 0; i < num_robots; ++i) {
        to_states.push_back(to_nodes[i]->state);
    }

    if (!isRobotRobotCollisionFree(to_states)) {
        return false;
    }

    // Check motion for robot-robot collisions (interpolated)
    std::vector<Eigen::VectorXd> from_states;
    from_states.reserve(num_robots);
    for (size_t i = 0; i < num_robots; ++i) {
        from_states.push_back(from_nodes[i]->state);
    }

    return isRobotRobotMotionValid(from_states, to_states);
}

bool CompositeDBRRTPlanner::isRobotRobotCollisionFree(
    const std::vector<Eigen::VectorXd>& states) const
{
    for (size_t i = 0; i < robots_.size(); ++i) {
        for (size_t j = i + 1; j < robots_.size(); ++j) {
            // Get transforms for all parts of both robots
            for (size_t part_i = 0; part_i < robots_[i]->numParts(); ++part_i) {
                // Create temporary OMPL state for robot i
                auto si_i = robots_[i]->getSpaceInformation();
                auto state_i = si_i->allocState();
                auto* se2_i = state_i->as<ob::SE2StateSpace::StateType>();
                se2_i->setX(states[i](0));
                se2_i->setY(states[i](1));
                se2_i->setYaw(states[i].size() > 2 ? states[i](2) : 0.0);

                const auto& transform_i = robots_[i]->getTransform(state_i, part_i);
                fcl::CollisionObjectf co_i(robots_[i]->getCollisionGeometry(part_i));
                co_i.setTranslation(transform_i.translation());
                co_i.setRotation(transform_i.rotation());
                co_i.computeAABB();

                for (size_t part_j = 0; part_j < robots_[j]->numParts(); ++part_j) {
                    // Create temporary OMPL state for robot j
                    auto si_j = robots_[j]->getSpaceInformation();
                    auto state_j = si_j->allocState();
                    auto* se2_j = state_j->as<ob::SE2StateSpace::StateType>();
                    se2_j->setX(states[j](0));
                    se2_j->setY(states[j](1));
                    se2_j->setYaw(states[j].size() > 2 ? states[j](2) : 0.0);

                    const auto& transform_j = robots_[j]->getTransform(state_j, part_j);
                    fcl::CollisionObjectf co_j(robots_[j]->getCollisionGeometry(part_j));
                    co_j.setTranslation(transform_j.translation());
                    co_j.setRotation(transform_j.rotation());
                    co_j.computeAABB();

                    fcl::CollisionRequestf request;
                    request.num_max_contacts = 1;
                    fcl::CollisionResultf result;
                    fcl::collide(&co_i, &co_j, request, result);

                    si_j->freeState(state_j);

                    if (result.isCollision()) {
                        si_i->freeState(state_i);
                        return false;
                    }
                }

                si_i->freeState(state_i);
            }
        }
    }

    return true;
}

bool CompositeDBRRTPlanner::isRobotRobotMotionValid(
    const std::vector<Eigen::VectorXd>& from_states,
    const std::vector<Eigen::VectorXd>& to_states) const
{
    size_t num_robots = from_states.size();

    // Determine number of interpolation steps based on max distance
    int steps = 1;
    for (size_t i = 0; i < num_robots; ++i) {
        double dist = dynobench_robots_[i]->distance(from_states[i], to_states[i]);
        int robot_steps = static_cast<int>(std::ceil(dist / 0.1));  // 0.1 step resolution
        steps = std::max(steps, robot_steps);
    }

    // Check intermediate states
    for (int s = 1; s <= steps; ++s) {
        double t = static_cast<double>(s) / static_cast<double>(steps);

        std::vector<Eigen::VectorXd> interp_states;
        interp_states.reserve(num_robots);

        for (size_t i = 0; i < num_robots; ++i) {
            Eigen::VectorXd state = (1.0 - t) * from_states[i] + t * to_states[i];
            // Normalize angle if needed (assuming state[2] is angle for unicycle)
            if (state.size() > 2) {
                while (state(2) > M_PI) state(2) -= 2 * M_PI;
                while (state(2) < -M_PI) state(2) += 2 * M_PI;
            }
            interp_states.push_back(state);
        }

        if (!isRobotRobotCollisionFree(interp_states)) {
            return false;
        }
    }

    return true;
}

bool CompositeDBRRTPlanner::checkGoalReached() {
    // Check all vertices in composite tree
    auto checkVertex = [this](CompositeDBRRTVertex* vertex) -> bool {
        for (size_t i = 0; i < individual_trees_.size(); ++i) {
            if (!individual_trees_[i]->isAtGoal(
                    vertex->robot_nodes[i]->state, goal_states_[i])) {
                return false;
            }
        }
        return true;
    };

    // Check root
    if (checkVertex(composite_tree_->getRoot())) {
        goal_vertex_ = composite_tree_->getRoot();
        return true;
    }

    // Check all other vertices
    for (const auto& vertex_ptr : composite_tree_->getVertices()) {
        if (checkVertex(vertex_ptr.get())) {
            goal_vertex_ = vertex_ptr.get();
            return true;
        }
    }

    return false;
}

double CompositeDBRRTPlanner::compositeDistance(
    const std::vector<DBRRTNode*>& nodes,
    const std::vector<Eigen::VectorXd>& config)
{
    double total_dist_sq = 0.0;

    for (size_t i = 0; i < nodes.size(); ++i) {
        double d = dynobench_robots_[i]->distance(nodes[i]->state, config[i]);
        total_dist_sq += d * d;
    }

    return std::sqrt(total_dist_sq);
}

std::vector<dynobench::Trajectory> CompositeDBRRTPlanner::extractSolution(
    CompositeDBRRTVertex* goal_vertex)
{
    std::vector<dynobench::Trajectory> trajectories(individual_trees_.size());

    // Build path from goal to root
    std::vector<CompositeDBRRTVertex*> path;
    CompositeDBRRTVertex* current = goal_vertex;
    while (current != nullptr) {
        path.push_back(current);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());

    // For each robot, extract the trajectory
    for (size_t r = 0; r < individual_trees_.size(); ++r) {
        dynobench::Trajectory& traj = trajectories[r];

        for (size_t p = 0; p < path.size(); ++p) {
            DBRRTNode* node = path[p]->robot_nodes[r];

            // Add state
            traj.states.push_back(node->state);

            // Add actions from motion primitive (if available and not root)
            if (p > 0 && node->used_motion >= 0) {
                const auto& motion = individual_trees_[r]->getMotions()[node->used_motion];
                for (const auto& action : motion.traj.actions) {
                    traj.actions.push_back(action);
                }
            }
        }
    }

    return trajectories;
}

// =============================================================================
// Configuration Loading
// =============================================================================

CompositeDBRRTConfig loadCompositeDBRRTConfig(const YAML::Node& node) {
    CompositeDBRRTConfig config;

    if (node["time_limit"]) {
        config.time_limit = node["time_limit"].as<double>();
    }
    if (node["delta"]) {
        config.delta = node["delta"].as<double>();
    }
    if (node["goal_region"]) {
        config.goal_region = node["goal_region"].as<double>();
    }
    if (node["max_motions"]) {
        config.max_motions = node["max_motions"].as<size_t>();
    }
    if (node["expansions_per_iter"]) {
        config.expansions_per_iter = node["expansions_per_iter"].as<unsigned int>();
    }
    if (node["goal_threshold"]) {
        config.goal_threshold = node["goal_threshold"].as<double>();
    }
    if (node["require_all_move"]) {
        config.require_all_move = node["require_all_move"].as<bool>();
    }
    if (node["goal_bias"]) {
        config.goal_bias = node["goal_bias"].as<double>();
    }
    if (node["models_base_path"]) {
        config.models_base_path = node["models_base_path"].as<std::string>();
    }
    if (node["seed"]) {
        config.seed = node["seed"].as<int>();
    }
    if (node["debug"]) {
        config.debug = node["debug"].as<bool>();
    }

    // Load motion files map
    if (node["motion_files"]) {
        for (const auto& item : node["motion_files"]) {
            std::string robot_type = item.first.as<std::string>();
            std::string file_path = item.second.as<std::string>();
            config.motion_files[robot_type] = file_path;
        }
    }

    return config;
}
