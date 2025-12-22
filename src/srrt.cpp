#include "srrt.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/tools/config/SelfConfig.h>
#include "../db-CBS/src/robots.h"

namespace {

bool srrtDebugEnabled()
{
    const char* env = std::getenv("SRRT_DEBUG");
    if (!env) {
        return false;
    }
    return std::string(env) != "0";
}

int srrtDebugLevel()
{
    const char* env = std::getenv("SRRT_DEBUG_LEVEL");
    if (!env) {
        return 1;
    }
    char* end = nullptr;
    long level = std::strtol(env, &end, 10);
    if (end == env || level < 0) {
        return 1;
    }
    return static_cast<int>(level);
}

int srrtDebugEvery()
{
    const char* env = std::getenv("SRRT_DEBUG_EVERY");
    if (!env) {
        return 200;
    }
    char* end = nullptr;
    long every = std::strtol(env, &end, 10);
    if (end == env || every <= 0) {
        return 200;
    }
    return static_cast<int>(every);
}

int srrtDebugPolicyMax()
{
    const char* env = std::getenv("SRRT_DEBUG_POLICY_MAX");
    if (!env) {
        return 200;
    }
    char* end = nullptr;
    long max_nodes = std::strtol(env, &end, 10);
    if (end == env) {
        return 200;
    }
    return static_cast<int>(max_nodes);
}

void srrtDebugLog(const std::string& tag, const std::string& msg)
{
    if (!srrtDebugEnabled()) {
        return;
    }
    std::cout << "[sRRT][" << tag << "] " << msg << std::endl;
}

std::string formatSe2State(const ob::State* state)
{
    const auto* se2 = state->as<ob::SE2StateSpace::StateType>();
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "(" << se2->getX() << ", " << se2->getY() << ", " << se2->getYaw() << ")";
    return oss.str();
}

std::string formatCompositeState(const ob::State* state, int num_robots)
{
    const auto* compound = state->as<ob::CompoundStateSpace::StateType>();
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < num_robots; ++i) {
        const auto* se2 = compound->components[i]->as<ob::SE2StateSpace::StateType>();
        oss << std::fixed << std::setprecision(3)
            << "(" << se2->getX() << ", " << se2->getY() << ", " << se2->getYaw() << ")";
        if (i + 1 < num_robots) {
            oss << " ";
        }
    }
    oss << "]";
    return oss.str();
}

std::string formatCollisionSet(const std::set<int>& collisions)
{
    std::ostringstream oss;
    oss << "{";
    for (auto it = collisions.begin(); it != collisions.end(); ++it) {
        if (it != collisions.begin()) {
            oss << ", ";
        }
        oss << *it;
    }
    oss << "}";
    return oss.str();
}

std::string formatDistances(const std::vector<double>& distances)
{
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < distances.size(); ++i) {
        if (i > 0) {
            oss << ", ";
        }
        if (std::isfinite(distances[i])) {
            oss << std::fixed << std::setprecision(3) << distances[i];
        } else {
            oss << "inf";
        }
    }
    oss << "]";
    return oss.str();
}

struct DebugCounters {
    size_t iterations = 0;
    size_t samples = 0;
    size_t motions_added = 0;
    size_t obstacle_rejects = 0;
    size_t collision_events = 0;
    size_t policy_nulls = 0;
    size_t policy_used = 0;
    size_t policy_zero_steps = 0;
    size_t sample_used = 0;
    size_t zero_extensions = 0;
    size_t max_collision_set = 0;
    std::vector<double> best_goal_dist;
};

DebugCounters& debugCounters()
{
    static DebugCounters counters;
    return counters;
}

struct ProjectionDebugInfo {
    std::vector<std::string> choices;
};

ProjectionDebugInfo& projectionDebugInfo()
{
    static ProjectionDebugInfo info;
    return info;
}

void resetDebugCounters(int num_robots)
{
    DebugCounters fresh;
    fresh.best_goal_dist.assign(num_robots, std::numeric_limits<double>::infinity());
    debugCounters() = std::move(fresh);
}

}  // namespace

// ============================================================================
// IndividualPolicyTree Implementation - Uses OMPL's RRT
// ============================================================================

IndividualPolicyTree::IndividualPolicyTree(const ob::SpaceInformationPtr& si, int robot_id)
    : si_(si), robot_id_(robot_id), goal_state_(nullptr)
{
    // Create RRT planner for this robot's individual space
    rrt_planner_ = std::make_shared<og::RRT>(si_);

    // Create problem definition
    pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
    rrt_planner_->setProblemDefinition(pdef_);

    // Initialize nearest neighbors structure for policy queries
    nn_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<PolicyNode*>(rrt_planner_.get()));
    nn_->setDistanceFunction([this](const PolicyNode* a, const PolicyNode* b) {
        return distanceFunction(a, b);
    });
}

IndividualPolicyTree::~IndividualPolicyTree()
{
    if (goal_state_)
        si_->freeState(goal_state_);

    // Free policy nodes
    for (auto* node : policy_nodes_) {
        if (node->state)
            si_->freeState(node->state);
        if (node->parent_state)
            si_->freeState(node->parent_state);
        delete node;
    }
}

void IndividualPolicyTree::buildTree(ob::State* start_state, ob::State* goal_state, double timeout)
{
    std::cout << "Building individual policy tree for robot " << robot_id_ << std::endl;

    if (srrtDebugEnabled()) {
        std::ostringstream oss;
        oss << "Robot " << robot_id_ << " start=" << formatSe2State(start_state)
            << " goal=" << formatSe2State(goal_state)
            << " dist=" << std::fixed << std::setprecision(3) << si_->distance(start_state, goal_state)
            << " start_valid=" << si_->isValid(start_state)
            << " goal_valid=" << si_->isValid(goal_state)
            << " timeout=" << timeout;
        srrtDebugLog("POLICY", oss.str());
    }

    // Store goal for later use
    goal_state_ = si_->allocState();
    si_->copyState(goal_state_, goal_state);

    // For backward RRT: goal becomes the "start" and start becomes the "goal"
    // This builds a tree from goal toward start
    pdef_->clearStartStates();
    pdef_->addStartState(goal_state_);

    // Set start state as the goal (backward planning)
    // Need to allocate and copy the start state
    ob::State* goal_for_backward = si_->allocState();
    si_->copyState(goal_for_backward, start_state);

    auto goal_region = std::make_shared<ob::GoalState>(si_);
    goal_region->setState(goal_for_backward);
    goal_region->setThreshold(0.5);  // Small threshold to reach near start
    pdef_->setGoal(goal_region);

    rrt_planner_->clear();
    rrt_planner_->setup();

    // Run RRT until we reach the start or timeout
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(timeout);
    ob::PlannerStatus status = rrt_planner_->solve(ptc);

    std::cout << "Robot " << robot_id_ << " policy tree: " << status << std::endl;

    if (srrtDebugEnabled()) {
        std::ostringstream oss;
        oss << "Robot " << robot_id_ << " policy status=" << status
            << " solutions=" << pdef_->getSolutionCount();
        srrtDebugLog("POLICY", oss.str());
    }

    // Extract tree structure for efficient policy queries
    extractTreeStructure();
}

void IndividualPolicyTree::extractTreeStructure()
{
    // Get planner data from RRT
    ob::PlannerData pdata(si_);
    rrt_planner_->getPlannerData(pdata);

    std::cout << "Extracting policy tree structure for robot " << robot_id_
              << " (" << pdata.numVertices() << " vertices)" << std::endl;

    // Build a map from state pointer to vertex index for parent lookup
    std::map<const ob::State*, unsigned int> state_to_index;
    for (unsigned int i = 0; i < pdata.numVertices(); ++i) {
        state_to_index[pdata.getVertex(i).getState()] = i;
    }

    // Extract all vertices and their parents
    for (unsigned int i = 0; i < pdata.numVertices(); ++i) {
        PolicyNode* node = new PolicyNode();

        // Copy current state
        node->state = si_->allocState();
        si_->copyState(node->state, pdata.getVertex(i).getState());

        // Get parent vertex indices
        std::vector<unsigned int> parent_indices;
        pdata.getIncomingEdges(i, parent_indices);

        // In a tree, each node has at most one parent
        if (!parent_indices.empty()) {
            // Copy parent state (toward goal in backward tree)
            node->parent_state = si_->allocState();
            si_->copyState(node->parent_state, pdata.getVertex(parent_indices[0]).getState());
        } else {
            // This is the root (goal state) - no parent
            node->parent_state = nullptr;
        }

        policy_nodes_.push_back(node);
        nn_->add(node);
    }

    std::cout << "Extracted " << policy_nodes_.size() << " policy nodes for robot "
              << robot_id_ << std::endl;

    if (srrtDebugEnabled() && policy_nodes_.empty()) {
        srrtDebugLog("POLICY", "Robot " + std::to_string(robot_id_) +
                               " policy tree is empty after extraction.");
    }

    if (srrtDebugEnabled() && srrtDebugLevel() >= 2) {
        const int max_nodes = srrtDebugPolicyMax();
        const size_t total = policy_nodes_.size();
        const size_t to_print = (max_nodes <= 0)
                                    ? total
                                    : std::min(total, static_cast<size_t>(max_nodes));
        srrtDebugLog("POLICY_TREE", "Robot " + std::to_string(robot_id_) +
                                        " nodes=" + std::to_string(total) +
                                        " printing=" + std::to_string(to_print));
        for (size_t i = 0; i < to_print; ++i) {
            const auto* node = policy_nodes_[i];
            std::ostringstream oss;
            oss << "node[" << i << "] state=" << formatSe2State(node->state);
            if (node->parent_state) {
                oss << " parent=" << formatSe2State(node->parent_state);
            } else {
                oss << " parent=<root>";
            }
            srrtDebugLog("POLICY_TREE", oss.str());
        }
        if (to_print < total) {
            srrtDebugLog("POLICY_TREE", "Truncated output. Set SRRT_DEBUG_POLICY_MAX=0 to print all nodes.");
        }
    }
}

ob::State* IndividualPolicyTree::getPolicy(ob::State* query_state)
{
    // Query the tree for nearest state to query_state
    if (policy_nodes_.empty()) {
        return nullptr;  // No tree built yet
    }

    // Create temporary node for query
    PolicyNode query_node;
    query_node.state = query_state;

    // Find nearest node in policy tree
    PolicyNode* nearest = nn_->nearest(&query_node);

    if (nearest == nullptr) {
        return nullptr;
    }

    // Return parent state (next step toward goal)
    // If no parent (at goal), return current state
    return nearest->parent_state ? nearest->parent_state : nearest->state;
}

double IndividualPolicyTree::distanceFunction(const PolicyNode* a, const PolicyNode* b) const
{
    return si_->distance(a->state, b->state);
}

// ============================================================================
// ForwardSearchTree Implementation - Custom tree with collision sets
// ============================================================================

ForwardSearchTree::ForwardSearchTree(const ob::SpaceInformationPtr& si, int num_robots)
    : si_(si), num_robots_(num_robots), root_(nullptr)
{
    // Initialize nearest neighbors structure
    // Use GNAT (Geometric Near-neighbor Access Tree) which works well for motion planning
    nn_ = std::make_shared<ompl::NearestNeighborsGNAT<ForwardMotion*>>();

    // Set distance function
    nn_->setDistanceFunction([this](const ForwardMotion* a, const ForwardMotion* b) {
        return distanceFunction(a, b);
    });
}

ForwardSearchTree::~ForwardSearchTree()
{
    // Free all motions
    for (auto* motion : motions_) {
        if (motion->state)
            si_->freeState(motion->state);
        delete motion;
    }
}

void ForwardSearchTree::initialize(ob::State* start_state)
{
    std::cout << "Initializing forward tree with start state" << std::endl;

    // Create root motion with empty collision set
    root_ = new ForwardMotion(si_);
    si_->copyState(root_->state, start_state);
    root_->parent = nullptr;
    root_->collision_set.clear();

    if (srrtDebugEnabled()) {
        srrtDebugLog("FORWARD", "Root state=" + formatCompositeState(root_->state, num_robots_));
    }

    // Add to tree
    motions_.push_back(root_);
    nn_->add(root_);
}

ForwardMotion* ForwardSearchTree::findNearest(ob::State* state)
{
    // Create temporary motion for query
    ForwardMotion query;
    query.state = state;

    // Use nearest neighbors structure
    return nn_->nearest(&query);
}

void ForwardSearchTree::addMotion(ForwardMotion* motion)
{
    motions_.push_back(motion);
    nn_->add(motion);
}

double ForwardSearchTree::distanceFunction(const ForwardMotion* a, const ForwardMotion* b) const
{
    return si_->distance(a->state, b->state);
}

// ============================================================================
// sRRTPlanner Implementation
// ============================================================================

sRRTPlanner::sRRTPlanner(const ob::SpaceInformationPtr& si,
                         int num_robots,
                         const std::vector<ob::SpaceInformationPtr>& individual_si,
                         const std::vector<std::shared_ptr<Robot>>& robots)
    : composite_si_(si), individual_si_(individual_si), num_robots_(num_robots),
      robots_(robots), max_distance_(0.5), goal_threshold_(0.1), collision_radius_(0.5)
{
    std::cout << "Initializing sRRT planner for " << num_robots << " robots" << std::endl;

    // Create composite sampler
    composite_sampler_ = composite_si_->allocStateSampler();
}

sRRTPlanner::~sRRTPlanner()
{
    // Cleanup handled by unique_ptrs and destructors
}

bool sRRTPlanner::solve(const std::vector<ob::State*>& start_states,
                        const std::vector<ob::State*>& goal_states,
                        double timeout)
{
    std::cout << "Starting sRRT planning..." << std::endl;

    if (srrtDebugEnabled()) {
        resetDebugCounters(num_robots_);
        std::ostringstream oss;
        oss << "Debug enabled level=" << srrtDebugLevel()
            << " report_every=" << srrtDebugEvery()
            << " max_distance=" << max_distance_
            << " goal_threshold=" << goal_threshold_
            << " timeout=" << timeout;
        srrtDebugLog("CONFIG", oss.str());
    }

    // Phase 1: Build individual policies (backward trees)
    buildIndividualPolicies(start_states, goal_states, timeout * 0.2);

    // Phase 2: Initialize forward tree
    initializeForwardTree(start_states);

    // Phase 3: Main search loop
    bool success = mainSearchLoop(goal_states, timeout * 0.8);

    return success;
}

std::vector<ob::State*> sRRTPlanner::getSolutionPath() const
{
    return solution_path_;
}

// ============================================================================
// Phase 1: Initialization
// ============================================================================

void sRRTPlanner::buildIndividualPolicies(const std::vector<ob::State*>& start_states,
                                           const std::vector<ob::State*>& goal_states,
                                           double timeout)
{
    std::cout << "Building individual policy trees..." << std::endl;

    individual_policies_.clear();

    // Build backward RRT for each robot using OMPL's RRT
    for (int i = 0; i < num_robots_; ++i) {
        auto policy = std::make_unique<IndividualPolicyTree>(individual_si_[i], i);
        policy->buildTree(start_states[i], goal_states[i], timeout / num_robots_);
        individual_policies_.push_back(std::move(policy));
    }
}

void sRRTPlanner::initializeForwardTree(const std::vector<ob::State*>& start_states)
{
    std::cout << "Initializing forward search tree..." << std::endl;

    // Combine individual start states into composite start
    ob::State* composite_start = combineIndividualStates(start_states);

    // Initialize forward tree with composite start (empty collision set)
    forward_tree_ = std::make_unique<ForwardSearchTree>(composite_si_, num_robots_);
    forward_tree_->initialize(composite_start);

    composite_si_->freeState(composite_start);
}

// ============================================================================
// Phase 2: Main Search Loop
// ============================================================================

bool sRRTPlanner::mainSearchLoop(const std::vector<ob::State*>& goal_states, double timeout)
{
    std::cout << "Running main search loop..." << std::endl;

    auto start_time = std::chrono::high_resolution_clock::now();
    int iterations = 0;
    const bool debug_enabled = srrtDebugEnabled();
    const int debug_level = srrtDebugLevel();
    const int debug_every = srrtDebugEvery();
    auto& counters = debugCounters();

    std::vector<ob::State*> debug_goal_states;
    if (debug_enabled) {
        debug_goal_states.resize(num_robots_, nullptr);
        for (int i = 0; i < num_robots_; ++i) {
            debug_goal_states[i] = individual_si_[i]->allocState();
        }
    }
    std::vector<double> current_goal_distances;
    if (debug_enabled) {
        current_goal_distances.resize(num_robots_, 0.0);
    }

    auto cleanup_debug_states = [&]() {
        if (!debug_enabled) {
            return;
        }
        for (int i = 0; i < num_robots_; ++i) {
            if (debug_goal_states[i]) {
                individual_si_[i]->freeState(debug_goal_states[i]);
                debug_goal_states[i] = nullptr;
            }
        }
    };

    while (true) {
        // Check timeout
        auto current_time = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(current_time - start_time).count();
        if (elapsed > timeout) {
            std::cout << "Timeout after " << iterations << " iterations" << std::endl;
            if (debug_enabled) {
                srrtDebugLog("LOOP", "Timeout after " + std::to_string(iterations) +
                                        " iterations. Best goal distances=" +
                                        formatDistances(counters.best_goal_dist));
            }
            cleanup_debug_states();
            return false;
        }

        if (debug_enabled) {
            counters.iterations++;
        }

        // 1. Sample random state in composite space
        ob::State* qs = sampleCompositeState();
        if (debug_enabled) {
            counters.samples++;
        }

        // 2. Find nearest motion in forward tree
        ForwardMotion* qr = findNearestMotion(qs);
        if (debug_enabled && qr == nullptr) {
            srrtDebugLog("LOOP", "Nearest motion query returned null.");
        }

        // 3. Project sample onto subdimensional space
        ob::State* qs_proj = projectState(qs, qr);
        if (debug_enabled) {
            const auto& proj_info = projectionDebugInfo();
            std::ostringstream oss;
            oss << "iter=" << iterations << " choices=[";
            for (int i = 0; i < num_robots_; ++i) {
                if (i > 0) {
                    oss << " ";
                }
                oss << "r" << i << ":" << proj_info.choices[i];
            }
            oss << "]";
            srrtDebugLog("PROJECTION", oss.str());
        }

        // 4. Extend from qr toward qs_proj
        ob::State* qnew = extend(qr, qs_proj);
        if (qnew == nullptr) {
            composite_si_->freeState(qs);
            if (qs_proj) composite_si_->freeState(qs_proj);
            continue;
        }
        if (debug_enabled) {
            std::ostringstream oss;
            oss << "iter=" << iterations
                << " sample=" << formatCompositeState(qs, num_robots_)
                << " nearest=" << formatCompositeState(qr->state, num_robots_)
                << " projected=" << formatCompositeState(qs_proj, num_robots_)
                << " extended=" << formatCompositeState(qnew, num_robots_);
            srrtDebugLog("ITER", oss.str());
        }

        // 5. Check for robot-obstacle collisions
        if (checkRobotObstacleCollision(qnew)) {
            composite_si_->freeState(qs);
            if (qs_proj) composite_si_->freeState(qs_proj);
            composite_si_->freeState(qnew);
            continue;
        }

        // 6. Check for robot-robot collisions
        std::set<int> new_collisions = checkRobotRobotCollisions(qr->state, qnew);

        // 7. Create new motion with updated collision set
        ForwardMotion* new_motion = new ForwardMotion(composite_si_);
        composite_si_->copyState(new_motion->state, qnew);
        new_motion->parent = qr;
        new_motion->collision_set = qr->collision_set;  // Inherit from parent

        // 8. Update collision set with new collisions
        updateCollisionSet(new_motion, new_collisions);

        // 9. Add to tree
        forward_tree_->addMotion(new_motion);
        if (debug_enabled) {
            counters.motions_added++;
            if (new_motion->collision_set.size() > counters.max_collision_set) {
                counters.max_collision_set = new_motion->collision_set.size();
            }
        }

        if (debug_enabled) {
            if (!new_collisions.empty()) {
                srrtDebugLog("COLLISION", "iter=" + std::to_string(iterations) +
                                              " new_collisions=" + formatCollisionSet(new_collisions));
            }

            double max_goal_dist = 0.0;
            for (int i = 0; i < num_robots_; ++i) {
                extractIndividualState(qnew, i, debug_goal_states[i]);
                double dist = individual_si_[i]->distance(debug_goal_states[i], goal_states[i]);
                current_goal_distances[i] = dist;
                if (dist > max_goal_dist) {
                    max_goal_dist = dist;
                }
                if (dist < counters.best_goal_dist[i]) {
                    counters.best_goal_dist[i] = dist;
                    if (debug_level >= 2) {
                        std::ostringstream oss;
                        oss << "Robot " << i << " best_goal_dist="
                            << std::fixed << std::setprecision(3) << dist
                            << " at iter=" << iterations;
                        srrtDebugLog("GOAL", oss.str());
                    }
                }
            }

            std::ostringstream oss;
            oss << "iter=" << iterations
                << " state=" << formatCompositeState(qnew, num_robots_)
                << " goal_dist=" << formatDistances(current_goal_distances)
                << " max_goal_dist=" << std::fixed << std::setprecision(3) << max_goal_dist
                << " collision_set=" << formatCollisionSet(new_motion->collision_set);
            srrtDebugLog("NODE", oss.str());
        }

        // 10. Check if goal reached
        if (isGoalReached(qnew, goal_states)) {
            std::cout << "Goal reached after " << iterations << " iterations!" << std::endl;
            extractPath(new_motion);
            composite_si_->freeState(qs);
            if (qs_proj) composite_si_->freeState(qs_proj);
            composite_si_->freeState(qnew);
            cleanup_debug_states();
            return true;
        }

        composite_si_->freeState(qs);
        if (qs_proj) composite_si_->freeState(qs_proj);
        composite_si_->freeState(qnew);
        iterations++;

        if (iterations % 100 == 0) {
            std::cout << "Iteration " << iterations << ", tree size: "
                      << forward_tree_->getMotions().size() << std::endl;
        }

        if (debug_enabled && debug_every > 0 && iterations % debug_every == 0) {
            std::ostringstream oss;
            oss << "iter=" << iterations
                << " tree=" << forward_tree_->getMotions().size()
                << " added=" << counters.motions_added
                << " samples=" << counters.samples
                << " obstacle_rejects=" << counters.obstacle_rejects
                << " collisions=" << counters.collision_events
                << " zero_extensions=" << counters.zero_extensions
                << " policy_nulls=" << counters.policy_nulls
                << " policy_zero_steps=" << counters.policy_zero_steps
                << " max_collision_set=" << counters.max_collision_set
                << " best_goal_dist=" << formatDistances(counters.best_goal_dist);
            srrtDebugLog("LOOP", oss.str());
        }
    }

    cleanup_debug_states();
    return false;
}

ob::State* sRRTPlanner::sampleCompositeState()
{
    // Sample uniformly in composite configuration space
    ob::State* state = composite_si_->allocState();
    composite_sampler_->sampleUniform(state);
    return state;
}

ForwardMotion* sRRTPlanner::findNearestMotion(ob::State* state)
{
    return forward_tree_->findNearest(state);
}

ob::State* sRRTPlanner::projectState(ob::State* sample, ForwardMotion* nearest)
{
    // Key step: project sample onto subdimensional search space
    // For each robot i:
    //   if i in nearest->collision_set:
    //     qs_proj[i] = sample[i]  (free to explore in full space)
    //   else:
    //     qs_proj[i] = φi(nearest->state[i])  (follow individual policy toward goal)

    // Build vector of individual states for projection
    std::vector<ob::State*> individual_states(num_robots_);
    ProjectionDebugInfo* proj_info = nullptr;
    if (srrtDebugEnabled()) {
        proj_info = &projectionDebugInfo();
        proj_info->choices.assign(num_robots_, std::string());
    }

    for (int i = 0; i < num_robots_; ++i) {
        individual_states[i] = individual_si_[i]->allocState();

        if (nearest->collision_set.find(i) != nearest->collision_set.end()) {
            // Robot i is in collision set - free to explore
            // Use the sampled state component for this robot
            extractIndividualState(sample, i, individual_states[i]);
            if (srrtDebugEnabled()) {
                debugCounters().sample_used++;
                if (proj_info) {
                    proj_info->choices[i] = "sample=" + formatSe2State(individual_states[i]);
                }
            }
        } else {
            // Robot i is NOT in collision set - follow individual policy
            // Extract current state for this robot from nearest node
            ob::State* current_individual = individual_si_[i]->allocState();
            extractIndividualState(nearest->state, i, current_individual);

            // Query policy tree for next state toward goal
            ob::State* policy_state = individual_policies_[i]->getPolicy(current_individual);

            if (policy_state) {
                // Policy found - use it
                individual_si_[i]->copyState(individual_states[i], policy_state);
                if (srrtDebugEnabled()) {
                    debugCounters().policy_used++;
                    double step = individual_si_[i]->distance(current_individual, policy_state);
                    if (step < 1e-6) {
                        debugCounters().policy_zero_steps++;
                        if (srrtDebugLevel() >= 2 && debugCounters().policy_zero_steps <= 5) {
                            srrtDebugLog("POLICY", "Robot " + std::to_string(i) +
                                                   " policy step ~0 from state " +
                                                   formatSe2State(current_individual));
                        }
                    }
                    if (proj_info) {
                        proj_info->choices[i] = "policy_in=" + formatSe2State(current_individual) +
                                                " policy_out=" + formatSe2State(policy_state);
                    }
                }
            } else {
                // Policy returned null (shouldn't happen if tree built properly)
                // Fallback: stay at current state
                individual_si_[i]->copyState(individual_states[i], current_individual);
                if (srrtDebugEnabled()) {
                    debugCounters().policy_nulls++;
                    if (srrtDebugLevel() >= 2 && debugCounters().policy_nulls <= 5) {
                        srrtDebugLog("POLICY", "Robot " + std::to_string(i) +
                                               " policy returned null at state " +
                                               formatSe2State(current_individual));
                    }
                    if (proj_info) {
                        proj_info->choices[i] = "policy_in=" + formatSe2State(current_individual) +
                                                " policy_out=null stay=" +
                                                formatSe2State(current_individual);
                    }
                }
            }

            individual_si_[i]->freeState(current_individual);
        }
    }

    // Combine individual states into composite projected state
    ob::State* projected = combineIndividualStates(individual_states);

    // Free temporary individual states
    for (int i = 0; i < num_robots_; ++i) {
        individual_si_[i]->freeState(individual_states[i]);
    }

    return projected;
}

ob::State* sRRTPlanner::extend(ForwardMotion* from, ob::State* toward)
{
    // Apply local planner to extend from->state toward the target state
    // Respects max_distance_ constraint

    double d = composite_si_->distance(from->state, toward);

    ob::State* qnew = composite_si_->allocState();

    if (srrtDebugEnabled() && d < 1e-6) {
        debugCounters().zero_extensions++;
        if (srrtDebugLevel() >= 2 && (debugCounters().zero_extensions <= 5 ||
                                     debugCounters().zero_extensions % 100 == 0)) {
            srrtDebugLog("EXTEND", "Zero-length extension from " +
                                       formatCompositeState(from->state, num_robots_) +
                                       " toward " + formatCompositeState(toward, num_robots_));
        }
    }

    if (d <= max_distance_) {
        // Within range, use toward directly
        composite_si_->copyState(qnew, toward);
    } else {
        // Interpolate to stay within max_distance_
        composite_si_->getStateSpace()->interpolate(from->state, toward,
                                                     max_distance_ / d, qnew);
    }

    return qnew;
}

bool sRRTPlanner::checkRobotObstacleCollision(ob::State* state)
{
    // Check if any robot collides with static obstacles
    // Uses composite space validity checker
    bool valid = composite_si_->isValid(state);
    if (srrtDebugEnabled() && !valid) {
        auto& counters = debugCounters();
        counters.obstacle_rejects++;
        if (srrtDebugLevel() >= 2 &&
            (counters.obstacle_rejects <= 5 || counters.obstacle_rejects % 100 == 0)) {
            srrtDebugLog("OBSTACLE", "Invalid state rejected count=" +
                                       std::to_string(counters.obstacle_rejects) +
                                       " state=" + formatCompositeState(state, num_robots_));
        }
    }
    return !valid;
}

std::set<int> sRRTPlanner::checkRobotRobotCollisions(ob::State* from, ob::State* to)
{
    std::set<int> colliding_robots;

    // Check if we have Robot objects for FCL-based collision checking
    bool use_fcl = !robots_.empty() && robots_.size() == static_cast<size_t>(num_robots_);

    // Extract individual states for all robots at 'to' position
    std::vector<ob::State*> robot_states(num_robots_);
    for (int i = 0; i < num_robots_; ++i) {
        robot_states[i] = individual_si_[i]->allocState();
        extractIndividualState(to, i, robot_states[i]);
    }

    // Check all pairs of robots for collisions
    for (int i = 0; i < num_robots_; ++i) {
        for (int j = i + 1; j < num_robots_; ++j) {
            bool collision = false;

            if (use_fcl) {
                // FCL-based collision checking
                const int part = 0;  // Assume single part per robot

                // Get collision object for robot i
                const auto& transform_i = robots_[i]->getTransform(robot_states[i], part);
                fcl::CollisionObjectf co_i(robots_[i]->getCollisionGeometry(part));
                co_i.setTranslation(transform_i.translation());
                co_i.setRotation(transform_i.rotation());
                co_i.computeAABB();

                // Get collision object for robot j
                const auto& transform_j = robots_[j]->getTransform(robot_states[j], part);
                fcl::CollisionObjectf co_j(robots_[j]->getCollisionGeometry(part));
                co_j.setTranslation(transform_j.translation());
                co_j.setRotation(transform_j.rotation());
                co_j.computeAABB();

                // Check collision
                fcl::CollisionRequest<float> request;
                fcl::CollisionResult<float> result;
                fcl::collide(&co_i, &co_j, request, result);

                collision = result.isCollision();
            } else {
                // Fallback: distance-based collision checking
                double dist = individual_si_[i]->distance(robot_states[i], robot_states[j]);
                collision = (dist < 2.0 * collision_radius_);
            }

            if (collision) {
                // Both robots involved in collision
                colliding_robots.insert(i);
                colliding_robots.insert(j);
                if (srrtDebugEnabled()) {
                    debugCounters().collision_events++;
                    if (srrtDebugLevel() >= 2 && debugCounters().collision_events <= 5) {
                        srrtDebugLog("COLLISION", "Robot pair (" + std::to_string(i) +
                                                      ", " + std::to_string(j) +
                                                      ") collision detected.");
                    }
                }
            }
        }
    }

    // Free allocated states
    for (int i = 0; i < num_robots_; ++i) {
        individual_si_[i]->freeState(robot_states[i]);
    }

    return colliding_robots;
}

void sRRTPlanner::updateCollisionSet(ForwardMotion* motion, const std::set<int>& new_collisions)
{
    // Add new collisions to this motion and propagate to ancestors
    for (int robot_id : new_collisions) {
        if (motion->collision_set.find(robot_id) == motion->collision_set.end()) {
            motion->collision_set.insert(robot_id);
            if (srrtDebugEnabled()) {
                srrtDebugLog("COLLISION_SET", "Added robot " + std::to_string(robot_id) +
                                                  " new_set=" + formatCollisionSet(motion->collision_set));
            }
            propagateCollisionSet(motion->parent, robot_id);
        }
    }
}

void sRRTPlanner::propagateCollisionSet(ForwardMotion* motion, int robot_id)
{
    // Recursively propagate collision set to ancestors
    if (motion == nullptr)
        return;
    if (motion->collision_set.find(robot_id) != motion->collision_set.end())
        return;  // Already in set, stop propagation

    motion->collision_set.insert(robot_id);
    if (srrtDebugEnabled() && srrtDebugLevel() >= 2) {
        srrtDebugLog("COLLISION_SET", "Propagated robot " + std::to_string(robot_id) +
                                          " new_set=" + formatCollisionSet(motion->collision_set));
    }
    propagateCollisionSet(motion->parent, robot_id);
}

bool sRRTPlanner::isGoalReached(ob::State* state, const std::vector<ob::State*>& goal_states)
{
    // Check if all robots are within goal_threshold_ of their goals
    // Extract individual states and check distance to corresponding goals

    for (int i = 0; i < num_robots_; ++i) {
        ob::State* individual_state = individual_si_[i]->allocState();
        extractIndividualState(state, i, individual_state);

        double dist = individual_si_[i]->distance(individual_state, goal_states[i]);
        individual_si_[i]->freeState(individual_state);

        if (dist > goal_threshold_) {
            return false;
        }
    }

    return true;
}

void sRRTPlanner::extractPath(ForwardMotion* goal_motion)
{
    std::cout << "Extracting solution path..." << std::endl;

    solution_path_.clear();
    ForwardMotion* current = goal_motion;

    // Backtrack through parent pointers
    while (current != nullptr) {
        // Copy state to solution path
        ob::State* state = composite_si_->allocState();
        composite_si_->copyState(state, current->state);
        solution_path_.push_back(state);
        current = current->parent;
    }

    // Reverse to get path from start to goal
    std::reverse(solution_path_.begin(), solution_path_.end());

    std::cout << "Path extracted with " << solution_path_.size() << " waypoints" << std::endl;
}

// ============================================================================
// Helper Functions
// ============================================================================

ob::State* sRRTPlanner::combineIndividualStates(const std::vector<ob::State*>& individual_states)
{
    // Allocate composite state
    ob::State* composite = composite_si_->allocState();

    // Cast to CompoundStateSpace::StateType to access components
    auto* compound_state = composite->as<ob::CompoundStateSpace::StateType>();

    // Copy each individual robot state into the corresponding component
    for (int i = 0; i < num_robots_; ++i) {
        individual_si_[i]->copyState(compound_state->components[i], individual_states[i]);
    }

    return composite;
}

void sRRTPlanner::extractIndividualState(ob::State* composite_state, int robot_id, ob::State* individual_state)
{
    // Cast composite state to CompoundStateSpace::StateType
    auto* compound_state = composite_state->as<ob::CompoundStateSpace::StateType>();

    // Copy the robot's component state to individual_state
    individual_si_[robot_id]->copyState(individual_state, compound_state->components[robot_id]);
}

// ============================================================================
// Main function for testing
// ============================================================================

int main(int argc, char* argv[]) {
    namespace po = boost::program_options;

    // Declare command-line options
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string outputFile;
    std::string cfgFile;
    int timelimit;

    desc.add_options()
        ("help", "produce help message")
        ("input,i", po::value<std::string>(&inputFile)->required(), "input file (yaml)")
        ("output,o", po::value<std::string>(&outputFile)->required(), "output file (yaml)")
        ("cfg,c", po::value<std::string>(&cfgFile)->required(), "configuration file (yaml)")
        ("timelimit", po::value<int>(&timelimit)->default_value(60), "time limit for planner (seconds)");

    try {
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help") != 0u) {
            std::cout << desc << "\n";
            return 0;
        }

        po::notify(vm);
    } catch (po::error& e) {
        std::cerr << e.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;
        return 1;
    }

    std::cout << "sRRT Planner" << std::endl;
    std::cout << "============" << std::endl;
    if (srrtDebugEnabled()) {
        std::cout << "Debug enabled (SRRT_DEBUG=1, level=" << srrtDebugLevel()
                  << ", report_every=" << srrtDebugEvery() << ")" << std::endl;
    }

    // Load configuration file
    std::cout << "Loading configuration file: " << cfgFile << std::endl;
    YAML::Node cfg = YAML::LoadFile(cfgFile);

    double max_distance = cfg["max_distance"].as<double>();
    double goal_threshold = cfg["goal_threshold"].as<double>();
    double policy_timeout = cfg["policy_timeout"].as<double>();

    std::cout << "  Max distance: " << max_distance << std::endl;
    std::cout << "  Goal threshold: " << goal_threshold << std::endl;
    std::cout << "  Policy timeout: " << policy_timeout << std::endl;

    // Load problem description
    std::cout << "Loading problem description: " << inputFile << std::endl;
    YAML::Node env = YAML::LoadFile(inputFile);

    // Get environment bounds
    const auto &env_min = env["environment"]["min"];
    const auto &env_max = env["environment"]["max"];
    ob::RealVectorBounds position_bounds(env_min.size());
    for (size_t i = 0; i < env_min.size(); ++i) {
        position_bounds.setLow(i, env_min[i].as<double>());
        position_bounds.setHigh(i, env_max[i].as<double>());
    }

    std::cout << "  Environment bounds: ["
              << position_bounds.low[0] << ", " << position_bounds.low[1] << "] to ["
              << position_bounds.high[0] << ", " << position_bounds.high[1] << "]" << std::endl;

    // Load obstacles (for robot-obstacle collision checking if needed)
    std::vector<fcl::CollisionObjectf*> obstacles;
    for (const auto &obs : env["environment"]["obstacles"]) {
        if (obs["type"].as<std::string>() == "box") {
            const auto &size = obs["size"];
            std::shared_ptr<fcl::CollisionGeometryf> geom;
            geom.reset(new fcl::Boxf(size[0].as<float>(), size[1].as<float>(), 1.0));
            const auto &center = obs["center"];
            auto co = new fcl::CollisionObjectf(geom);
            co->setTranslation(fcl::Vector3f(center[0].as<float>(), center[1].as<float>(), 0));
            co->computeAABB();
            obstacles.push_back(co);
        }
    }
    std::cout << "  Loaded " << obstacles.size() << " obstacles" << std::endl;

    // Create robots and state spaces
    std::vector<std::shared_ptr<Robot>> robots;
    std::vector<ob::SpaceInformationPtr> individual_si;
    std::vector<ob::State*> start_states;
    std::vector<ob::State*> goal_states;

    std::cout << "Initializing robots..." << std::endl;
    int robot_idx = 0;
    for (const auto &robot_node : env["robots"]) {
        auto robotType = robot_node["type"].as<std::string>();

        // Create Robot object for collision checking
        std::shared_ptr<Robot> robot = create_robot(robotType, position_bounds);
        robots.push_back(robot);

        // Create geometric state space (SE2) for this robot
        auto space = std::make_shared<ob::SE2StateSpace>();
        ob::RealVectorBounds bounds(2);
        bounds.setLow(0, position_bounds.low[0]);
        bounds.setLow(1, position_bounds.low[1]);
        bounds.setHigh(0, position_bounds.high[0]);
        bounds.setHigh(1, position_bounds.high[1]);
        space->setBounds(bounds);

        // Create SpaceInformation
        auto si = std::make_shared<ob::SpaceInformation>(space);
        si->setStateValidityCheckingResolution(0.01);
        si->setup();
        individual_si.push_back(si);

        // Extract start state
        ob::State* start = si->allocState();
        auto start_se2 = start->as<ob::SE2StateSpace::StateType>();
        const auto& start_vec = robot_node["start"];
        start_se2->setX(start_vec[0].as<double>());
        start_se2->setY(start_vec[1].as<double>());
        start_se2->setYaw(start_vec.size() > 2 ? start_vec[2].as<double>() : 0.0);
        start_states.push_back(start);

        // Extract goal state
        ob::State* goal = si->allocState();
        auto goal_se2 = goal->as<ob::SE2StateSpace::StateType>();
        const auto& goal_vec = robot_node["goal"];
        goal_se2->setX(goal_vec[0].as<double>());
        goal_se2->setY(goal_vec[1].as<double>());
        goal_se2->setYaw(goal_vec.size() > 2 ? goal_vec[2].as<double>() : 0.0);
        goal_states.push_back(goal);

        std::cout << "  Robot " << robot_idx << " (" << robotType << "): "
                  << "start=(" << start_se2->getX() << ", " << start_se2->getY() << "), "
                  << "goal=(" << goal_se2->getX() << ", " << goal_se2->getY() << ")" << std::endl;
        robot_idx++;
    }

    int num_robots = robots.size();
    std::cout << "  Total robots: " << num_robots << std::endl;

    // Create composite state space
    std::cout << "Creating composite state space..." << std::endl;
    auto composite_space = std::make_shared<ob::CompoundStateSpace>();
    for (int i = 0; i < num_robots; ++i) {
        composite_space->addSubspace(individual_si[i]->getStateSpace(), 1.0);
    }
    auto composite_si = std::make_shared<ob::SpaceInformation>(composite_space);
    composite_si->setStateValidityCheckingResolution(0.01);
    composite_si->setup();

    // Create sRRT planner
    std::cout << "Creating sRRT planner..." << std::endl;
    sRRTPlanner planner(composite_si, num_robots, individual_si, robots);
    planner.setMaxDistance(max_distance);
    planner.setGoalThreshold(goal_threshold);

    // Solve
    std::cout << "Starting planning (timeout: " << timelimit << "s)..." << std::endl;
    auto start_time = std::chrono::steady_clock::now();

    bool solved = planner.solve(start_states, goal_states, static_cast<double>(timelimit));

    auto end_time = std::chrono::steady_clock::now();
    double planning_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() / 1000.0;

    std::cout << "Planning time: " << planning_time << "s" << std::endl;

    // Output results
    YAML::Node output;
    output["solved"] = solved;
    output["planning_time"] = planning_time;

    if (solved) {
        std::cout << "Solution found!" << std::endl;

        auto solution_path = planner.getSolutionPath();
        std::cout << "  Path length: " << solution_path.size() << " waypoints" << std::endl;

        // Write solution path to YAML in visualizer-compatible format
        // Format: result[robot_idx]["states"] = [[x, y, yaw], ...]
        YAML::Node result_node;

        // Create a path for each robot
        for (int r = 0; r < num_robots; ++r) {
            YAML::Node robot_data;
            YAML::Node states_node;

            // Extract this robot's states from all waypoints
            for (size_t i = 0; i < solution_path.size(); ++i) {
                auto compound_state = solution_path[i]->as<ob::CompoundStateSpace::StateType>();
                auto se2_state = compound_state->components[r]->as<ob::SE2StateSpace::StateType>();

                YAML::Node state;
                state.push_back(se2_state->getX());
                state.push_back(se2_state->getY());
                state.push_back(se2_state->getYaw());
                states_node.push_back(state);
            }

            robot_data["states"] = states_node;
            result_node.push_back(robot_data);
        }

        output["result"] = result_node;
    } else {
        std::cout << "No solution found within time limit." << std::endl;
    }

    // Write output file
    std::cout << "Writing output to: " << outputFile << std::endl;
    std::ofstream fout(outputFile);
    fout << output;
    fout.close();

    // Cleanup
    for (auto* obs : obstacles) {
        delete obs;
    }

    for (size_t i = 0; i < start_states.size(); ++i) {
        individual_si[i]->freeState(start_states[i]);
    }

    for (size_t i = 0; i < goal_states.size(); ++i) {
        individual_si[i]->freeState(goal_states[i]);
    }

    std::cout << "Done!" << std::endl;
    return 0;
}
