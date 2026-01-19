#ifndef ARC_H
#define ARC_H

#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/State.h>
#include <fcl/fcl.h>
#include <memory>
#include <vector>
#include <string>

// Reuse existing structures
#include "coupled_rrt.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Forward declarations
class Robot;

// ============================================================================
// Configuration Structure
// ============================================================================

struct ARCConfig {
    // Overall planning parameters
    double time_limit = 60.0;
    double goal_threshold = 0.5;
    int seed = -1;  // Random seed (-1 for random)

    // Subproblem configuration
    int initial_time_window = 10;          // Initial timesteps before/after conflict
    int time_window_expansion_step = 5;    // Timesteps to expand each iteration
    int max_subproblem_expansions = 5;     // Max times to expand subproblem spatially/temporally
    double spatial_expansion_factor = 1.5; // Factor to expand environment bounds

    // Planning hierarchy timeouts (for Algorithm 2)
    double prioritized_query_timeout = 1.0;
    double decoupled_prm_timeout = 5.0;
    double composite_prm_timeout = 10.0;

    // Sampling parameters for PRM methods
    int prm_samples = 1000;                // Number of samples for PRM
    int prm_nearest_neighbors = 10;        // k-nearest neighbors for PRM

    // Path discretization (for collision checking)
    int states_per_check = 10;             // Number of states to interpolate between waypoints

    // Termination criteria
    int max_conflicts_resolved = 100;      // Maximum number of conflicts to resolve
};

// ============================================================================
// Conflict Structure
// ============================================================================

struct Conflict {
    size_t robot_i;                        // Index of first robot
    size_t robot_j;                        // Index of second robot
    int timestep;                          // Timestep where conflict occurs
    ob::State* config_i;                   // Configuration of robot i at conflict (allocated)
    ob::State* config_j;                   // Configuration of robot j at conflict (allocated)

    Conflict() : robot_i(0), robot_j(0), timestep(0), config_i(nullptr), config_j(nullptr) {}

    // Helper to free allocated states (implementation in .cpp to avoid incomplete type)
    void freeStates(const std::vector<std::shared_ptr<Robot>>& robots);
};

// ============================================================================
// Subproblem Structure
// ============================================================================

struct Subproblem {
    // Robots involved in this subproblem
    std::vector<size_t> robot_indices;

    // Local start and goal states for subproblem (allocated from robot state spaces)
    std::vector<ob::State*> local_starts;
    std::vector<ob::State*> local_goals;

    // Local environment bounds (can be smaller than global environment)
    std::vector<double> env_min;
    std::vector<double> env_max;

    // Time window parameters
    int start_timestep;                    // Timestep where subproblem starts
    int end_timestep;                      // Timestep where subproblem ends
    int time_window;                       // Current time window size (timesteps on each side of conflict)

    // Obstacles in the local region
    std::vector<fcl::CollisionObjectf*> local_obstacles;

    // Track expansion attempts
    int num_expansions;

    Subproblem() : start_timestep(0), end_timestep(0), time_window(0), num_expansions(0) {}

    // Helper to free allocated states (implementation in .cpp to avoid incomplete type)
    void freeStates(const std::vector<std::shared_ptr<Robot>>& robots);
};

// ============================================================================
// Subproblem Solution Structure
// ============================================================================

struct SubproblemSolution {
    bool solved;                                           // Whether solution was found
    double planning_time;                                  // Time spent solving
    std::vector<std::shared_ptr<og::PathGeometric>> paths;  // Paths for each robot in subproblem
    std::string method_used;                               // Which method solved it (for debugging)

    SubproblemSolution() : solved(false), planning_time(0.0), method_used("none") {}
};

// ============================================================================
// Discretized Path Structure
// ============================================================================

struct DiscretizedPath {
    std::vector<ob::State*> states;        // States at each timestep (allocated)
    int num_timesteps;                     // Total number of timesteps

    DiscretizedPath() : num_timesteps(0) {}

    // Helper to free allocated states (implementation in .cpp to avoid incomplete type)
    void freeStates(const std::shared_ptr<Robot>& robot);
};

// ============================================================================
// ARC Result Structure
// ============================================================================

struct ARCResult {
    bool solved;                                           // Whether exact solution was found
    double planning_time;                                  // Total time spent planning (seconds)
    std::vector<std::shared_ptr<og::PathGeometric>> paths;  // Individual robot paths

    // Statistics
    int num_conflicts_found;                               // Total conflicts detected
    int num_conflicts_resolved;                            // Conflicts successfully resolved
    int num_subproblems_created;                           // Number of subproblems created
    std::vector<int> robots_per_subproblem;                // Number of robots in each subproblem
    std::vector<std::string> methods_used;                 // Which method solved each subproblem

    ARCResult() : solved(false), planning_time(0.0),
                  num_conflicts_found(0), num_conflicts_resolved(0),
                  num_subproblems_created(0) {}
};

// ============================================================================
// ARCPlanner Class
// ============================================================================

class ARCPlanner {
public:
    // Constructor
    explicit ARCPlanner(const ARCConfig& config);

    // Destructor
    ~ARCPlanner();

    // Main planning function (Algorithm 1 from paper)
    ARCResult plan(const PlanningProblem& problem);

private:
    // ========================================================================
    // Algorithm 1: Main ARC Loop
    // ========================================================================

    // Step 1: Compute initial individual paths
    void computeInitialPaths();

    // Step 2: Find first conflict in current paths
    Conflict* findConflict();

    // Step 3: Create subproblem around conflict
    Subproblem createSubproblem(const Conflict& conflict);

    // Step 4: Solve subproblem (calls Algorithm 2)
    SubproblemSolution solveSubproblem(Subproblem& subproblem);

    // Step 5: Update solution with resolved conflict
    void updateSolution(const SubproblemSolution& solution, const Subproblem& subproblem);

    // ========================================================================
    // Algorithm 2: Subproblem Solving Hierarchy
    // ========================================================================

    // Level 1: Try prioritized query (robots wait for each other)
    SubproblemSolution tryPrioritizedQuery(const Subproblem& subproblem);

    // Level 2: Try decoupled PRM (sample individual spaces)
    SubproblemSolution tryDecoupledPRM(const Subproblem& subproblem);

    // Level 3: Try composite PRM (sample composite space)
    SubproblemSolution tryCompositePRM(const Subproblem& subproblem);

    // ========================================================================
    // Subproblem Management
    // ========================================================================

    // Expand subproblem spatially (increase environment bounds)
    void expandSubproblemSpatially(Subproblem& subproblem);

    // Expand subproblem temporally (increase time window)
    void expandSubproblemTemporally(Subproblem& subproblem);

    // Check if additional robots should be added to subproblem
    std::vector<size_t> getConflictingRobots(const Subproblem& subproblem);

    // Update subproblem with additional robots
    void addRobotsToSubproblem(Subproblem& subproblem, const std::vector<size_t>& new_robots);

    // Get obstacles that overlap with subproblem region
    std::vector<fcl::CollisionObjectf*> getObstaclesInRegion(
        const std::vector<double>& region_min,
        const std::vector<double>& region_max) const;

    // ========================================================================
    // Conflict Detection
    // ========================================================================

    // Discretize continuous paths into timesteps
    void discretizePaths();

    // Check if two robots collide at a specific timestep
    bool checkCollision(size_t robot_i, size_t robot_j, int timestep);

    // Get state of robot at a specific timestep (from discretized path)
    ob::State* getStateAtTimestep(size_t robot_idx, int timestep) const;

    // FCL collision check between two robot states
    bool checkFCLCollision(size_t robot_i, const ob::State* state_i,
                          size_t robot_j, const ob::State* state_j) const;

    // ========================================================================
    // Path Integration
    // ========================================================================

    // Splice local solution into global paths
    void splicePaths(const SubproblemSolution& solution, const Subproblem& subproblem);

    // Interpolate between two states to create a path segment
    std::shared_ptr<og::PathGeometric> interpolatePath(
        size_t robot_idx,
        const ob::State* start,
        const ob::State* goal);

    // ========================================================================
    // Helper Methods
    // ========================================================================

    // Setup environment (obstacles, bounds)
    void setupEnvironment(const PlanningProblem& problem);

    // Setup robots (create Robot objects)
    void setupRobots(const PlanningProblem& problem);

    // Cleanup allocated memory
    void cleanup();

    // Compute maximum timesteps across all paths
    int getMaxTimesteps() const;

    // ========================================================================
    // Member Variables
    // ========================================================================

    // Configuration
    ARCConfig config_;

    // Problem specification
    PlanningProblem problem_;
    ob::RealVectorBounds position_bounds_;

    // Collision detection
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_environment_;

    // Robots
    std::vector<std::shared_ptr<Robot>> robots_;
    std::vector<ob::State*> start_states_;
    std::vector<ob::State*> goal_states_;

    // Current solution
    std::vector<std::shared_ptr<og::PathGeometric>> current_paths_;
    std::vector<DiscretizedPath> discretized_paths_;

    // Statistics
    ARCResult result_;

    // Timing
    double planning_start_time_;
};

#endif // ARC_H
