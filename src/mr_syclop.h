#ifndef MR_SYCLOP_H
#define MR_SYCLOP_H

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <fcl/fcl.h>
#include <memory>
#include <vector>
#include <string>

#include "decomposition.h"
#include "robots.h"
#include "coupled_rrt.h"
#include "guided/guided_planner.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

// ============================================================================
// Configuration Structure
// ============================================================================

struct MAPFConfig {
    std::string method = "decoupled";  // Options: "decoupled", "astar", "cbs"
    int region_capacity = 1;           // Robots per vertex/edge (for CBS)
};

struct MRSyCLoPConfig {
    int decomposition_region_length = 1;
    double planning_time_limit = 60.0;

    // MAPF configuration
    MAPFConfig mapf_config;

    // Coupled RRT config for composite planner
    CoupledRRTConfig coupled_rrt_config;

    // Guided planner configuration
    std::string guided_planner_method = "syclop_rrt";
    GuidedPlannerConfig guided_planner_config;

    // Segmentation configuration
    int segment_timesteps = 30;  // Number of timesteps per segment
};

// ============================================================================
// Path Segment Structure
// ============================================================================

struct PathSegment {
    size_t robot_index;           // Which robot this segment belongs to
    size_t segment_index;         // Index of this segment in the robot's path
    ob::State* start_state;       // Start state of segment (owned by original path)
    ob::State* end_state;         // End state of segment (owned by original path)
    std::vector<oc::Control*> controls;      // Controls in this segment (owned by original path)
    std::vector<double> control_durations;   // Duration for each control (may be partial)
    double total_duration;        // Total duration of this segment
    int start_timestep;           // Starting timestep of this segment
    int end_timestep;             // Ending timestep of this segment
};

// ============================================================================
// Planning Result Structure
// ============================================================================

struct MRSyCLoPResult {
    bool success = false;
    double planning_time = 0.0;
    // TODO: Add path/trajectory data structures as needed
};

// ============================================================================
// MRSyCLoPPlanner Class
// ============================================================================

class MRSyCLoPPlanner {
public:
    // Constructor
    explicit MRSyCLoPPlanner(const MRSyCLoPConfig& config);

    // Destructor
    ~MRSyCLoPPlanner();

    // Load problem from data structures
    void loadProblem(
        const std::vector<std::string>& robot_types,
        const std::vector<std::vector<double>>& starts,
        const std::vector<std::vector<double>>& goals,
        const std::vector<fcl::CollisionObjectf*>& obstacles,
        const std::vector<double>& env_min,
        const std::vector<double>& env_max);

    // Main planning method - runs full SyCLoP pipeline
    MRSyCLoPResult plan();

    // Individual planning phases (can be called separately if needed)
    void computeHighLevelPaths();
    void computeGuidedPaths();
    void segmentGuidedPaths();
    bool checkSegmentsForCollisions();
    void resolveCollisions();

    // Accessors
    const std::vector<std::vector<int>>& getHighLevelPaths() const { return high_level_paths_; }
    const oc::DecompositionPtr& getDecomposition() const { return decomp_; }
    const std::vector<GuidedPlanningResult>& getGuidedPaths() const { return guided_planning_results_; }
    const std::vector<std::vector<PathSegment>>& getPathSegments() const { return path_segments_; }

private:
    // Configuration
    MRSyCLoPConfig config_;

    // Environment data
    std::vector<fcl::CollisionObjectf*> obstacles_;
    std::vector<double> env_min_;
    std::vector<double> env_max_;
    ob::RealVectorBounds workspace_bounds_;
    oc::DecompositionPtr decomp_;

    // Robot data
    std::vector<std::shared_ptr<Robot>> robots_;
    std::vector<std::string> robot_types_;
    std::vector<std::vector<double>> starts_;
    std::vector<std::vector<double>> goals_;
    std::vector<ob::State*> start_states_;
    std::vector<ob::State*> goal_states_;

    // Planning state
    std::vector<std::vector<int>> high_level_paths_;
    std::vector<GuidedPlanningResult> guided_planning_results_;
    std::vector<std::vector<PathSegment>> path_segments_;  // Segments for each robot
    bool problem_loaded_ = false;

    // Collision manager for obstacles (shared with guided planners)
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager_;

    // Helper methods
    void setupDecomposition();
    void setupCollisionManager();
    void setupRobots();
    void cleanup();
    std::vector<fcl::CollisionObjectf*> getObstaclesInRegion(
        const std::vector<double>& region_min,
        const std::vector<double>& region_max) const;

    // Collision resolution strategies
    void updateDecomposition();
    void expandSubproblem();
    PlanningResult useCompositePlanner(
        const std::vector<size_t>& robot_indices,
        const std::vector<std::vector<double>>& subproblem_starts,
        const std::vector<std::vector<double>>& subproblem_goals,
        const std::vector<double>& subproblem_env_min,
        const std::vector<double>& subproblem_env_max);
};

#endif // MR_SYCLOP_H
