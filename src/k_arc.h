/**
 * @file k_arc.h
 * @brief K-ARC: Kinodynamic Adaptive Robot Coordination
 *
 * Extends ARC to handle kinodynamic constraints through segment-based
 * iterative solution construction. Builds solutions segment by segment,
 * converting kinematic guidance to kinodynamic trajectories one segment
 * at a time. Conflicts are resolved per-segment before advancing.
 *
 * Based on: "K-ARC: Adaptive Robot Coordination for Multi-Robot
 *            Kinodynamic Planning" (2501.01559)
 */

#ifndef K_ARC_H
#define K_ARC_H

// Pinocchio/Crocoddyl must be included before Boost/OMPL headers
#include <dynoplan/optimization/ocp.hpp>

#include <ompl/geometric/PathGeometric.h>
#include <ompl/control/PathControl.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <fcl/fcl.h>
#include <Eigen/Core>
#include <memory>
#include <vector>
#include <string>
#include <random>
#include <chrono>

// Reuse existing structures
// Note: composite_dbrrt.h transitively includes dynoplan/dbrrt/dbrrt.hpp,
// dynobench/robot_models.hpp, and dynobench/motions.hpp. Those headers
// lack include guards so they must NOT be included again directly.
#include "coupled_rrt.h"          // PlanningProblem, RobotSpec
#include "composite_dbrrt.h"      // CompositeDBRRTPlanner, CompositeDBRRTConfig, dynoplan/dbrrt, dynobench
#include "guided/guided_planner.h" // DBRRTConfig, dynobench::Obstacle

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

// Forward declarations
class Robot;

// ============================================================================
// K-ARC Configuration
// ============================================================================

struct KARCConfig {
    // Segmentation
    int num_segments = 4;                    // m in the paper (segments per robot path)

    // Initial kinematic planning
    double initial_planning_time = 10.0;     // Time budget for initial kinematic paths
    double goal_threshold = 0.5;

    // Per-segment kinodynamic planning (DB-RRT)
    mr_syclop::DBRRTConfig db_rrt_config;    // Reuse existing DB-RRT config
    double segment_planning_time = 5.0;      // Time budget per segment per robot

    // Waypoint tolerances
    double waypoint_tolerance = 0.5;         // Intermediate segment goal tolerance
    double final_goal_tolerance = 0.3;       // Final goal tolerance

    // Conflict resolution
    int max_solver_escalations = 3;          // Max times to expand subproblem
    double prioritized_timeout = 5.0;        // Level 1: Prioritized DB-RRT timeout
    double composite_timeout = 30.0;         // Level 2: Composite DB-RRT timeout

    // Composite planner config (for joint planning)
    CompositeDBRRTConfig composite_dbrrt_config;

    // General
    double total_time_limit = 300.0;
    std::string models_base_path;
    std::string motions_file;
    int seed = -1;
    bool debug = false;
};

// ============================================================================
// K-ARC Segment Conflict
// ============================================================================

struct KARCConflict {
    size_t robot_i;
    size_t robot_j;
    int segment_index;                       // Which segment the conflict is in
    int timestep;                            // Timestep within the segment
};

// ============================================================================
// K-ARC Result
// ============================================================================

struct KARCResult {
    bool solved = false;
    double planning_time = 0.0;
    std::vector<dynobench::Trajectory> trajectories;  // Per-robot final trajectories

    // Statistics
    int num_segments = 0;
    int num_conflicts_found = 0;
    int num_conflicts_resolved = 0;
    std::vector<std::string> methods_used;   // Which solver resolved each conflict
    std::string failure_reason;
};

// ============================================================================
// KARCPlanner Class
// ============================================================================

class KARCPlanner {
public:
    explicit KARCPlanner(const KARCConfig& config);
    ~KARCPlanner();

    // Main planning function (Algorithm 1 from K-ARC paper)
    KARCResult plan(const PlanningProblem& problem);

private:
    // ========================================================================
    // Setup Methods
    // ========================================================================

    void setupEnvironment(const PlanningProblem& problem);
    void setupRobots(const PlanningProblem& problem);
    void setupDynobenchObstacles();
    void cleanup();

    // ========================================================================
    // Phase 1: Initial Kinematic Paths (Individual RRTConnect)
    // ========================================================================

    void computeInitialKinematicPaths();

    // ========================================================================
    // Phase 2: Path Segmentation
    // ========================================================================

    void segmentKinematicPaths();

    // ========================================================================
    // Phase 3: Per-Segment Kinodynamic Planning (DB-RRT)
    // ========================================================================

    bool computeKinodynamicSegments(int segment_idx);

    // Helper: Plan a single robot segment using DB-RRT (idbrrt)
    std::pair<bool, dynobench::Trajectory> planRobotSegment(
        size_t robot_idx,
        const Eigen::VectorXd& start_state,
        const Eigen::VectorXd& goal_state,
        double time_budget);

    // ========================================================================
    // Phase 4: Per-Segment Conflict Detection
    // ========================================================================

    std::vector<KARCConflict> findSegmentConflicts(int segment_idx);

    bool checkTwoRobotCollision(
        size_t robot_i, const Eigen::VectorXd& state_i,
        size_t robot_j, const Eigen::VectorXd& state_j) const;

    // ========================================================================
    // Phase 5: Conflict Resolution (Algorithm 2)
    // ========================================================================

    bool resolveConflicts(
        int segment_idx,
        const std::vector<KARCConflict>& conflicts);

    // Level 1: Prioritized DB-RRT (replan one robot treating other as obstacle)
    bool tryPrioritizedDBRRT(
        int segment_idx,
        size_t robot_i, size_t robot_j);

    // Level 2: Composite DB-RRT (joint planning for conflicting robots)
    bool tryCompositeDBRRT(
        int segment_idx,
        const std::vector<size_t>& robot_indices);

    // Subproblem adaptation: expand to previous/next segment boundaries
    bool tryExpandedSubproblem(
        int segment_idx,
        const std::vector<size_t>& robot_indices,
        int escalation_level);

    // ========================================================================
    // Phase 6: Solution Construction
    // ========================================================================

    void concatenateSegments();

    // ========================================================================
    // State Conversion Helpers
    // ========================================================================

    Eigen::VectorXd omplStateToEigen(const ob::State* state,
                                      const std::shared_ptr<Robot>& robot) const;
    void eigenToOmplState(const Eigen::VectorXd& eigen_state,
                          ob::State* ompl_state,
                          const std::shared_ptr<Robot>& robot) const;

    void getPositionBounds(const std::shared_ptr<Robot>& robot,
                           std::vector<double>& min_bounds,
                           std::vector<double>& max_bounds) const;

    std::string getRobotType(const std::shared_ptr<Robot>& robot) const;

    // ========================================================================
    // Trajectory Conversion
    // ========================================================================

    std::shared_ptr<oc::PathControl> convertDynobenchTrajectory(
        const dynobench::Trajectory& traj,
        const std::shared_ptr<Robot>& robot) const;

    // ========================================================================
    // Timing
    // ========================================================================

    bool isTimeoutExceeded() const;

    // ========================================================================
    // Member Variables
    // ========================================================================

    KARCConfig config_;

    // Problem data
    PlanningProblem problem_;
    ob::RealVectorBounds position_bounds_;

    // Collision detection
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_environment_;

    // Dynobench obstacles
    std::vector<dynobench::Obstacle> dynobench_obstacles_;

    // Robots
    std::vector<std::shared_ptr<Robot>> robots_;
    std::vector<ob::State*> start_states_;
    std::vector<ob::State*> goal_states_;

    // Phase 1: Kinematic paths (geometric, for guidance only)
    std::vector<std::shared_ptr<og::PathGeometric>> kinematic_paths_;

    // Phase 2: Segmented kinematic waypoints
    // waypoints_[robot_idx][segment_idx] = Eigen state at segment boundary
    std::vector<std::vector<Eigen::VectorXd>> waypoints_;

    // Phase 3: Per-segment kinodynamic trajectories
    // segment_trajectories_[robot_idx][segment_idx] = dynobench trajectory
    std::vector<std::vector<dynobench::Trajectory>> segment_trajectories_;

    // Current achieved states (end of last completed segment)
    std::vector<Eigen::VectorXd> achieved_states_;

    // Final concatenated trajectories
    std::vector<dynobench::Trajectory> final_trajectories_;

    // Result tracking
    KARCResult result_;

    // Timing
    std::chrono::steady_clock::time_point planning_start_time_;

    // RNG
    std::mt19937 rng_;
};

// ============================================================================
// Configuration Loading
// ============================================================================

KARCConfig loadKARCConfig(const YAML::Node& node);

#endif // K_ARC_H
