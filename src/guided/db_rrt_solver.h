/**
 * @file db_rrt_solver.h
 * @brief DB-RRT solver implementation as a GuidedPlanner for mr_syclop
 *
 * This class wraps the dynobench db-rrt planner to work within the mr_syclop
 * framework. It converts between OMPL and dynobench representations and
 * integrates motion primitive-based planning with region-guided search.
 */

#pragma once

// Pinocchio/Crocoddyl must be included before Boost/OMPL headers
#include <dynoplan/optimization/ocp.hpp>

#include "guided_planner.h"

#include <memory>
#include <string>
#include <vector>

// dynobench includes
#include "dynobench/general_utils.hpp"
#include "dynobench/robot_models.hpp"
#include "dynoplan/dbrrt/dbrrt.hpp"
#include "dynoplan/ompl/robots.h"

namespace mr_syclop {

// DBRRTConfig is defined in guided_planner.h

/**
 * @brief DB-RRT solver that implements the GuidedPlanner interface
 *
 * This class bridges between the OMPL-based mr_syclop framework and the
 * dynobench-based db-rrt planner. It handles:
 * - Converting OMPL states/controls to dynobench format
 * - Loading and using motion primitives
 * - Integrating region-based guidance from the high-level path
 * - Converting results back to OMPL format
 */
class DBRRTSolver : public GuidedPlanner {
public:
    /**
     * @brief Constructor
     * @param config General guided planner configuration
     * @param db_config DB-RRT specific configuration
     * @param collision_manager FCL collision manager for environment
     * @param dynobench_obstacles Precomputed obstacles in dynobench format
     */
    DBRRTSolver(const GuidedPlannerConfig& config,
                const DBRRTConfig& db_config,
                std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager,
                const std::vector<dynobench::Obstacle>& dynobench_obstacles = {});

    /**
     * @brief Destructor
     */
    ~DBRRTSolver() override = default;

    /**
     * @brief Solve guided planning problem using DB-RRT
     * @param robot Robot model
     * @param decomp Region decomposition
     * @param start_state Start state (OMPL format)
     * @param goal_state Goal state (OMPL format)
     * @param region_path Sequence of region indices from high-level path
     * @param robot_index Index of the robot being planned for
     * @return Planning result with path and statistics
     */
    GuidedPlanningResult solve(
        std::shared_ptr<::Robot> robot,
        oc::DecompositionPtr decomp,
        ob::State* start_state,
        ob::State* goal_state,
        const std::vector<int>& region_path,
        size_t robot_index) override;

    /**
     * @brief Get the name of this planner
     */
    std::string getName() const override { return "DB-RRT"; }

    /**
     * @brief Get the DB-RRT configuration
     */
    const DBRRTConfig& getDBRRTConfig() const { return db_config_; }

    /**
     * @brief Set motion primitives file
     */
    void setMotionsFile(const std::string& file) { db_config_.motions_file = file; }

    /**
     * @brief Set models base path
     */
    void setModelsBasePath(const std::string& path) { db_config_.models_base_path = path; }

private:
    /**
     * @brief Convert OMPL state to dynobench state vector
     */
    std::vector<double> omplStateToDynobench(const ob::State* state,
                                             const std::shared_ptr<::Robot>& robot);

    /**
     * @brief Convert dynobench state vector to OMPL state
     */
    void dynobenchStateToOmpl(const std::vector<double>& state,
                              ob::State* ompl_state,
                              const std::shared_ptr<::Robot>& robot);

    /**
     * @brief Create dynobench Problem from OMPL inputs
     */
    std::shared_ptr<dynobench::Problem> createDynobenchProblem(
        const std::shared_ptr<::Robot>& robot,
        const ob::State* start_state,
        const ob::State* goal_state,
        const oc::DecompositionPtr& decomp);

    /**
     * @brief Convert dynobench trajectory to OMPL PathControl
     */
    std::shared_ptr<oc::PathControl> convertTrajectory(
        const dynobench::Trajectory& traj,
        const std::shared_ptr<::Robot>& robot);

    /**
     * @brief Load motion primitives for the given robot
     */
    bool loadMotionPrimitives(
        const std::unique_ptr<dynobench::Model_robot>& dynobench_robot,
        std::vector<dynoplan::Motion>& motions);

    /**
     * @brief Create custom sampling function that biases towards regions
     */
    std::function<void(Eigen::Ref<Eigen::VectorXd>)> createRegionGuidedSampler(
        const std::vector<int>& region_path,
        const oc::DecompositionPtr& decomp);

    /**
     * @brief Get robot type string from Robot object
     */
    std::string getRobotType(const std::shared_ptr<::Robot>& robot);

    // =========================================================================
    // SYCLOP-style region guidance methods
    // =========================================================================

    /**
     * @brief Solve using SYCLOP-style waypoint planning through regions
     *
     * This method plans through intermediate waypoints sampled from regions
     * along the high-level MAPF path, similar to how SyclopRRT follows its lead.
     */
    GuidedPlanningResult solveWithRegionGuidance(
        std::shared_ptr<::Robot> robot,
        oc::DecompositionPtr decomp,
        ob::State* start_state,
        ob::State* goal_state,
        const std::vector<int>& region_path,
        size_t robot_index);

    /**
     * @brief Get the center point of a region in the decomposition
     */
    Eigen::VectorXd getRegionCenter(int region_id, const oc::DecompositionPtr& decomp);

    /**
     * @brief Sample a random point within a region
     */
    Eigen::VectorXd sampleInRegion(int region_id, const oc::DecompositionPtr& decomp);

    /**
     * @brief Check if a state is within a given region
     */
    bool isStateInRegion(const Eigen::VectorXd& state, int region_id,
                         const oc::DecompositionPtr& decomp);

    /**
     * @brief Find the current lead region index based on robot state
     */
    size_t findCurrentLeadIndex(const Eigen::VectorXd& state,
                                const std::vector<int>& region_path,
                                const oc::DecompositionPtr& decomp);

    /**
     * @brief Create intermediate waypoint goal from region
     */
    Eigen::VectorXd createWaypointGoal(int region_id,
                                       const Eigen::VectorXd& current_state,
                                       const Eigen::VectorXd& final_goal,
                                       const oc::DecompositionPtr& decomp);

    /**
     * @brief Plan a single segment from current state toward a waypoint
     */
    std::pair<bool, dynobench::Trajectory> planSegmentToWaypoint(
        const std::shared_ptr<::Robot>& robot,
        const Eigen::VectorXd& start,
        const Eigen::VectorXd& waypoint,
        const oc::DecompositionPtr& decomp,
        double time_budget);

    /**
     * @brief Concatenate multiple trajectory segments
     */
    dynobench::Trajectory concatenateTrajectories(
        const std::vector<dynobench::Trajectory>& segments);

    /**
     * @brief Get state bounds from Robot (includes all dimensions including orientation)
     */
    void getStateBounds(const std::shared_ptr<::Robot>& robot,
                       std::vector<double>& min_bounds,
                       std::vector<double>& max_bounds);

    /**
     * @brief Get position-only bounds from Robot (excludes orientation)
     * For dynobench p_lb/p_ub which only want translation bounds
     */
    void getPositionBounds(const std::shared_ptr<::Robot>& robot,
                          std::vector<double>& min_bounds,
                          std::vector<double>& max_bounds);

    // Configuration
    DBRRTConfig db_config_;

    // Collision manager
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager_;

    // Precomputed dynobench obstacles
    std::vector<dynobench::Obstacle> dynobench_obstacles_;
};

/**
 * @brief Load DB-RRT configuration from YAML node
 */
DBRRTConfig loadDBRRTConfig(const YAML::Node& node);

} // namespace mr_syclop
