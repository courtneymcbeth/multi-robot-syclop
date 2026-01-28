#ifndef COUPLED_RRT_H
#define COUPLED_RRT_H

#include <ompl/control/PathControl.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/PathGeometric.h>
#include <fcl/fcl.h>
#include <memory>
#include <vector>
#include <string>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

// ============================================================================
// Configuration Structures
// ============================================================================

struct CoupledRRTConfig {
    double time_limit = 60.0;
    double goal_threshold = 0.5;
    int min_control_duration = 1;
    int max_control_duration = 10;
    int seed = -1;  // Random seed (-1 for random)
    bool use_geometric = false;  // Use geometric RRT (default: kinodynamic)
};

// ============================================================================
// Input Structures
// ============================================================================

struct RobotSpec {
    std::string type;              // Robot type string (e.g., "DiffDrive", "Unicycle")
    std::vector<double> start;     // Start state [x, y, theta, ...]
    std::vector<double> goal;      // Goal state [x, y, theta, ...]
};

struct PlanningProblem {
    std::vector<double> env_min;   // Environment bounds minimum [x_min, y_min]
    std::vector<double> env_max;   // Environment bounds maximum [x_max, y_max]
    std::vector<RobotSpec> robots; // Robot specifications
    std::vector<fcl::CollisionObjectf*> obstacles; // FCL collision objects for obstacles
};

// ============================================================================
// Output Structure
// ============================================================================

struct PlanningResult {
    bool solved;                               // Whether exact solution was found
    double planning_time;                      // Time spent planning (seconds)
    std::shared_ptr<oc::PathControl> path;     // Solution path (compound path for all robots) - kinodynamic
    std::shared_ptr<og::PathGeometric> geometric_path;  // Solution path (compound path for all robots) - geometric
    std::vector<std::shared_ptr<oc::PathControl>> individual_paths;  // Individual paths per robot (for composite DB-RRT)
};

// ============================================================================
// Forward Declarations
// ============================================================================

class Robot;
class CompoundStatePropagator;
class CompoundStateValidityChecker;

// ============================================================================
// CoupledRRTPlanner Class
// ============================================================================

class CoupledRRTPlanner {
public:
    // Constructor
    explicit CoupledRRTPlanner(const CoupledRRTConfig& config);

    // Destructor
    ~CoupledRRTPlanner();

    // Main planning function
    PlanningResult plan(const PlanningProblem& problem);

private:
    // Configuration
    CoupledRRTConfig config_;

    // Helper methods for setup
    void setupEnvironment(const PlanningProblem& problem);
    void setupRobots(const PlanningProblem& problem);
    void setupCompoundSpaces();
    void setupProblemDefinition();
    void cleanup();

    // Internal state (cleaned up between planning calls)
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_environment_;
    std::vector<fcl::CollisionObjectf*> obstacles_;
    std::vector<std::shared_ptr<Robot>> robots_;
    std::vector<ob::State*> start_states_;
    std::vector<ob::State*> goal_states_;

    std::shared_ptr<ob::CompoundStateSpace> compound_state_space_;
    std::shared_ptr<oc::CompoundControlSpace> compound_control_space_;  // Only used in kinodynamic mode
    std::shared_ptr<ob::SpaceInformation> compound_si_;  // Base type works for both geometric and kinodynamic
    std::shared_ptr<ob::ProblemDefinition> pdef_;

    ob::RealVectorBounds position_bounds_;
};

#endif // COUPLED_RRT_H
