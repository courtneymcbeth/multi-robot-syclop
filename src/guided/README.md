# Guided Planners for MR-SyCLoP

This directory contains the guided planner implementations for low-level path planning in the MR-SyCLoP framework.

## Overview

Guided planners take a high-level path (sequence of regions) from the MAPF solver and compute detailed, dynamically-feasible trajectories for individual robots.

## Available Planners

### 1. SyclopRRT
- **File**: [syclop_rrt_solver.h](syclop_rrt_solver.h), [syclop_rrt_solver.cpp](syclop_rrt_solver.cpp)
- **Method**: Sampling-based RRT with decomposition guidance
- **Pros**: Fast, no preprocessing required
- **Cons**: Trajectories may not be optimal
- **Best for**: Quick prototyping, complex dynamics

### 2. DB-RRT (NEW!)
- **File**: [db_rrt_solver.h](db_rrt_solver.h), [db_rrt_solver.cpp](db_rrt_solver.cpp)
- **Method**: Motion primitive-based planning
- **Pros**: Dynamically feasible, high-quality trajectories
- **Cons**: Requires pre-computed primitives
- **Best for**: Robots with good primitive libraries

## Usage

### In Configuration File

```yaml
# Use SyclopRRT (default)
guided_planner_method: "syclop_rrt"

# Use DB-RRT
guided_planner_method: "db_rrt"

guided_planner:
  time_per_robot: 10.0
  debug: true

  # DB-RRT specific (only used if guided_planner_method: "db_rrt")
  db_rrt:
    models_base_path: "db-CBS/dynoplan/dynobench/models"
    max_expands: 10000
    goal_region: 0.3
```

### In Code

```cpp
#include "guided/guided_planner.h"

using namespace mr_syclop;

// Create planner via factory
auto planner = createGuidedPlanner(
    "db_rrt",  // or "syclop_rrt"
    config,
    collision_manager);

// Solve for one robot
auto result = planner->solve(
    robot,
    decomp,
    start_state,
    goal_state,
    region_path,
    robot_index);

if (result.success) {
    // Use result.path
}
```

## Interface

All guided planners implement the `GuidedPlanner` interface:

```cpp
class GuidedPlanner {
public:
    virtual GuidedPlanningResult solve(
        std::shared_ptr<Robot> robot,
        oc::DecompositionPtr decomp,
        ob::State* start_state,
        ob::State* goal_state,
        const std::vector<int>& region_path,
        size_t robot_index) = 0;

    virtual std::string getName() const = 0;
};
```

### Result Structure

```cpp
struct GuidedPlanningResult {
    bool success;                           // Planning succeeded?
    double planning_time;                   // Time taken (seconds)
    std::shared_ptr<oc::PathControl> path;  // Resulting trajectory
    size_t robot_index;                     // Robot ID
};
```

## Adding New Planners

To add a new guided planner:

1. **Create header/source files**: `my_planner.h`, `my_planner.cpp`

2. **Inherit from GuidedPlanner**:
   ```cpp
   namespace mr_syclop {

   class MyPlanner : public GuidedPlanner {
   public:
       GuidedPlanningResult solve(...) override;
       std::string getName() const override { return "MyPlanner"; }
   };

   }
   ```

3. **Update factory** in [guided_planner.cpp](guided_planner.cpp):
   ```cpp
   else if (method == "my_planner") {
       return std::make_unique<MyPlanner>(config, collision_manager);
   }
   ```

4. **Update CMakeLists.txt**:
   ```cmake
   add_library(guided_planner
       ...
       src/guided/my_planner.cpp
   )
   ```

## Documentation

- **DB-RRT Integration**: See [../../DB_RRT_INTEGRATION.md](../../DB_RRT_INTEGRATION.md)
- **MR-SyCLoP Main**: See [../../README.md](../../README.md)

## Files

```
guided/
├── README.md                   # This file
├── guided_planner.h            # Abstract interface
├── guided_planner.cpp          # Factory function
├── syclop_rrt_solver.h         # SyclopRRT header
├── syclop_rrt_solver.cpp       # SyclopRRT implementation
├── db_rrt_solver.h             # DB-RRT header
└── db_rrt_solver.cpp           # DB-RRT implementation
```
