# MRSyCLoP Implementation Documentation

## Overview

This document describes the implementation of **MRSyCLoP** (Multi-Robot Synergistic Combination of Layers of Planning), a geometric multi-robot motion planner that extends the SyCLoP framework to handle multiple robots simultaneously.

## What is MRSyCLoP?

MRSyCLoP is a hierarchical planning algorithm that combines:
1. **High-level discrete planning**: Guides exploration through workspace decomposition into regions
2. **Low-level geometric planning**: Uses RRT-based tree expansion for actual motion planning
3. **Multi-robot coordination**: Handles multiple robots with both individual and composite space planning

## Architecture

### File Structure

```
Multi-Robot-OMPL/src/ompl/multirobot/geometric/planners/mrsyclop/
├── MRSyCLoP.h              # Header file with class declaration
└── src/
    └── MRSyCLoP.cpp        # Implementation file
```

### Class Hierarchy

```
ompl::multirobot::base::Planner
    └── ompl::multirobot::geometric::MRSyCLoP
```

## Algorithm Overview

### High-Level Flow

```
1. Initialize decomposition and region graph
2. Estimate free volumes for each region
3. Add start states and identify goal regions
4. Compute "leads" (region sequences) for each robot using A*
5. Main planning loop:
   a. Group robots by current region in their leads
   b. For each region with robots:
      - If single robot: grow individual RRT
      - If multiple robots: grow composite RRT (handles coordination)
   c. Check if robots reached next region → advance timestep
   d. Check if all robots reached goals
6. Extract and return solution paths
```

### Key Concepts

#### Decomposition
- Workspace is divided into discrete regions
- Uses `ompl::control::Decomposition` interface
- Each region has an ID, free volume estimate, and coverage metric

#### Leads
- A "lead" is a sequence of regions from start to goal
- Computed using A* search on the region graph
- Each robot has its own lead
- Example: `[region_0, region_3, region_7, region_12]`

#### Timesteps
- Each robot advances through its lead at discrete timesteps
- When a robot reaches the next region in its lead, timestep increments
- All robots in the same region can be coordinated via composite space planning

#### Composite Space Planning
- When multiple robots occupy the same region simultaneously
- Creates a compound state space combining individual robot spaces
- Automatically checks inter-robot collisions
- Allows coordinated motion planning

## Key Components

### 1. Region Graph (`initRegionGraph()`)

Builds a graph representation of the workspace:
- **Nodes**: Regions from decomposition
- **Edges**: Adjacent regions (from `decomposition->getNeighbors()`)
- **Attributes**: Free volume, coverage, alpha values

```cpp
struct Region {
    int id;
    double freeVolume;      // Estimated collision-free volume
    double coverage;        // Coverage by search tree
    unsigned int numSelections;
    double alpha;           // Cost metric for A* search
    std::vector<ompl::base::State *> states;
};

struct Adjacency {
    int region1, region2;
    double cost;            // Edge cost for A* search
    double coverage;        // Edge coverage
    unsigned int numSelections;
};
```

### 2. Free Volume Estimation (`estimateFreeVolumes()`)

Estimates the collision-free volume of each region:
- Samples `numFreeVolSamples_` random states (default: 10,000)
- Checks validity using first robot's state space
- Computes ratio: `freeVolume = (valid_samples / total_samples) * region_volume`
- Updates alpha values: `alpha = 1 / ((1 + coverage) * freeVolume^4)`

### 3. Lead Computation (`computeLeadForRobot()`)

Uses A* search to find optimal region sequence:
- **Cost function**: Based on edge costs and region alpha values
- **Heuristic**: Uses region alpha (favors less-covered, larger free volume regions)
- **Output**: Sequence of region IDs from start to goal

```cpp
// Example lead for robot 0:
leads_[0] = {5, 12, 18, 24, 30};  // Start in region 5, goal in region 30
```

### 4. Individual RRT Growth (`growIndividualRRT()`)

Expands search tree for a single robot:

```cpp
1. Sample target state:
   - 80% from next region in lead (to make progress)
   - 20% from current region (exploration)
   - If at goal region: 90% sample directly from goal

2. Extend current state toward target:
   - Step size: 10% of state space extent
   - Interpolate if distance > step size

3. Validate motion and update current state

4. Store segment for path reconstruction
```

**Key features**:
- Biases sampling toward next region to encourage progress
- Heavy goal bias (90%) when in final region
- Simple one-step RRT extension (no tree data structure)

### 5. Composite RRT Growth (`growCompositeRRT()`)

Coordinates multiple robots in same region:

```cpp
1. Create composite space:
   - Compound state space of all robots in group
   - Custom validity checker for inter-robot collisions

2. Create composite current state from individual states

3. Sample target composite state:
   - Each robot component samples from ITS OWN next region (80% probability)
   - Or from current region (20% probability for exploration)
   - This allows coordinated progress toward individual goals

4. Extend in composite space toward sampled target

5. Extract individual states from new composite state

6. Update each robot's current state
```

**Key Feature - Individual Progress Bias**:
Unlike earlier implementations that sampled all robots from the same region, each robot now samples from its own next region. This allows robots to make coordinated progress toward their different goals while avoiding collisions.

**Collision checking**:
```cpp
// Composite validity checker checks:
- Individual robot validity (obstacles, workspace bounds)
- Inter-robot collisions using areStatesValid()
```

### 6. Composite Space Creation (`createCompositeSpaceInfo()`)

```cpp
auto compoundSpace = std::make_shared<ompl::base::CompoundStateSpace>();

for (unsigned int r : robotIndices) {
    compoundSpace->addSubspace(
        siG_->getIndividual(r)->getStateSpace(),
        1.0  // equal weight
    );
}

compoundSpace->lock();
auto compositeSI = std::make_shared<ompl::base::SpaceInformation>(compoundSpace);

// Set custom validity checker that checks:
// 1. Each robot's individual validity
// 2. Inter-robot collisions
compositeSI->setStateValidityChecker(...);
```

## Data Structures

### RRTSegment
Represents a segment of the RRT tree:

```cpp
struct RRTSegment {
    ompl::base::PlannerPtr planner;            // RRT planner instance
    std::vector<unsigned int> robotIndices;     // Which robots
    int regionId;                               // Which region
    unsigned int timestep;                      // High-level timestep
    ompl::base::State *startState;             // Segment start
    ompl::base::State *endState;               // Segment end
    bool isComposite;                           // Composite vs individual
};
```

### Per-Robot State
```cpp
std::vector<std::vector<int>> leads_;              // Lead for each robot
std::vector<unsigned int> currentTimestep_;        // Current position in lead
std::vector<std::vector<RRTSegment>> segments_;    // RRT segments per robot
std::vector<ompl::base::State *> currentStates_;   // Current state per robot
std::vector<int> startRegions_;                    // Start region per robot
std::vector<int> goalRegions_;                     // Goal region per robot
```

## Algorithm Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_region_expansions` | 100 | Number of RRT expansions per region per iteration |
| `num_tree_selections` | 1 | Tree selections per region expansion |
| `prob_shortest_path` | 0.95 | Probability of using A* vs random exploration for leads |
| `num_free_vol_samples` | 10,000 | Samples for estimating region free volumes |
| `prob_abandon_lead_early` | 0.25 | Probability of abandoning unproductive leads |

## Main Planning Loop

```cpp
while (!ptc && !hasSolution()) {
    // 1. Group robots by current region
    for each robot r:
        if currentTimestep_[r] < leads_[r].size():
            currentRegion = leads_[r][currentTimestep_[r]]
            regionToRobots[currentRegion].push_back(r)

    // 2. For each region with robots, grow RRT
    for each (regionId, robotIndices) in regionToRobots:
        for exp = 0 to numRegionExpansions_:
            if robotIndices.size() == 1:
                growIndividualRRT(robotIndices[0], regionId, ...)
            else:
                growCompositeRRT(robotIndices, regionId, ...)

    // 3. Check if robots advanced to next region
    for each robot r:
        if isNearNextRegion(r, currentStates_[r]):
            currentTimestep_[r]++
}
```

## Solution Extraction

The solution extraction captures waypoints at key moments during planning:

```cpp
void extractSolution():
    for each robot r:
        path = PathGeometric(si)

        // Add all waypoints captured during planning
        for waypoint in pathWaypoints_[r]:
            path.append(waypoint)

        // Add final state if different from last waypoint
        if distance(lastWaypoint, currentState) > epsilon:
            path.append(currentState)

        pdef->addSolutionPath(path)
```

**Waypoint Capture Strategy:**
- Start state is captured when planning begins
- A waypoint is captured each time a robot advances its timestep (transitions to next region)
- Final state is added if it differs from the last waypoint

This provides a meaningful path representation showing progression through the region decomposition, with waypoints at region boundaries.

**Example Output:**
For forced coordination scenario:
- Robot 0: `[2.5,5,0]` → `[5.99,4.04,2.99]` → `[7.5,5,0]` (left to right)
- Robot 1: `[7.5,5,0]` → `[5.35,3.12,2.78]` → `[2.5,5,0]` (right to left)

**Future enhancement**: Add additional intermediate states from RRT segments for smoother paths, or implement path smoothing/interpolation.

## Coordination Strategies

### Decoupled Planning
- Each robot has independent lead
- Robots only coordinate when in same region simultaneously
- Efficient for well-separated robots

### Composite Space Planning
- Triggered when multiple robots in same region
- Handles tight coordination and collision avoidance
- More expensive but guarantees collision-free motion

## Usage Example

```cpp
#include <ompl/multirobot/geometric/planners/mrsyclop/MRSyCLoP.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>

// Create multi-robot space information
auto si = std::make_shared<ompl::multirobot::base::SpaceInformation>();
// ... add individual robot spaces to si

// Create problem definition
auto pdef = std::make_shared<ompl::multirobot::base::ProblemDefinition>(si);
// ... set start and goal states

// Create decomposition (e.g., 10x10 grid)
ompl::base::RealVectorBounds bounds(2);
bounds.setLow(-10);
bounds.setHigh(10);
auto decomp = std::make_shared<ompl::control::GridDecomposition>(
    /*length=*/10, /*dim=*/2, bounds);

// Create planner
auto planner = std::make_shared<ompl::multirobot::geometric::MRSyCLoP>(si);
planner->setProblemDefinition(pdef);
planner->setDecomposition(decomp);

// Configure parameters
planner->setNumRegionExpansions(200);
planner->setProbShortestPath(0.95);

planner->setup();

// Solve
ompl::base::PlannerStatus solved = planner->solve(30.0);  // 30 second timeout

if (solved) {
    // Extract solution paths for each robot
    for (unsigned int r = 0; r < si->getIndividualCount(); ++r) {
        auto path = pdef->getIndividual(r)->getSolutionPath();
        // ... use path
    }
}
```

## Differences from Single-Robot SyCLoP

| Aspect | Single-Robot SyCLoP | MRSyCLoP |
|--------|---------------------|----------|
| State space | Single state space | Multiple individual + composite spaces |
| Leads | One lead | One lead per robot |
| Coordination | N/A | Composite space when robots in same region |
| Timesteps | Continuous tree growth | Discrete timesteps per robot |
| Collision checking | Obstacles only | Obstacles + inter-robot collisions |

## Current Limitations and Future Work

### Current Limitations
1. **No lead recomputation**: Leads computed once at start
2. **Fixed coordination**: Always uses composite space for co-located robots
3. **No prioritization**: All robots treated equally
4. **Coarse waypoints**: Path waypoints only at region boundaries (timestep advances)

### Recent Improvements (2025-12-15)

**1. Fixed Composite Space Planning**
- **Problem**: Robots in composite space were stuck, only sampling from current region
- **Solution**: Each robot now samples from its own next region (80% probability)
- **Impact**: Composite coordination now works correctly, robots make progress toward individual goals

**2. Implemented Waypoint-Based Path Reconstruction**
- **Problem**: Paths only contained start and goal states
- **Solution**: Capture waypoints at each timestep advancement
- **Impact**: Solution paths now show meaningful progression through workspace

### Future Enhancements
1. **Denser path reconstruction**: Add intermediate states from RRT segments between waypoints
2. **Adaptive leads**: Recompute leads when stuck or environment changes
3. **Flexible coordination**: Options for prioritized planning, CBS, etc.
4. **Dynamic obstacles**: Support for moving obstacles
5. **Optimization**: Path smoothing and shortcutting
6. **Better coverage tracking**: Implement coverage grid like original SyCLoP
7. **Edge cost updates**: Implement `updateEdgeCosts()` based on coverage

## Compilation and Build

The planner automatically integrates into the OMPL build system:

```bash
cd multi-robot-syclop
mkdir build
cd build
cmake ..
make -j4
```

No CMakeLists.txt modifications needed - files are auto-discovered via `GLOB_RECURSE`.

## Debugging

The planner includes verbose logging:
- Lead computation results
- Timestep advancement
- Robot states and goal satisfaction (every 100 iterations)

Enable OMPL console output:
```cpp
ompl::msg::setLogLevel(ompl::msg::LOG_INFO);
```

## References

1. **Original SyCLoP**: E. Plaku, L.E. Kavraki, and M.Y. Vardi, "Motion Planning with Dynamics by a Synergistic Combination of Layers of Planning," IEEE Transactions on Robotics, vol. 26, no. 3, pp. 469-482, June 2010.

2. **Multi-Robot OMPL Framework**: Based on the multi-robot extensions in this codebase

3. **Composite Space Planning**: Standard OMPL compound state spaces with custom validity checking

## Contact and Contributing

For questions or contributions related to this implementation, please refer to the main project repository.

---

*Last updated: 2025-12-15*
*Implementation status: Core algorithm complete with working composite coordination and waypoint-based path reconstruction. Optimization and advanced features pending.*
