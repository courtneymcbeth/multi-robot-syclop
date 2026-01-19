# multi-robot-syclop

A collection of multi-robot motion planning algorithms with support for kinodynamic constraints and various robot types.

## Install Dependencies

```bash
git submodule update --init --recursive
```

```bash
sudo apt install libmsgpack-dev nlohmann-json3-dev
```

## Build

Building for the first time (builds dependencies):
```bash
chmod +x ./build.sh
./build.sh
```

Building after:
```bash
cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="../install" -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF && make -j$(nproc)
```

## Available Planning Methods

### ARC (Adaptive Robot Coordination)

**Type:** Geometric

**Description:** Hierarchical conflict resolution approach that computes initial decoupled paths and resolves conflicts using a 3-level hierarchy of increasingly powerful methods.

**How it works:**
1. Computes fast decoupled paths for all robots
2. Detects conflicts between paths
3. Creates local subproblems around conflicts
4. Solves subproblems using:
   - Level 1: Prioritized Query (waiting strategies, fastest)
   - Level 2: Decoupled PRM (sequential planning)
   - Level 3: Composite PRM (coupled planning, most powerful)

**Usage:**
```bash
./build/arc -i examples/simple_2robots.yaml -o examples/output/arc_solution.yaml -c examples/planner_cfgs/arc_config.yaml -t 60
```

**Configuration Options** ([arc_config.yaml](examples/planner_cfgs/arc_config.yaml)):
- `goal_threshold`: Distance threshold to consider goal reached (default: 0.5)
- `initial_time_window`: Timesteps before/after conflict to include in subproblem (default: 10)
- `time_window_expansion_step`: Timesteps to add when expanding subproblem (default: 5)
- `max_subproblem_expansions`: Maximum number of subproblem expansions (default: 5)
- `spatial_expansion_factor`: Factor to multiply environment bounds when expanding spatially (default: 1.5)
- `prioritized_query_timeout`: Timeout for Level 1 solver (default: 1.0s)
- `decoupled_prm_timeout`: Timeout for Level 2 solver (default: 5.0s)
- `composite_prm_timeout`: Timeout for Level 3 solver (default: 10.0s)
- `states_per_check`: Number of states to interpolate for collision checking (default: 10)
- `max_conflicts_resolved`: Maximum conflicts to resolve before giving up (default: 100)

### Multi-Robot SyCLoP

**Type:** Kinodynamic

**Description:** Synergistic Combination of Layers of Planning - combines discrete multi-agent path finding (MAPF) on a decomposed workspace with kinodynamic motion planning.

**How it works:**
1. Decomposes the workspace into regions
2. Uses MAPF to find high-level paths through the region graph
3. Uses kinodynamic planners to generate feasible trajectories following the MAPF solution

**Usage:**
```bash
./build/mr_syclop -i examples/simple_2robots.yaml -o examples/output/mr_syclop_solution.yaml -c examples/planner_cfgs/mr_syclop_config.yaml
```

**Configuration Options** ([mr_syclop_config.yaml](examples/planner_cfgs/mr_syclop_config.yaml)):
- `seed`: Random seed for reproducibility (default: 42, use -1 for random)
- `decomposition_region_length`: Size of each decomposition region (default: 5)
- `segment_timesteps`: Number of timesteps per path segment (default: 30)
- `mapf.method`: MAPF algorithm to use - options: `"cbs"`, `"decoupled"`, `"astar"` (default: "cbs")
- `mapf.region_capacity`: Maximum robots per region/edge (default: 2)

### Coupled RRT

**Type:** Kinodynamic

**Description:** Plans in the composite (joint) configuration space of all robots simultaneously, ensuring all paths are collision-free.

**How it works:**
- Builds a single RRT in the combined state space of all robots
- Guarantees conflict-free solutions but may be slow for many robots
- Best for 2-3 robots with complex interactions

**Usage:**
```bash
./build/coupled_rrt -i examples/simple_2robots.yaml -o examples/output/coupled_solution.yaml -c examples/planner_cfgs/coupled_rrt_config.yaml -t 60
```

**Configuration Options** ([coupled_rrt_config.yaml](examples/planner_cfgs/coupled_rrt_config.yaml)):
- `goal_threshold`: Distance threshold to consider goal reached (default: 2.0)
- `min_control_duration`: Minimum propagation steps per control (default: 1)
- `max_control_duration`: Maximum propagation steps per control (default: 10)

### Decoupled RRT (Prioritized Planning)

**Type:** Kinodynamic

**Description:** Plans robots sequentially in order of priority, with lower-priority robots treating higher-priority robots as dynamic obstacles.

**How it works:**
1. Assigns priorities to robots (based on input order)
2. Plans for each robot in sequence
3. Lower-priority robots avoid higher-priority robots' paths
4. Fast but may fail to find solutions in tightly constrained scenarios

**Usage:**
```bash
./build/decoupled_rrt -i examples/simple_2robots.yaml -o examples/output/decoupled_solution.yaml -c examples/planner_cfgs/decoupled_rrt_config.yaml -t 60
```

**Configuration Options** ([decoupled_rrt_config.yaml](examples/planner_cfgs/decoupled_rrt_config.yaml)):
- `goal_threshold`: Distance threshold to consider goal reached (default: 0.5)
- `min_control_duration`: Minimum propagation steps per control (default: 1)
- `max_control_duration`: Maximum propagation steps per control (default: 10)
- `propagation_step_size`: Time step for state propagation (default: 0.1)

### sRRT (Subdimensional Expansion RRT)

**Type:** Geometric

**Description:** Multi-robot planner that builds individual policy trees (backward RRT from goals) and uses subdimensional expansion to coordinate robots only when needed.

**Usage:**
```bash
./build/srrt -i examples/simple_2robots.yaml -o examples/output/srrt_solution.yaml -c examples/planner_cfgs/srrt_config.yaml -t 60
```

**Configuration Options** ([srrt_config.yaml](examples/planner_cfgs/srrt_config.yaml)):
- `max_distance`: Maximum distance for extending tree in one step (default: 5.0)
- `goal_threshold`: Distance threshold to consider goal reached (default: 2.0)
- `policy_timeout`: Time limit for building each policy tree (default: 5.0s)

### dRRT (Discrete RRT)

**Type:** Geometric

**Description:** Builds reusable roadmaps (SPARStwo) for each robot, then uses discrete RRT to search through the implicit tensor product of these roadmaps for multi-robot coordination.

**Usage:**
```bash
./build/drrt -i examples/simple_2robots.yaml -o examples/output/drrt_solution.yaml -c examples/planner_cfgs/drrt_config.yaml -t 60
```

**Configuration Options** ([drrt_config.yaml](examples/planner_cfgs/drrt_config.yaml)):
- `roadmap_time`: Time to spend building each robot's roadmap (default: 0.5s)
- `expansions_per_iter`: Number of tree expansion steps per iteration (default: 5)
- `spars_max_failures`: Maximum consecutive failures before termination (default: 5000)
- `spars_sparse_delta_fraction`: Visibility range for sparse graph (default: 0.25)
- `spars_dense_delta_fraction`: Interface support range for dense graph (default: 0.001)
- `spars_stretch_factor`: Path quality stretch factor (default: 3.0, range: 1.1-3.0)

## Command-Line Arguments

All planners support the following arguments:
- `-i <file>`: Input YAML file (problem specification) - **required**
- `-o <file>`: Output YAML file (solution will be written here) - **required**
- `-c <file>`: Configuration YAML file (optional, uses defaults if not provided)
- `-t <seconds>`: Time limit in seconds (optional, default varies by planner)

## Creating Environments

Environments are specified in YAML files. See [examples/simple_2robots.yaml](examples/simple_2robots.yaml) for a complete example.

### Environment Format

```yaml
environment:
  min: [x_min, y_min]          # Lower bounds of environment
  max: [x_max, y_max]          # Upper bounds of environment
  obstacles:                   # List of obstacles
    - type: box                # Obstacle type (currently only "box" supported)
      center: [x, y]           # Obstacle center position
      size: [width, height]    # Obstacle dimensions
    - type: box
      center: [x2, y2]
      size: [w2, h2]
    # Add more obstacles as needed

robots:
  - type: unicycle_first_order_0_sphere    # Robot type (see below)
    start: [x, y, theta]                    # Starting state
    goal: [x, y, theta]                     # Goal state
  - type: unicycle_first_order_0_sphere
    start: [x2, y2, theta2]
    goal: [x2_goal, y2_goal, theta2_goal]
  # Add more robots as needed
```

### Available Robot Types

Robot models are defined in [db-CBS/src/robots.cpp](db-CBS/src/robots.cpp). Available robot types:

**Unicycle Models** (state: `[x, y, theta]`):
- `unicycle_first_order_0`: First-order unicycle with box collision shape (0.5m x 0.25m)
  - Control: `[v, w]` where v ∈ [-0.5, 0.5] m/s, w ∈ [-0.5, 0.5] rad/s
- `unicycle_first_order_0_sphere`: First-order unicycle with sphere collision shape (radius 0.4m)
  - Control: `[v, w]` where v ∈ [-0.5, 0.5] m/s, w ∈ [-2.0, 2.0] rad/s
- `unicycle_second_order_0`: Second-order unicycle with velocity/acceleration dynamics
  - State: `[x, y, theta, v, w]` (5D)
  - Control: `[a, w_dot]` where a ∈ [-0.25, 0.25] m/s², w_dot ∈ [-0.25, 0.25] rad/s²

**Car Models**:
- `car_first_order_0`: Simple car model with Ackermann steering
  - State: `[x, y, theta]`
  - Control: `[v, phi]` where v ∈ [-0.5, 0.5] m/s, phi ∈ [-π/3, π/3] rad
  - Length: L = 0.25m
- `car_first_order_with_1_trailers_0`: Car with one trailer
  - State: `[x, y, theta_car, theta_trailer]` (4D)
  - Control: `[v, phi]` where v ∈ [-0.1, 0.5] m/s, phi ∈ [-π/3, π/3] rad
  - Car length: L = 0.25m, hitch length = 0.5m

**Integrator Models**:
- `single_integrator_0`: First-order 2D point robot (sphere, radius 0.1m)
  - State: `[x, y]`
  - Control: `[vx, vy]` where vx, vy ∈ [-0.5, 0.5] m/s
- `double_integrator_0`: Second-order 2D point robot (sphere, radius 0.15m)
  - State: `[x, y, vx, vy]` (4D)
  - Control: `[ax, ay]` where ax, ay ∈ [-2.0, 2.0] m/s²

### State Dimensions by Robot Type

- **First-order unicycle/car**: 3D state `[x, y, theta]`
- **Second-order unicycle**: 5D state `[x, y, theta, v, w]`
- **Car with 1 trailer**: 4D state `[x, y, theta_car, theta_trailer]`
- **Single integrator**: 2D state `[x, y]`
- **Double integrator**: 4D state `[x, y, vx, vy]`

### Adding Obstacles

Obstacles are defined as boxes with a center position and size:

```yaml
obstacles:
  - type: box
    center: [5.0, 5.0]     # Center at (5, 5)
    size: [2.0, 1.0]       # Width=2.0, Height=1.0
  - type: box
    center: [10.0, 15.0]
    size: [3.0, 3.0]
```

### Environment Tips

- Start with simple scenarios (2-3 robots, few obstacles)
- Ensure start and goal states are collision-free
- For unicycle robots, make sure the orientation (theta) is in radians
- Keep environment bounds reasonable for the robot capabilities
- Test individual robot paths before running multi-robot scenarios

## Visualization

Python visualization scripts are available in [examples/](examples/) to visualize planning results.

### Installation

Install required Python packages:
```bash
pip install matplotlib pyyaml
```

### Available Visualization Tools

#### 1. Basic Solution Visualization

Visualize a computed solution with static plot or animation:

```bash
# Static plot
python examples/visualize_solution.py --env examples/simple_2robots.yaml --solution examples/output/arc_solution.yaml

# Save static plot to file
python examples/visualize_solution.py --env examples/simple_2robots.yaml --solution examples/output/arc_solution.yaml --output solution.png

# Animated visualization
python examples/visualize_solution.py --env examples/simple_2robots.yaml --solution examples/output/arc_solution.yaml --animate

# Save animation to file
python examples/visualize_solution.py --env examples/simple_2robots.yaml --solution examples/output/arc_solution.yaml --video solution.gif
```

**Options:**
- `--env`: Environment YAML file (required)
- `--solution`: Solution YAML file (required)
- `--output`: Save static plot to image file (e.g., `.png`, `.pdf`)
- `--animate`: Show animated visualization
- `--video`: Save animation to file (e.g., `.gif`, `.mp4`)
- `--speed`: Robot speed for animation in map units/second (default: 1.0)
- `--fps`: Animation frames per second (default: 20)

#### 2. Interactive Visualization

Interactive visualization with playback controls (play/pause and timeline slider):

```bash
python examples/visualize_interactive.py --env examples/simple_2robots.yaml --solution examples/output/arc_solution.yaml
```

**Options:**
- `--env`: Environment YAML file (required)
- `--solution`: Solution YAML file (required)
- `--speed`: Robot speed in map units/second (default: 1.0)

**Controls:**
- Play/Pause button to control animation
- Timeline slider to seek through the solution
- Real-time display of collision status

#### 3. Planning Stages Visualization

Visualize the multi-stage planning pipeline (MR-SyCLoP only):

```bash
python examples/visualize_planning_stages.py --env examples/simple_2robots.yaml --solution examples/output/mr_syclop_solution.yaml
```

**Options:**
- `--env`: Environment YAML file (required)
- `--solution`: Solution YAML file with debug data (required)

**Shows:**
1. Workspace decomposition and high-level MAPF paths
2. Low-level guided kinodynamic paths
3. Path segmentation
4. Collision detection results
5. Final solution

Use navigation buttons to step through planning stages.

## Running Experiments

The `experiments/` directory contains a Python framework for systematic benchmarking of all planners.

### Quick Start

```bash
cd experiments/

# Generate test scenarios
python3 generate_scenarios.py --type empty --size 20 20 --name open_20x20
python3 generate_scenarios.py --type corridor --width 40 --height 10 --name corridor_40x10
python3 generate_scenarios.py --type random --size 30 30 --density 0.15 --name cluttered_30x30

# Run experiments (adaptive scaling - increases robots until all seeds fail)
python3 run_experiments.py

# Or with custom parameters
python3 run_experiments.py --seeds 5 --timeout 120 --scenarios open_20x20 --planners arc mr_syclop
```

### Experiment Framework Components

| Script | Purpose |
|--------|---------|
| `generate_scenarios.py` | Create test environments (empty, random, corridor, grid, rooms) |
| `visualize_problem.py` | Debug visualization for environments and solutions |
| `run_experiments.py` | Adaptive batch runner with seed control |
| `parse_results.py` | Extract metrics from planner output YAML |
| `aggregate_results.py` | Compute statistics across seeds |
| `plot_results.py` | Generate comparison plots |

### Creating Scenarios

```bash
# Empty environment
python3 generate_scenarios.py --type empty --size 20 20 --name my_empty

# Random obstacles (15% coverage)
python3 generate_scenarios.py --type random --size 30 30 --density 0.15 --name my_random

# Corridor with gaps
python3 generate_scenarios.py --type corridor --width 40 --height 10 --corridor-width 4 --name my_corridor

# Grid pattern
python3 generate_scenarios.py --type grid --size 20 20 --grid-size 4 --name my_grid

# Rooms with doors
python3 generate_scenarios.py --type rooms --size 20 20 --rooms-x 2 --rooms-y 2 --name my_rooms

# List all scenario types
python3 generate_scenarios.py --list-types
```

### Visualizing Problems

```bash
# View a scenario (environment only)
python3 visualize_problem.py scenarios/open_20x20.yaml

# View a problem instance (environment + robot positions)
python3 visualize_problem.py problems/open_20x20/robots_4/seed_0.yaml

# View a solution
python3 visualize_problem.py problems/open_20x20/robots_4/seed_0.yaml --solution results/open_20x20/arc/robots_4/seed_0.yaml

# Save to file
python3 visualize_problem.py scenarios/corridor_40x10.yaml -o corridor.png
```

### Configuration

Edit `experiments/config.yaml` to configure experiments:

```yaml
# Experiment parameters
num_seeds: 10           # Random seeds per configuration
timeout: 300            # Time limit per run (seconds)
start_robots: 2         # Starting number of robots
robot_increment: 2      # Increase by 2 each iteration
robot_type: unicycle_first_order_0_sphere

# Scenarios to run
scenarios:
  - open_20x20
  - corridor_40x10

# Planners to test
planners:
  arc:
    executable: ../build/arc
    config: ../examples/planner_cfgs/arc_config.yaml
  mr_syclop:
    executable: ../build/mr_syclop
    config: ../examples/planner_cfgs/mr_syclop_config.yaml
  # ... more planners
```

### Adaptive Scaling

The experiment runner uses adaptive scaling:
1. Starts with 2 robots
2. Runs all seeds (default 10)
3. If **any seed succeeds**, increases to 4 robots
4. Continues until **all seeds fail** at some robot count
5. Stops testing that (scenario, planner) pair

This saves computation time by not testing configurations that are clearly beyond a planner's capability.

### Analyzing Results

```bash
# Parse raw results into CSV
python3 parse_results.py results/ -o analysis/results.csv

# Aggregate statistics across seeds
python3 aggregate_results.py results/ -o analysis/summary.csv

# Print summary table
python3 aggregate_results.py results/

# Generate all plots
python3 plot_results.py analysis/summary.csv -o analysis/plots/
```

### Output Structure

```
experiments/
├── scenarios/           # Base environments (no robots)
│   ├── open_20x20.yaml
│   └── corridor_40x10.yaml
├── problems/            # Generated problem instances
│   └── {scenario}/robots_{N}/seed_{S}.yaml
├── results/             # Planner outputs
│   └── {scenario}/{planner}/robots_{N}/seed_{S}.yaml
├── analysis/            # Aggregated results and plots
│   ├── summary.csv
│   └── plots/
└── configs/             # Per-seed planner configs
    └── {planner}/seed_{S}.yaml
```

### Seed Control

All planners support deterministic execution via the `seed` parameter in their config files:
- `seed: 42` - Use seed 42 for reproducible results
- `seed: -1` - Use random seed

The experiment runner automatically injects seeds into planner configs to ensure reproducibility.

## Source Files

- [src/arc.cpp](src/arc.cpp): ARC planner implementation
- [src/mr_syclop.cpp](src/mr_syclop.cpp): Multi-Robot SyCLoP implementation
- [src/coupled_rrt.cpp](src/coupled_rrt.cpp): Coupled RRT planner
- [src/decoupled_rrt.cpp](src/decoupled_rrt.cpp): Decoupled RRT planner (prioritized planning)
- [src/srrt.cpp](src/srrt.cpp): sRRT planner implementation
- [src/drrt.cpp](src/drrt.cpp): dRRT planner implementation
- [src/decomposition.cpp](src/decomposition.cpp): Workspace decomposition
- [src/mapf/](src/mapf/): Multi-agent path finding solvers
- [src/guided/](src/guided/): Guided kinodynamic planners
- [experiments/](experiments/): Experiment framework for benchmarking
