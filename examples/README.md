# Multi-Robot SyCLoP Examples

This directory contains example scenarios and utilities for running Multi-Robot SyCLoP.

## Quick Start

### 1. Build the Project

```bash
cd /home/courtney/multi-robot-syclop
mkdir -p build
cd build
cmake ..
cmake --build .
```

### 2. Run Examples

```bash
cd /home/courtney/multi-robot-syclop/examples
./run_example.sh
```

This will run two example scenarios:
- **2 Robots Swap**: Two robots exchange positions
- **3 Robots Corridor**: Three robots coordinate in tight spaces

### 3. Visualize Results (Optional)

Install Python dependencies:
```bash
pip install matplotlib pyyaml
```

Visualize a solution:
```bash
./visualize_solution.py \
    --env simple_2robots.yaml \
    --solution output/simple_2robots_solution.yaml \
    --output output/simple_2robots_viz.png
```

Create an animation:
```bash
./visualize_solution.py \
    --env simple_2robots.yaml \
    --solution output/simple_2robots_solution.yaml \
    --animate
```

Save animation to file:
```bash
./visualize_solution.py \
    --env simple_2robots.yaml \
    --solution output/simple_2robots_solution.yaml \
    --video output/simple_2robots.gif
```

## File Formats

### Environment File (YAML)

Defines the workspace and robot start/goal positions:

```yaml
environment:
  min: [0.0, 0.0]  # [x_min, y_min]
  max: [10.0, 10.0]  # [x_max, y_max]

robots:
  - name: robot0
    start: [2.0, 2.0, 0.0]  # [x, y, theta]
    goal: [8.0, 8.0, 0.0]

  - name: robot1
    start: [8.0, 8.0, 0.0]
    goal: [2.0, 2.0, 0.0]
```

### Configuration File (YAML)

Planner parameters:

```yaml
goal_epsilon: 0.5              # Goal tolerance
grid_resolution: 16            # Grid size (16x16)
num_region_expansions: 10      # RRT expansions per region
num_tree_selections: 1         # Tree selections per expansion
prob_shortest_path: 0.95       # Probability of using A* vs random
```

### Solution File (YAML)

Output format with paths for each robot:

```yaml
result:
  - states:
    - [2.0, 2.0, 0.0]  # Waypoint 1
    - [3.5, 3.5, 0.785]  # Waypoint 2
    - [8.0, 8.0, 0.0]  # Goal
  - states:
    - [8.0, 8.0, 0.0]
    - [6.5, 6.5, 3.927]
    - [2.0, 2.0, 0.0]
```

## Running Custom Scenarios

### Manual Execution

```bash
cd /home/courtney/multi-robot-syclop

./build/main_mr_syclop \
    --input examples/simple_2robots.yaml \
    --output examples/output/my_solution.yaml \
    --stats examples/output/my_stats.yaml \
    --cfg examples/planner_config.yaml \
    --planner mr-syclop \
    --timelimit 60
```

### Command-Line Options

- `--input, -i`: Input environment file (required)
- `--output, -o`: Output solution file (required)
- `--cfg, -c`: Configuration file (required)
- `--stats`: Statistics output file (default: ompl_stats.yaml)
- `--planner, -p`: Planner name (default: mr-syclop)
- `--timelimit`: Time limit in seconds (default: 60)
- `--help`: Show help message

## Example Scenarios

### 1. Simple 2-Robot Swap ([simple_2robots.yaml](simple_2robots.yaml))

Two robots need to swap positions in a 10x10 environment. This tests basic coupling/decoupling as robots pass near each other.

### 2. 3-Robot Corridor ([3robots_corridor.yaml](3robots_corridor.yaml))

Three robots must coordinate in tight spaces. Robot 0 and 2 move horizontally in opposite directions, while Robot 1 moves vertically, creating a coordination challenge at the intersection.

## Creating Your Own Scenarios

1. Create a new environment YAML file:

```yaml
environment:
  min: [0.0, 0.0]
  max: [20.0, 20.0]

robots:
  - name: robot0
    start: [1.0, 1.0, 0.0]
    goal: [19.0, 19.0, 0.0]

  - name: robot1
    start: [19.0, 1.0, 1.57]
    goal: [1.0, 19.0, 1.57]
```

2. Run the planner:

```bash
./build/main_mr_syclop \
    --input my_scenario.yaml \
    --output my_solution.yaml \
    --cfg examples/planner_config.yaml \
    --timelimit 120
```

3. Visualize:

```bash
./examples/visualize_solution.py \
    --env my_scenario.yaml \
    --solution my_solution.yaml \
    --animate
```

## Tuning Parameters

### Grid Resolution
- **Lower values** (8-16): Faster planning, coarser paths
- **Higher values** (32-64): Slower planning, finer paths
- Start with 16 and increase if paths are too coarse

### Number of Region Expansions
- Controls how many RRT samples per region
- **Lower values** (5-10): Faster but may miss solutions
- **Higher values** (20-50): More thorough but slower

### Probability of Shortest Path
- Controls exploration vs exploitation
- **High values** (0.9-0.99): Follow A* lead closely
- **Lower values** (0.5-0.8): More random exploration

### Goal Epsilon
- Tolerance for goal satisfaction
- SE2 distance: `sqrt(dx² + dy²) + |dtheta|`
- Increase if goals are hard to reach exactly

## Troubleshooting

### "No solution found within time limit"
- Increase `--timelimit`
- Increase `num_region_expansions`
- Check that start/goal regions are reachable
- Verify environment bounds are large enough

### "Failed to compute lead for robot"
- Start or goal is outside environment bounds
- No path exists in the region graph
- Increase `grid_resolution` for finer decomposition

### Robots collide in solution
- Inter-robot collision checking needs implementation
- See [IMPLEMENTATION_NOTES.md](../IMPLEMENTATION_NOTES.md) for details
- Current implementation uses placeholder collision checking

## Output Files

### Solution File
- Contains waypoints for each robot
- Format: `[x, y, theta]` per waypoint
- Can be visualized with `visualize_solution.py`

### Statistics File
- Planning time
- Success status
- Can be extended with more metrics

## Next Steps

1. **Implement full inter-robot collision checking**: See [IMPLEMENTATION_NOTES.md](../IMPLEMENTATION_NOTES.md) Section "Inter-Robot Collision Checking"

2. **Improve path extraction**: Currently simplified, see "Path Synchronization" section

3. **Add obstacles**: Extend environment format and state validity checker

4. **Optimize parameters**: Tune for your specific scenarios

## References

- Original SyCLoP paper: Plaku et al., "Motion Planning with Dynamics by a Synergistic Combination of Layers of Planning", IEEE TRO 2010
- OMPL: [https://ompl.kavrakilab.org/](https://ompl.kavrakilab.org/)
- db-CBS reference: [https://github.com/IMRCLab/db-CBS](https://github.com/IMRCLab/db-CBS)
