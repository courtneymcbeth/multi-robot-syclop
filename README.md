# multi-robot-syclop

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

## Running Planners

All planners use YAML files for problem specification and configuration.

### ARC (Adaptive Robot Coordination)

ARC uses a hierarchical approach to resolve conflicts between robot paths.

```bash
./build/arc -i examples/simple_2robots.yaml -o examples/output/arc_solution.yaml -c examples/planner_cfgs/arc_config.yaml -t 60
```

Arguments:
- `-i`: Input YAML file (problem specification)
- `-o`: Output YAML file (solution will be written here)
- `-c`: Configuration YAML file (optional, uses defaults if not provided)
- `-t`: Time limit in seconds (optional, default: 60)

### Other Methods

#### Multi-Robot SyCLoP
```bash
./build/mr_syclop -i examples/simple_2robots.yaml -o examples/output/mr_syclop_solution.yaml -c examples/planner_cfgs/mr_syclop_config.yaml
```

#### Coupled RRT
```bash
./build/coupled_rrt -i examples/simple_2robots.yaml -o examples/output/coupled_solution.yaml -c examples/planner_cfgs/coupled_rrt_config.yaml -t 60
```

#### Decoupled RRT
```bash
./build/decoupled_rrt -i examples/simple_2robots.yaml -o examples/output/decoupled_solution.yaml -c examples/planner_cfgs/decoupled_rrt_config.yaml -t 60
```

## Configuration Files

Configuration files are located in `examples/planner_cfgs/`:
- `arc_config.yaml`: ARC planner configuration
- `mr_syclop_config.yaml`: Multi-Robot SyCLoP configuration
- `coupled_rrt_config.yaml`: Coupled RRT configuration
- `decoupled_rrt_config.yaml`: Decoupled RRT configuration

## Relevant Files

`src/mr_syclop.cpp`: high-level implementation of SyCLoMP and its main function (builds into an executable)

`src/arc.cpp`: ARC (Adaptive Robot Coordination) planner implementation

`src/coupled_rrt.cpp`: Coupled RRT planner (plans in composite space)

`src/decoupled_rrt.cpp`: Decoupled RRT planner (prioritized planning)
