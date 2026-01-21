#!/usr/bin/env python3
"""
Interactive visualization script for Multi-Robot SyCLoP solutions with slider control
Features: slider for seeking through the animation timeline
Supports: mr_syclop format and dynobench/DB-RRT format
Requires: matplotlib, pyyaml
Install: pip install matplotlib pyyaml
"""

import bisect
import yaml
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Slider, Button
import numpy as np
import argparse
import sys
import time
import os

# Default path to dynobench models (relative to this script or absolute)
DEFAULT_MODELS_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'db-CBS', 'dynoplan', 'dynobench', 'models'
)

DEFAULT_GEOMETRIES = {
    'single_integrator_0': [('sphere', 0.1)],
    'double_integrator_0': [('sphere', 0.15)],
    'unicycle_first_order_0': [('box', 0.5, 0.25)],
    'unicycle_first_order_0_sphere': [('sphere', 0.4)],
    'unicycle_second_order_0': [('box', 0.5, 0.25)],
    'car_first_order_0': [('box', 0.5, 0.25)],
    'car_first_order_with_1_trailers_0': [
        ('box', 0.5, 0.25),
        ('box', 0.3, 0.25),
    ],
}

def load_yaml(filename, exit_on_error=True):
    """Load YAML file"""
    try:
        with open(filename, 'r') as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        if exit_on_error:
            print(f"Error: File '{filename}' not found")
            sys.exit(1)
        return None
    except yaml.YAMLError as e:
        if exit_on_error:
            print(f"Error parsing YAML: {e}")
            sys.exit(1)
        return None


def load_robot_model(robot_type, models_path=None):
    """Load robot geometry from dynobench model file.

    Args:
        robot_type: Robot type string (e.g., 'unicycle_first_order_0')
        models_path: Path to dynobench models directory

    Returns:
        dict with 'shape', 'size', 'radius' fields, or None if not found
    """
    if models_path is None:
        models_path = DEFAULT_MODELS_PATH

    model_file = os.path.join(models_path, f'{robot_type}.yaml')
    model = load_yaml(model_file, exit_on_error=False)

    if model is None:
        return None

    return {
        'shape': model.get('shape', 'sphere'),
        'size': model.get('size', [0.5, 0.25]),
        'radius': model.get('radius', None),
    }


def detect_config_format(config):
    """Detect whether config is in mr_syclop or s2m2/DB-RRT format.

    Returns: 'mr_syclop', 's2m2', or 'unknown'
    """
    # s2m2/DB-RRT format has 'agents' and 'limits'
    if 'agents' in config and 'limits' in config:
        return 's2m2'
    # mr_syclop format has 'environment' and 'robots'
    if 'environment' in config or 'robots' in config:
        return 'mr_syclop'
    return 'unknown'


def halfplanes_to_box(halfplanes):
    """Convert a list of half-plane constraints to a bounding box.

    Each half-plane is [normal_x, normal_y, offset] defining constraint:
    normal_x * x + normal_y * y <= offset

    Returns: (center_x, center_y, width, height) or None if invalid
    """
    # Extract bounds from half-planes
    x_min, x_max = float('-inf'), float('inf')
    y_min, y_max = float('-inf'), float('inf')

    for hp in halfplanes:
        if len(hp) < 3:
            continue
        nx, ny, d = hp[0], hp[1], hp[2]

        # Normalize to identify axis-aligned planes
        if abs(ny) < 1e-6:  # Vertical plane (x constraint)
            if nx > 0:  # x <= d/nx
                x_max = min(x_max, d / nx)
            else:  # x >= d/nx (since nx < 0: nx*x <= d means x >= d/nx)
                x_min = max(x_min, d / nx)
        elif abs(nx) < 1e-6:  # Horizontal plane (y constraint)
            if ny > 0:  # y <= d/ny
                y_max = min(y_max, d / ny)
            else:  # y >= d/ny
                y_min = max(y_min, d / ny)

    # Check for valid box
    if x_min == float('-inf') or x_max == float('inf'):
        return None
    if y_min == float('-inf') or y_max == float('inf'):
        return None
    if x_min >= x_max or y_min >= y_max:
        return None

    center_x = (x_min + x_max) / 2
    center_y = (y_min + y_max) / 2
    width = x_max - x_min
    height = y_max - y_min

    return (center_x, center_y, width, height)


def convert_s2m2_to_mrsyclop(config, models_path=None):
    """Convert s2m2/DB-RRT config format to mr_syclop format.

    Args:
        config: s2m2 format config dict
        models_path: Optional path to dynobench models

    Returns:
        mr_syclop format config dict
    """
    env = {'environment': {}, 'robots': []}

    # Convert limits to environment bounds
    limits = config.get('limits', [[0, 10], [0, 10]])
    env['environment']['min'] = [limits[0][0], limits[1][0]]
    env['environment']['max'] = [limits[0][1], limits[1][1]]

    # Convert obstacles from half-planes to boxes
    env['environment']['obstacles'] = []
    for obs_halfplanes in config.get('obstacles', []):
        box = halfplanes_to_box(obs_halfplanes)
        if box is not None:
            center_x, center_y, width, height = box
            env['environment']['obstacles'].append({
                'type': 'box',
                'center': [center_x, center_y],
                'size': [width, height],
            })

    # Convert agents to robots
    agents = config.get('agents', [])
    starts = config.get('starts', [])
    goals = config.get('goals', [])

    for i, agent in enumerate(agents):
        robot_type = agent.get('type', 'unicycle_first_order_0')
        agent_size = agent.get('size', None)

        # Try to load model for geometry
        model = load_robot_model(robot_type, models_path)

        robot_config = {
            'type': robot_type,
            'name': f'Robot {i}',
        }

        # Determine radius from: agent.size > model.radius > model.size > default
        if agent_size is not None:
            robot_config['radius'] = agent_size
        elif model is not None:
            if model['radius'] is not None:
                robot_config['radius'] = model['radius']
            elif model['shape'] == 'sphere' and model['size']:
                robot_config['radius'] = model['size'][0] if isinstance(model['size'], list) else model['size']
            elif model['shape'] == 'box' and model['size']:
                # Use half-diagonal as effective radius for collision
                w, h = model['size'][0], model['size'][1]
                robot_config['radius'] = 0.5 * np.hypot(w, h)
                robot_config['box_size'] = model['size']
            robot_config['shape'] = model['shape']

        # Get start position
        if i < len(starts):
            start = starts[i]
            robot_config['start'] = list(start) + [0.0] * (3 - len(start))
        else:
            robot_config['start'] = [0.0, 0.0, 0.0]

        # Get goal position (goals is a list of goal configurations per robot)
        # Each goal config is a list of half-planes; extract center point
        if i < len(goals) and goals[i]:
            # Goals are half-planes, convert to point
            goal_box = halfplanes_to_box(goals[i])
            if goal_box is not None:
                robot_config['goal'] = [goal_box[0], goal_box[1], 0.0]
            else:
                robot_config['goal'] = [1.0, 1.0, 0.0]
        else:
            robot_config['goal'] = [1.0, 1.0, 0.0]

        env['robots'].append(robot_config)

    return env


def detect_format(solution):
    """Detect whether solution is in mr_syclop or dynobench/DB-RRT format.

    Returns: 'mr_syclop' or 'dbrrt'
    """
    # DB-RRT format has 'solved' field (integer 0/1)
    if 'solved' in solution:
        return 'dbrrt'
    # mr_syclop format has 'success' field (boolean)
    if 'success' in solution:
        return 'mr_syclop'
    # Fallback: if result exists, assume mr_syclop
    if 'result' in solution:
        return 'mr_syclop'
    return 'unknown'


def extract_paths_from_solution(solution):
    """Extract paths from solution regardless of format.

    Returns: list of paths, where each path is a list of [x, y, ...] states
    """
    paths = []

    if 'result' not in solution:
        return paths

    result = solution['result']

    # Handle list of robot trajectories
    if isinstance(result, list):
        for item in result:
            if isinstance(item, dict) and 'states' in item:
                # Format: [{states: [...]}, {states: [...]}]
                paths.append(item['states'])
            elif isinstance(item, list):
                # Format: [[[x,y,theta], ...], [[x,y,theta], ...]]
                # This could be a single trajectory as list of states
                if item and isinstance(item[0], (list, tuple)) and len(item[0]) >= 2:
                    paths.append(item)
    elif isinstance(result, dict) and 'states' in result:
        # Single robot: {states: [...]}
        paths.append(result['states'])

    return paths


def compute_bounds_from_paths(paths, margin=1.0):
    """Compute environment bounds from trajectory data.

    Returns: (env_min, env_max) as [x, y] lists
    """
    if not paths:
        return [0.0, 0.0], [10.0, 10.0]

    all_x = []
    all_y = []

    for path in paths:
        for state in path:
            if len(state) >= 2:
                all_x.append(state[0])
                all_y.append(state[1])

    if not all_x:
        return [0.0, 0.0], [10.0, 10.0]

    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)

    # Add margin
    env_min = [min_x - margin, min_y - margin]
    env_max = [max_x + margin, max_y + margin]

    return env_min, env_max


def create_synthetic_env(solution, paths, models_path=None):
    """Create a synthetic environment dict from DB-RRT solution data.

    This allows visualization without a separate environment file.
    Uses dynobench model files when available for accurate robot geometry.
    """
    env = {'environment': {}, 'robots': []}

    # Compute bounds from trajectories
    env_min, env_max = compute_bounds_from_paths(paths, margin=1.0)
    env['environment']['min'] = env_min
    env['environment']['max'] = env_max
    env['environment']['obstacles'] = []

    # Extract start/goal from solution if available, otherwise from paths
    start = solution.get('start', None)
    goal = solution.get('goal', None)
    robot_type = solution.get('robotType', 'unicycle_first_order_0_sphere')

    # Try to load robot model for geometry
    model = load_robot_model(robot_type, models_path)

    for i, path in enumerate(paths):
        robot_config = {
            'type': robot_type,
            'name': f'Robot {i}',
        }

        # Determine radius from model or use default
        if model is not None:
            if model['radius'] is not None:
                robot_config['radius'] = model['radius']
            elif model['shape'] == 'sphere' and model['size']:
                robot_config['radius'] = model['size'][0] if isinstance(model['size'], list) else model['size']
            elif model['shape'] == 'box' and model['size']:
                w, h = model['size'][0], model['size'][1]
                robot_config['radius'] = 0.5 * np.hypot(w, h)
                robot_config['box_size'] = model['size']
            robot_config['shape'] = model['shape']
        else:
            # Fallback to DEFAULT_GEOMETRIES or hardcoded default
            robot_config['radius'] = 0.3

        # Get start from solution or first path state
        if start is not None and len(start) >= 2:
            robot_config['start'] = list(start)
        elif path and len(path[0]) >= 2:
            robot_config['start'] = list(path[0])
        else:
            robot_config['start'] = [0.0, 0.0, 0.0]

        # Get goal from solution or last path state
        if goal is not None and len(goal) >= 2:
            robot_config['goal'] = list(goal)
        elif path and len(path[-1]) >= 2:
            robot_config['goal'] = list(path[-1])
        else:
            robot_config['goal'] = [1.0, 1.0, 0.0]

        env['robots'].append(robot_config)

    return env

def get_robot_radius(robot_config):
    if 'radius' in robot_config:
        return robot_config['radius']
    robot_type = robot_config.get('type', '')
    if robot_type in DEFAULT_GEOMETRIES:
        max_radius = 0.0
        for spec in DEFAULT_GEOMETRIES[robot_type]:
            if spec[0] == 'sphere':
                radius = spec[1]
            else:
                radius = 0.5 * float(np.hypot(spec[1], spec[2]))
            if radius > max_radius:
                max_radius = radius
        if max_radius > 0.0:
            return max_radius
    return 0.5

def draw_obstacles(ax, env):
    obstacles = env.get('environment', {}).get('obstacles', [])
    for obs in obstacles:
        obs_type = obs.get('type', '')
        if obs_type == 'box':
            size = obs.get('size', [0.0, 0.0])
            center = obs.get('center', [0.0, 0.0])
            width = float(size[0])
            height = float(size[1])
            x = float(center[0]) - width / 2.0
            y = float(center[1]) - height / 2.0
            rect = patches.Rectangle(
                (x, y),
                width,
                height,
                facecolor='dimgray',
                edgecolor='black',
                linewidth=1.5,
                alpha=0.6,
                zorder=1,
            )
            ax.add_patch(rect)
        else:
            print(f"Warning: Unsupported obstacle type '{obs_type}' in visualization")

def compute_segment_max_distances(paths, num_segments):
    max_distances = [0.0] * num_segments
    for path in paths:
        if len(path) < 2:
            continue
        limit = min(len(path) - 1, num_segments)
        for i in range(limit):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            dist = float(np.hypot(dx, dy))
            if dist > max_distances[i]:
                max_distances[i] = dist
    return max_distances

def build_waypoint_times(segment_max_distances, speed):
    waypoint_times = [0.0]
    for dist in segment_max_distances:
        if dist <= 1e-9:
            waypoint_times.append(waypoint_times[-1])
        else:
            waypoint_times.append(waypoint_times[-1] + dist / speed)
    return waypoint_times

def interpolate_at_time(path, waypoint_times, t):
    if not path:
        return None
    if t <= 0.0:
        return path[0]
    if t >= waypoint_times[-1]:
        return path[-1]
    end_times = waypoint_times[1:]
    seg_idx = bisect.bisect_right(end_times, t)
    if seg_idx >= len(path) - 1:
        return path[-1]
    start_time = waypoint_times[seg_idx]
    end_time = waypoint_times[seg_idx + 1]
    duration = end_time - start_time
    if duration <= 1e-9:
        return path[seg_idx]
    ratio = (t - start_time) / duration
    ratio = min(max(ratio, 0.0), 1.0)
    x = path[seg_idx][0] + ratio * (path[seg_idx + 1][0] - path[seg_idx][0])
    y = path[seg_idx][1] + ratio * (path[seg_idx + 1][1] - path[seg_idx][1])
    return [x, y]

def trail_points(path, waypoint_times, t):
    if not path:
        return [], []
    if t <= 0.0:
        return [path[0][0]], [path[0][1]]
    if t >= waypoint_times[-1]:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        return xs, ys
    end_times = waypoint_times[1:]
    seg_idx = bisect.bisect_right(end_times, t)
    last_idx = min(seg_idx, len(path) - 1)
    xs = [p[0] for p in path[:last_idx + 1]]
    ys = [p[1] for p in path[:last_idx + 1]]
    if seg_idx < len(path) - 1:
        start_time = waypoint_times[seg_idx]
        end_time = waypoint_times[seg_idx + 1]
        duration = end_time - start_time
        if duration > 1e-9:
            ratio = (t - start_time) / duration
            ratio = min(max(ratio, 0.0), 1.0)
            interp_x = path[seg_idx][0] + ratio * (path[seg_idx + 1][0] - path[seg_idx][0])
            interp_y = path[seg_idx][1] + ratio * (path[seg_idx + 1][1] - path[seg_idx][1])
            if not xs or interp_x != xs[-1] or interp_y != ys[-1]:
                xs.append(interp_x)
                ys.append(interp_y)
    return xs, ys

def interactive_animation(env_file, solution_file, initial_speed=1.0, config_file=None,
                          models_path=None, show_high_level=True, show_grid=True,
                          planner_config_file=None):
    """Create an interactive visualization with a slider control

    Args:
        env_file: Path to environment YAML file (optional, can be None for DB-RRT format)
        solution_file: Path to solution YAML file
        initial_speed: Robot speed in map units per second
        config_file: Path to problem config file (s2m2 or mr_syclop format)
        models_path: Path to dynobench models directory
        show_high_level: Whether to display high-level paths through regions
        show_grid: Whether to display decomposition grid lines
        planner_config_file: Path to planner config file (for decomposition.resolution)
    """

    # Load solution
    solution = load_yaml(solution_file)

    if not solution or 'result' not in solution:
        print("Warning: No solution found in file")
        return

    # Detect format
    fmt = detect_format(solution)
    print(f"Detected solution format: {fmt}")

    # Load planner config if provided (for decomposition.resolution)
    planner_config = None
    if planner_config_file:
        planner_config = load_yaml(planner_config_file, exit_on_error=False)
        if planner_config:
            print(f"Loaded planner config: {planner_config_file}")

    # Extract paths using unified function
    paths = extract_paths_from_solution(solution)

    if not paths:
        print("Warning: No valid paths found in solution")
        return

    # Load environment from config file, env file, or create synthetic
    env = None

    # Priority 1: Use config file if provided
    if config_file:
        config = load_yaml(config_file)
        config_fmt = detect_config_format(config)
        print(f"Loaded config file (format: {config_fmt}): {config_file}")

        if config_fmt == 's2m2':
            env = convert_s2m2_to_mrsyclop(config, models_path)
        elif config_fmt == 'mr_syclop':
            env = config
            # Augment with model data if robots don't have explicit radius
            for robot in env.get('robots', []):
                if 'radius' not in robot:
                    robot_type = robot.get('type', '')
                    model = load_robot_model(robot_type, models_path)
                    if model is not None:
                        if model['radius'] is not None:
                            robot['radius'] = model['radius']
                        elif model['shape'] == 'sphere' and model['size']:
                            robot['radius'] = model['size'][0] if isinstance(model['size'], list) else model['size']
                        elif model['shape'] == 'box' and model['size']:
                            w, h = model['size'][0], model['size'][1]
                            robot['radius'] = 0.5 * np.hypot(w, h)
                            robot['box_size'] = model['size']
                        robot['shape'] = model['shape']

    # Priority 2: Use env file if provided
    if env is None and env_file:
        env = load_yaml(env_file)
        # Augment with model data
        for robot in env.get('robots', []):
            if 'radius' not in robot:
                robot_type = robot.get('type', '')
                model = load_robot_model(robot_type, models_path)
                if model is not None:
                    if model['radius'] is not None:
                        robot['radius'] = model['radius']
                    elif model['shape'] == 'sphere' and model['size']:
                        robot['radius'] = model['size'][0] if isinstance(model['size'], list) else model['size']
                    elif model['shape'] == 'box' and model['size']:
                        w, h = model['size'][0], model['size'][1]
                        robot['radius'] = 0.5 * np.hypot(w, h)
                        robot['box_size'] = model['size']
                    robot['shape'] = model['shape']

    # Priority 3: Create synthetic environment from solution data
    if env is None:
        print("No environment/config file provided, inferring from solution...")
        env = create_synthetic_env(solution, paths, models_path)

    # Extract environment bounds
    env_min = env['environment']['min']
    env_max = env['environment']['max']

    if initial_speed <= 0.0:
        initial_speed = 1.0

    max_points = max((len(path) for path in paths if path), default=0)
    if max_points < 2:
        print("No valid paths to animate")
        return
    segment_max_distances = compute_segment_max_distances(paths, max_points - 1)
    waypoint_times = build_waypoint_times(segment_max_distances, initial_speed)
    max_time = waypoint_times[-1]

    if max_time <= 0.0:
        print("No valid paths to animate")
        return

    # Create figure with space for slider at the bottom
    fig = plt.figure(figsize=(12, 10))

    # Main plot area
    ax = plt.axes([0.1, 0.2, 0.8, 0.7])
    ax.set_xlim(env_min[0], env_max[0])
    ax.set_ylim(env_min[1], env_max[1])
    ax.set_aspect('equal')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Multi-Robot SyCLoP Interactive Visualization')

    draw_obstacles(ax, env)

    # Draw regions if decomposition data is available
    # Store high-level path artists for legend
    high_level_path_artists = []

    # Determine decomposition info from solution or planner config
    decomp = None
    grid_size = None
    bounds_min = None
    bounds_max = None
    cell_width = None
    cell_height = None

    # Priority 1: Use planner config's decomposition.resolution if provided (overrides solution)
    if planner_config and 'decomposition' in planner_config:
        planner_decomp = planner_config['decomposition']
        if planner_decomp.get('type') == 'grid' and 'resolution' in planner_decomp:
            resolution = planner_decomp['resolution']
            grid_size = [resolution[0], resolution[1]]  # [x, y] from resolution
            bounds_min = env_min
            bounds_max = env_max
            cell_width = (bounds_max[0] - bounds_min[0]) / grid_size[0]
            cell_height = (bounds_max[1] - bounds_min[1]) / grid_size[1]
            # Use solution's leads if available for high-level path visualization
            if 'decomposition' in solution and 'leads' in solution['decomposition']:
                decomp = {'type': 'grid', 'leads': solution['decomposition']['leads']}
            else:
                decomp = {'type': 'grid'}
            print(f"Using decomposition resolution={resolution[:2]} from planner config -> grid_size={grid_size}")
    # Priority 2: Use solution's decomposition if no planner config provided
    elif 'decomposition' in solution:
        decomp = solution['decomposition']
        if decomp.get('type') == 'grid':
            grid_size = decomp['grid_size']
            bounds_min = decomp['bounds']['min']
            bounds_max = decomp['bounds']['max']
            cell_width = (bounds_max[0] - bounds_min[0]) / grid_size[0]
            cell_height = (bounds_max[1] - bounds_min[1]) / grid_size[1]

    if decomp and decomp.get('type') == 'grid' and grid_size and (show_high_level or show_grid):
        def get_region_bounds(region_id):
            """Convert region ID to bounding box coordinates.

            OMPL GridDecomposition uses row-major ordering with the formula:
                Region ID = x_index * length + y_index

            Where x_index is coord[0] (first dimension) and y_index is coord[1]
            (second dimension). The grid is indexed from the origin (bounds_min).

            The inverse mapping is:
                x_index = region_id // length
                y_index = region_id % length

            Note: grid_size[0] should equal 'length' (cells per dimension).
            For a 5x5 grid, length=5, so region IDs range from 0 to 24.
            """
            # OMPL formula: region_id = x_index * length + y_index
            # Inverse: x_index = region_id // length, y_index = region_id % length
            length = grid_size[0]  # Assumes uniform grid (grid_size[0] == grid_size[1])
            x_index = region_id // length
            y_index = region_id % length
            x = bounds_min[0] + x_index * cell_width
            y = bounds_min[1] + y_index * cell_height
            return x, y, cell_width, cell_height

        def get_region_center(region_id):
            x, y, w, h = get_region_bounds(region_id)
            return x + w / 2, y + h / 2

        # Draw grid lines first (background)
        if show_grid:
            for i in range(grid_size[0] + 1):
                x = bounds_min[0] + i * cell_width
                ax.axvline(x, color='darkgray', linewidth=1.0, alpha=0.4, linestyle='-', zorder=0)

            for j in range(grid_size[1] + 1):
                y = bounds_min[1] + j * cell_height
                ax.axhline(y, color='darkgray', linewidth=1.0, alpha=0.4, linestyle='-', zorder=0)

        if show_high_level and 'leads' in decomp and decomp['leads']:
            colors = plt.cm.tab10(np.linspace(0, 1, len(env['robots'])))
            for robot_idx, lead in enumerate(decomp['leads']):
                if lead:
                    color = colors[robot_idx]

                    # Fill regions along the high-level path with light color
                    for region_id in lead:
                        x, y, w, h = get_region_bounds(region_id)
                        rect = patches.Rectangle((x, y), w, h,
                                                linewidth=0,
                                                facecolor=color,
                                                alpha=0.12,
                                                zorder=0)
                        ax.add_patch(rect)

                    # Draw high-level path as connected line through region centers
                    if len(lead) >= 2:
                        path_x = []
                        path_y = []
                        for region_id in lead:
                            cx, cy = get_region_center(region_id)
                            path_x.append(cx)
                            path_y.append(cy)

                        # Draw the high-level path line (dashed)
                        hl_line, = ax.plot(path_x, path_y, '--',
                                          color=color, linewidth=2.5, alpha=0.7,
                                          zorder=2, marker='s', markersize=6,
                                          markerfacecolor=color, markeredgecolor='black',
                                          markeredgewidth=1)
                        high_level_path_artists.append((hl_line, f'Robot {robot_idx} HL Path'))

                        # Mark start and end of high-level path
                        start_cx, start_cy = get_region_center(lead[0])
                        end_cx, end_cy = get_region_center(lead[-1])
                        ax.plot(start_cx, start_cy, 'o', color=color, markersize=12,
                               markeredgecolor='black', markeredgewidth=2, alpha=0.8, zorder=3)
                        ax.plot(end_cx, end_cy, 'D', color=color, markersize=10,
                               markeredgecolor='black', markeredgewidth=2, alpha=0.8, zorder=3)

    colors = plt.cm.tab10(np.linspace(0, 1, len(env['robots'])))

    # Plot start and goal positions
    for robot_idx, robot_config in enumerate(env['robots']):
        color = colors[robot_idx]
        start = robot_config['start']
        goal = robot_config['goal']
        ax.plot(start[0], start[1], 'o', color=color, markersize=10,
                markeredgecolor='black', markeredgewidth=2, alpha=0.3)
        ax.plot(goal[0], goal[1], '*', color=color, markersize=13,
                markeredgecolor='black', markeredgewidth=1.5, alpha=0.3)

    # Initialize robot artists
    robot_artists = []
    trail_artists = []
    for robot_idx in range(len(paths)):
        color = colors[robot_idx]
        robot_name = env['robots'][robot_idx].get('name', f'Robot {robot_idx}')
        robot_radius = get_robot_radius(env['robots'][robot_idx])

        # Robot circle with actual size
        robot_circle = plt.Circle((0, 0), robot_radius, color=color, zorder=5,
                                 alpha=0.7, edgecolor='black', linewidth=2,
                                 label=robot_name)
        ax.add_patch(robot_circle)
        robot_artists.append(robot_circle)

        # Trail line
        trail_line, = ax.plot([], [], '-', color=color, linewidth=1.5, alpha=0.5)
        trail_artists.append(trail_line)

    ax.legend(loc='upper right')

    # Time display text
    time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                       verticalalignment='top', fontsize=12,
                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # Create slider for time control
    ax_slider = plt.axes([0.15, 0.08, 0.7, 0.03])
    time_slider = Slider(
        ax=ax_slider,
        label='Time (s)',
        valmin=0,
        valmax=max_time,
        valinit=0,
        valstep=max_time / 1000.0  # Smooth stepping
    )

    # Create play/pause button
    ax_play = plt.axes([0.45, 0.02, 0.1, 0.04])
    btn_play = Button(ax_play, 'Play')

    # Animation state
    anim_state = {
        'playing': False,
        'timer_id': None,
        'last_update_time': None
    }

    def update_display(t):
        """Update robot positions and trails based on time t"""
        for robot_idx in range(len(paths)):
            path = paths[robot_idx]
            if path:
                state = interpolate_at_time(path, waypoint_times, t)
                if state:
                    robot_artists[robot_idx].center = (state[0], state[1])
                xs, ys = trail_points(path, waypoint_times, t)
                trail_artists[robot_idx].set_data(xs, ys)

        progress_pct = (t / max_time * 100) if max_time > 0 else 0
        time_text.set_text(f'Time: {t:.2f}s / {max_time:.2f}s ({progress_pct:.1f}%)')
        fig.canvas.draw_idle()

    def on_slider_change(val):
        """Handle slider value changes"""
        if not anim_state['playing']:  # Only update when not playing
            update_display(val)

    def timer_callback():
        """Called periodically to update animation"""
        if not anim_state['playing']:
            return

        current_wall_time = time.time()

        if anim_state['last_update_time'] is None:
            dt = 0.05
        else:
            dt = current_wall_time - anim_state['last_update_time']

        anim_state['last_update_time'] = current_wall_time

        # Update time
        current_time = time_slider.val + dt

        # Loop at the end
        if current_time >= max_time:
            current_time = 0.0
            anim_state['last_update_time'] = None

        # Update slider without triggering callback
        time_slider.eventson = False
        time_slider.set_val(current_time)
        time_slider.eventson = True

        # Update display
        update_display(current_time)

    def on_play_pause(event):
        """Toggle play/pause"""
        if anim_state['playing']:
            # Pause
            anim_state['playing'] = False
            btn_play.label.set_text('Play')
            if anim_state['timer_id'] is not None:
                fig.canvas.mpl_disconnect(anim_state['timer_id'])
                anim_state['timer_id'] = None
            anim_state['last_update_time'] = None
        else:
            # Play
            anim_state['playing'] = True
            btn_play.label.set_text('Pause')
            anim_state['last_update_time'] = time.time()

            # Use a simple timer approach
            anim_state['timer_id'] = fig.canvas.new_timer(interval=50)
            anim_state['timer_id'].add_callback(timer_callback)
            anim_state['timer_id'].start()

    # Connect event handlers
    time_slider.on_changed(on_slider_change)
    btn_play.on_clicked(on_play_pause)

    # Initialize display at t=0
    update_display(0.0)

    plt.show()

def main():
    parser = argparse.ArgumentParser(
        description='Interactive visualization for Multi-Robot SyCLoP solutions',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Supported formats:
  - mr_syclop format: uses --env or --config with environment/robots fields
  - s2m2/DB-RRT format: uses --config with agents/limits/obstacles/starts/goals fields
  - dynobench/DB-RRT solution format: --env/--config optional (inferred from trajectory)

Robot geometry priority:
  1. Explicit 'radius' in config file
  2. 'size' field in agent definition (s2m2 format)
  3. Dynobench model file (loaded via --models-path)
  4. DEFAULT_GEOMETRIES lookup by robot type
  5. Fallback default (0.3)

Examples:
  # MR-SyCLoP format with environment file
  %(prog)s --env env.yaml --solution solution.yaml

  # s2m2/DB-RRT format with problem config
  %(prog)s --config problem.yaml --solution out.yaml

  # With explicit models path for robot geometry
  %(prog)s --config problem.yaml --solution out.yaml --models-path ./dynobench/models/

  # Solution only (geometry inferred from solution + models)
  %(prog)s --solution out_dbrrt.yaml

  # Speed control
  %(prog)s --config problem.yaml --solution out.yaml --speed 2.0

  # Hide high-level paths (show only trajectories)
  %(prog)s --solution out.yaml --no-high-level

  # Hide decomposition grid
  %(prog)s --solution out.yaml --no-grid

  # With planner config for decomposition grid (when not in solution)
  %(prog)s --solution out.yaml --planner-config planner_cfgs/config.yaml
        '''
    )
    parser.add_argument('--env', required=False, default=None,
                        help='Environment YAML file (mr_syclop format)')
    parser.add_argument('--config', required=False, default=None,
                        help='Problem config YAML file (s2m2 or mr_syclop format)')
    parser.add_argument('--planner-config', required=False, default=None,
                        help='Planner config YAML file (for decomposition.resolution)')
    parser.add_argument('--solution', required=True, help='Solution YAML file')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Robot speed in map units per second (default: 1.0)')
    parser.add_argument('--models-path', required=False, default=None,
                        help='Path to dynobench models directory for robot geometry')
    parser.add_argument('--show-high-level', action='store_true', default=True,
                        help='Show high-level paths through decomposition regions (default: True)')
    parser.add_argument('--no-high-level', action='store_true',
                        help='Hide high-level paths')
    parser.add_argument('--show-grid', action='store_true', default=True,
                        help='Show decomposition grid lines (default: True)')
    parser.add_argument('--no-grid', action='store_true',
                        help='Hide decomposition grid lines')

    args = parser.parse_args()

    # Handle negation flags
    show_high_level = args.show_high_level and not args.no_high_level
    show_grid = args.show_grid and not args.no_grid

    interactive_animation(
        env_file=args.env,
        solution_file=args.solution,
        initial_speed=args.speed,
        config_file=args.config,
        models_path=args.models_path,
        show_high_level=show_high_level,
        show_grid=show_grid,
        planner_config_file=args.planner_config,
    )

if __name__ == '__main__':
    main()
