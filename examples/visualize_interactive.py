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

def load_yaml(filename):
    """Load YAML file"""
    try:
        with open(filename, 'r') as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found")
        sys.exit(1)
    except yaml.YAMLError as e:
        print(f"Error parsing YAML: {e}")
        sys.exit(1)


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


def create_synthetic_env(solution, paths):
    """Create a synthetic environment dict from DB-RRT solution data.

    This allows visualization without a separate environment file.
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

    for i, path in enumerate(paths):
        robot_config = {
            'type': robot_type,
            'name': f'Robot {i}',
            'radius': 0.3,
        }

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

def interactive_animation(env_file, solution_file, initial_speed=1.0):
    """Create an interactive visualization with a slider control

    Args:
        env_file: Path to environment YAML file (optional, can be None for DB-RRT format)
        solution_file: Path to solution YAML file
        initial_speed: Robot speed in map units per second
    """

    # Load solution
    solution = load_yaml(solution_file)

    if not solution or 'result' not in solution:
        print("Warning: No solution found in file")
        return

    # Detect format
    fmt = detect_format(solution)
    print(f"Detected solution format: {fmt}")

    # Extract paths using unified function
    paths = extract_paths_from_solution(solution)

    if not paths:
        print("Warning: No valid paths found in solution")
        return

    # Load or create environment
    if env_file:
        env = load_yaml(env_file)
    else:
        # Create synthetic environment from solution data
        print("No environment file provided, inferring from solution...")
        env = create_synthetic_env(solution, paths)

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
    if 'decomposition' in solution:
        decomp = solution['decomposition']
        if decomp.get('type') == 'grid':
            grid_size = decomp['grid_size']
            bounds_min = decomp['bounds']['min']
            bounds_max = decomp['bounds']['max']

            cell_width = (bounds_max[0] - bounds_min[0]) / grid_size[0]
            cell_height = (bounds_max[1] - bounds_min[1]) / grid_size[1]

            def get_region_bounds(region_id):
                row = region_id // grid_size[0]
                col = region_id % grid_size[0]
                x = bounds_min[0] + col * cell_width
                y = bounds_min[1] + row * cell_height
                return x, y, cell_width, cell_height

            if 'leads' in decomp and decomp['leads']:
                colors = plt.cm.tab10(np.linspace(0, 1, len(env['robots'])))
                for robot_idx, lead in enumerate(decomp['leads']):
                    if lead:
                        color = colors[robot_idx]
                        for region_id in lead:
                            x, y, w, h = get_region_bounds(region_id)
                            rect = patches.Rectangle((x, y), w, h,
                                                    linewidth=0,
                                                    facecolor=color,
                                                    alpha=0.15,
                                                    zorder=0)
                            ax.add_patch(rect)

            for i in range(grid_size[0] + 1):
                x = bounds_min[0] + i * cell_width
                ax.axvline(x, color='darkgray', linewidth=1.5, alpha=0.6, linestyle='-')

            for j in range(grid_size[1] + 1):
                y = bounds_min[1] + j * cell_height
                ax.axhline(y, color='darkgray', linewidth=1.5, alpha=0.6, linestyle='-')

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
        epilog='''
Supported formats:
  - mr_syclop format: requires --env file
  - dynobench/DB-RRT format: --env is optional (bounds inferred from trajectory)

Examples:
  %(prog)s --env env.yaml --solution solution.yaml
  %(prog)s --solution out_dbrrt.yaml
  %(prog)s --solution out_dbrrt.yaml --env env.yaml --speed 2.0
        '''
    )
    parser.add_argument('--env', required=False, default=None,
                        help='Environment YAML file (optional for DB-RRT format)')
    parser.add_argument('--solution', required=True, help='Solution YAML file')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Robot speed in map units per second (default: 1.0)')

    args = parser.parse_args()

    interactive_animation(args.env, args.solution, args.speed)

if __name__ == '__main__':
    main()
