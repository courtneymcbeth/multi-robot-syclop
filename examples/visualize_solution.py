#!/usr/bin/env python3
"""
Simple visualization script for Multi-Robot SyCLoP solutions
Requires: matplotlib, pyyaml
Install: pip install matplotlib pyyaml
"""

import bisect
import yaml
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np
import argparse
import sys

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

def plot_solution(env_file, solution_file, output_image=None):
    """Plot the solution paths for all robots"""

    # Load environment and solution
    env = load_yaml(env_file)
    solution = load_yaml(solution_file)

    if not solution or 'result' not in solution:
        print("Warning: No solution found in file")
        return

    # Extract environment bounds
    env_min = env['environment']['min']
    env_max = env['environment']['max']

    # Create figure
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(env_min[0], env_max[0])
    ax.set_ylim(env_min[1], env_max[1])
    ax.set_aspect('equal')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Multi-Robot SyCLoP Solution Paths')

    # Draw regions if decomposition data is available
    if 'decomposition' in solution:
        decomp = solution['decomposition']
        if decomp.get('type') == 'grid':
            grid_size = decomp['grid_size']
            bounds_min = decomp['bounds']['min']
            bounds_max = decomp['bounds']['max']

            # Calculate cell dimensions
            cell_width = (bounds_max[0] - bounds_min[0]) / grid_size[0]
            cell_height = (bounds_max[1] - bounds_min[1]) / grid_size[1]

            # Helper function to get region cell coordinates
            def get_region_bounds(region_id):
                """Get the (x, y, width, height) for a region given its ID"""
                row = region_id // grid_size[0]
                col = region_id % grid_size[0]
                x = bounds_min[0] + col * cell_width
                y = bounds_min[1] + row * cell_height
                return x, y, cell_width, cell_height

            # Shade lead regions for each robot (if leads available)
            if 'leads' in decomp and decomp['leads']:
                colors = plt.cm.tab10(np.linspace(0, 1, len(env['robots'])))
                for robot_idx, lead in enumerate(decomp['leads']):
                    if lead:  # Check if lead is not empty
                        color = colors[robot_idx]
                        for region_id in lead:
                            x, y, w, h = get_region_bounds(region_id)
                            rect = patches.Rectangle((x, y), w, h,
                                                    linewidth=0,
                                                    facecolor=color,
                                                    alpha=0.15,
                                                    zorder=0)
                            ax.add_patch(rect)

            # Draw region boundary lines (thicker and more visible)
            for i in range(grid_size[0] + 1):
                x = bounds_min[0] + i * cell_width
                ax.axvline(x, color='darkgray', linewidth=1.5, alpha=0.6, linestyle='-')

            for j in range(grid_size[1] + 1):
                y = bounds_min[1] + j * cell_height
                ax.axhline(y, color='darkgray', linewidth=1.5, alpha=0.6, linestyle='-')

            # Optionally label regions (only for small grids)
            if grid_size[0] * grid_size[1] <= 25:
                region_id = 0
                for j in range(grid_size[1]):
                    for i in range(grid_size[0]):
                        x_center = bounds_min[0] + (i + 0.5) * cell_width
                        y_center = bounds_min[1] + (j + 0.5) * cell_height
                        ax.text(x_center, y_center, str(region_id),
                               ha='center', va='center', fontsize=8,
                               alpha=0.3, color='gray')
                        region_id += 1

    # Colors for different robots
    colors = plt.cm.tab10(np.linspace(0, 1, len(env['robots'])))

    # Plot each robot's path
    for robot_idx, (robot_config, path_data) in enumerate(zip(env['robots'], solution['result'])):
        if 'states' not in path_data:
            continue

        states = path_data['states']
        if not states:
            continue

        # Extract x, y coordinates
        xs = [state[0] for state in states]
        ys = [state[1] for state in states]

        color = colors[robot_idx]
        robot_name = robot_config.get('name', f'Robot {robot_idx}')
        robot_radius = get_robot_radius(robot_config)

        # Plot path
        ax.plot(xs, ys, '-', color=color, linewidth=2, alpha=0.7, label=f'{robot_name} path')

        # Plot robot geometry at start and goal positions
        start = robot_config['start']
        goal = robot_config['goal']

        # Draw circles showing robot size at start and goal
        start_circle = patches.Circle((start[0], start[1]), robot_radius,
                                     color=color, alpha=0.2, edgecolor='black', linewidth=2)
        goal_circle = patches.Circle((goal[0], goal[1]), robot_radius,
                                    color=color, alpha=0.2, edgecolor='black',
                                    linewidth=2, linestyle='--')
        ax.add_patch(start_circle)
        ax.add_patch(goal_circle)

        # Plot start position (circle marker)
        ax.plot(start[0], start[1], 'o', color=color, markersize=12,
                markeredgecolor='black', markeredgewidth=2, label=f'{robot_name} start')

        # Plot goal position (star)
        ax.plot(goal[0], goal[1], '*', color=color, markersize=15,
                markeredgecolor='black', markeredgewidth=1.5, label=f'{robot_name} goal')

        # Add arrows to show direction along path
        if len(xs) > 1:
            for i in range(0, len(xs)-1, max(1, len(xs)//5)):
                dx = xs[i+1] - xs[i]
                dy = ys[i+1] - ys[i]
                ax.arrow(xs[i], ys[i], dx*0.8, dy*0.8,
                        head_width=0.2, head_length=0.15, fc=color, ec=color, alpha=0.5)

    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()

    if output_image:
        plt.savefig(output_image, dpi=150, bbox_inches='tight')
        print(f"Saved visualization to: {output_image}")
    else:
        plt.show()

def animate_solution(env_file, solution_file, output_video=None, speed=5.0, fps=20):
    """Create an animation of the robots moving along their paths"""

    # Load environment and solution
    env = load_yaml(env_file)
    solution = load_yaml(solution_file)

    if not solution or 'result' not in solution:
        print("Warning: No solution found in file")
        return

    # Extract environment bounds
    env_min = env['environment']['min']
    env_max = env['environment']['max']

    # Extract all paths
    paths = []
    max_length = 0
    for robot_idx, path_data in enumerate(solution['result']):
        if 'states' in path_data and path_data['states']:
            paths.append(path_data['states'])
            max_length = max(max_length, len(path_data['states']))
        else:
            paths.append([])

    if max_length == 0:
        print("No valid paths to animate")
        return

    def compute_cumulative_distances(path):
        distances = [0.0]
        for i in range(1, len(path)):
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
            distances.append(distances[-1] + float(np.hypot(dx, dy)))
        return distances

    def interpolate_at_distance(path, distances, distance):
        if not path:
            return None
        if distance <= 0.0:
            return path[0]
        total = distances[-1]
        if distance >= total:
            return path[-1]
        idx = bisect.bisect_right(distances, distance) - 1
        if idx >= len(path) - 1:
            return path[-1]
        segment_len = distances[idx + 1] - distances[idx]
        if segment_len <= 1e-9:
            return path[idx]
        ratio = (distance - distances[idx]) / segment_len
        x = path[idx][0] + ratio * (path[idx + 1][0] - path[idx][0])
        y = path[idx][1] + ratio * (path[idx + 1][1] - path[idx][1])
        return [x, y]

    def trail_points(path, distances, distance):
        if not path:
            return [], []
        if distance <= 0.0:
            return [path[0][0]], [path[0][1]]
        total = distances[-1]
        if distance >= total:
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
            return xs, ys
        idx = bisect.bisect_right(distances, distance) - 1
        xs = [p[0] for p in path[:idx + 1]]
        ys = [p[1] for p in path[:idx + 1]]
        interp = interpolate_at_distance(path, distances, distance)
        if interp is not None and (not xs or interp[0] != xs[-1] or interp[1] != ys[-1]):
            xs.append(interp[0])
            ys.append(interp[1])
        return xs, ys

    path_infos = []
    for path in paths:
        if path:
            distances = compute_cumulative_distances(path)
            total = distances[-1]
        else:
            distances = []
            total = 0.0
        path_infos.append({"path": path, "distances": distances, "total": total})

    # Create figure
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(env_min[0], env_max[0])
    ax.set_ylim(env_min[1], env_max[1])
    ax.set_aspect('equal')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

    # Draw regions if decomposition data is available
    if 'decomposition' in solution:
        decomp = solution['decomposition']
        if decomp.get('type') == 'grid':
            grid_size = decomp['grid_size']
            bounds_min = decomp['bounds']['min']
            bounds_max = decomp['bounds']['max']

            # Calculate cell dimensions
            cell_width = (bounds_max[0] - bounds_min[0]) / grid_size[0]
            cell_height = (bounds_max[1] - bounds_min[1]) / grid_size[1]

            # Helper function to get region cell coordinates
            def get_region_bounds(region_id):
                """Get the (x, y, width, height) for a region given its ID"""
                row = region_id // grid_size[0]
                col = region_id % grid_size[0]
                x = bounds_min[0] + col * cell_width
                y = bounds_min[1] + row * cell_height
                return x, y, cell_width, cell_height

            # Shade lead regions for each robot (if leads available)
            if 'leads' in decomp and decomp['leads']:
                colors = plt.cm.tab10(np.linspace(0, 1, len(env['robots'])))
                for robot_idx, lead in enumerate(decomp['leads']):
                    if lead:  # Check if lead is not empty
                        color = colors[robot_idx]
                        for region_id in lead:
                            x, y, w, h = get_region_bounds(region_id)
                            rect = patches.Rectangle((x, y), w, h,
                                                    linewidth=0,
                                                    facecolor=color,
                                                    alpha=0.15,
                                                    zorder=0)
                            ax.add_patch(rect)

            # Draw region boundary lines (thicker and more visible)
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
    time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                       verticalalignment='top', fontsize=12,
                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    if speed <= 0.0:
        speed = 1.0
    if fps <= 0:
        fps = 20
    max_time = 0.0
    for info in path_infos:
        if info["total"] > 0.0:
            max_time = max(max_time, info["total"] / speed)
    if max_time <= 0.0:
        print("No valid paths to animate")
        return
    total_frames = int(np.ceil(max_time * fps)) + 1

    def init():
        for artist in robot_artists:
            artist.center = (0, 0)
        for artist in trail_artists:
            artist.set_data([], [])
        time_text.set_text('')
        return robot_artists + trail_artists + [time_text]

    def update(frame):
        t = frame / fps
        for robot_idx in range(len(paths)):
            info = path_infos[robot_idx]
            if info["path"]:
                distance = min(info["total"], speed * t)
                state = interpolate_at_distance(info["path"], info["distances"], distance)
                if state:
                    robot_artists[robot_idx].center = (state[0], state[1])
                xs, ys = trail_points(info["path"], info["distances"], distance)
                trail_artists[robot_idx].set_data(xs, ys)

        time_text.set_text(f'Time: {t:.2f}s / {max_time:.2f}s')
        return robot_artists + trail_artists + [time_text]

    interval_ms = int(1000 / fps)
    anim = FuncAnimation(fig, update, init_func=init, frames=total_frames,
                        interval=interval_ms, blit=True, repeat=True)

    if output_video:
        print(f"Saving animation to: {output_video}")
        anim.save(output_video, writer='pillow', fps=fps)
        print("Animation saved!")
    else:
        plt.show()

def main():
    parser = argparse.ArgumentParser(description='Visualize Multi-Robot SyCLoP solutions')
    parser.add_argument('--env', required=True, help='Environment YAML file')
    parser.add_argument('--solution', required=True, help='Solution YAML file')
    parser.add_argument('--output', help='Output image file (e.g., solution.png)')
    parser.add_argument('--animate', action='store_true', help='Create animation')
    parser.add_argument('--video', help='Save animation to file (e.g., solution.gif)')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Robot speed in map units per second (default: 1.0)')
    parser.add_argument('--fps', type=int, default=20,
                        help='Animation frames per second (default: 20)')

    args = parser.parse_args()

    if args.animate or args.video:
        animate_solution(args.env, args.solution, args.video, args.speed, args.fps)
    else:
        plot_solution(args.env, args.solution, args.output)

if __name__ == '__main__':
    main()
