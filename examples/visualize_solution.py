#!/usr/bin/env python3
"""
Simple visualization script for Multi-Robot SyCLoP solutions
Requires: matplotlib, pyyaml
Install: pip install matplotlib pyyaml
"""

import yaml
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np
import argparse
import sys

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
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Multi-Robot SyCLoP Solution Paths')

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

        # Plot path
        ax.plot(xs, ys, '-', color=color, linewidth=2, alpha=0.7, label=f'{robot_name} path')

        # Plot start position (circle)
        start = robot_config['start']
        ax.plot(start[0], start[1], 'o', color=color, markersize=12,
                markeredgecolor='black', markeredgewidth=2, label=f'{robot_name} start')

        # Plot goal position (star)
        goal = robot_config['goal']
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

def animate_solution(env_file, solution_file, output_video=None):
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

    # Create figure
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(env_min[0], env_max[0])
    ax.set_ylim(env_min[1], env_max[1])
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

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

        # Robot circle
        robot_circle = plt.Circle((0, 0), 0.3, color=color, zorder=5,
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

    def init():
        for artist in robot_artists:
            artist.center = (0, 0)
        for artist in trail_artists:
            artist.set_data([], [])
        time_text.set_text('')
        return robot_artists + trail_artists + [time_text]

    def update(frame):
        for robot_idx, path in enumerate(paths):
            if path:
                # Get current position (loop at end if path is shorter)
                idx = min(frame, len(path) - 1)
                state = path[idx]

                # Update robot position
                robot_artists[robot_idx].center = (state[0], state[1])

                # Update trail
                trail_states = path[:idx+1]
                xs = [s[0] for s in trail_states]
                ys = [s[1] for s in trail_states]
                trail_artists[robot_idx].set_data(xs, ys)

        time_text.set_text(f'Step: {frame}/{max_length-1}')
        return robot_artists + trail_artists + [time_text]

    anim = FuncAnimation(fig, update, init_func=init, frames=max_length,
                        interval=200, blit=True, repeat=True)

    if output_video:
        print(f"Saving animation to: {output_video}")
        anim.save(output_video, writer='pillow', fps=5)
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

    args = parser.parse_args()

    if args.animate or args.video:
        animate_solution(args.env, args.solution, args.video)
    else:
        plot_solution(args.env, args.solution, args.output)

if __name__ == '__main__':
    main()
