#!/usr/bin/env python3
"""
Visualization utility for multi-robot planning experiments.

Usage:
    # View a scenario (environment only)
    python visualize_problem.py scenarios/corridor.yaml

    # View a problem instance (environment + robot start/goals)
    python visualize_problem.py problems/corridor/robots_4/seed_0.yaml

    # View a solution (problem + paths)
    python visualize_problem.py problem.yaml --solution solution.yaml

    # Save to file instead of displaying
    python visualize_problem.py scenario.yaml -o output.png
"""

import yaml
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import argparse
import sys
from pathlib import Path

# Robot geometry definitions (radius or bounding box)
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
    """Load YAML file."""
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
    """Get the radius/size of a robot for visualization."""
    if 'radius' in robot_config:
        return robot_config['radius']
    robot_type = robot_config.get('type', '')
    if robot_type in DEFAULT_GEOMETRIES:
        max_radius = 0.0
        for spec in DEFAULT_GEOMETRIES[robot_type]:
            if spec[0] == 'sphere':
                radius = spec[1]
            else:
                # For boxes, use half-diagonal as approximate radius
                radius = 0.5 * float(np.hypot(spec[1], spec[2]))
            max_radius = max(max_radius, radius)
        if max_radius > 0.0:
            return max_radius
    return 0.5  # Default fallback


def draw_obstacles(ax, env):
    """Draw obstacles from environment definition."""
    obstacles = env.get('environment', {}).get('obstacles', [])
    for obs in obstacles:
        obs_type = obs.get('type', '')
        if obs_type == 'box':
            size = obs.get('size', [0.0, 0.0])
            center = obs.get('center', [0.0, 0.0])
            width = float(size[0])
            height = float(size[1])
            # Convert center to corner coordinates
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
            print(f"Warning: Unsupported obstacle type '{obs_type}'")


def draw_robots(ax, env, show_radius=True):
    """Draw robot start and goal positions."""
    robots = env.get('robots', [])
    if not robots:
        return

    colors = plt.cm.tab10(np.linspace(0, 1, max(len(robots), 10)))

    for robot_idx, robot_config in enumerate(robots):
        color = colors[robot_idx % len(colors)]
        start = robot_config.get('start', [])
        goal = robot_config.get('goal', [])
        robot_name = robot_config.get('name', f'Robot {robot_idx}')

        if len(start) < 2 or len(goal) < 2:
            continue

        # Get robot radius for visualization
        robot_radius = get_robot_radius(robot_config)

        if show_radius:
            # Draw robot footprint at start
            start_circle = patches.Circle(
                (start[0], start[1]), robot_radius,
                color=color, alpha=0.2,
                edgecolor='black', linewidth=1.5,
                zorder=2
            )
            ax.add_patch(start_circle)

            # Draw robot footprint at goal (dashed outline)
            goal_circle = patches.Circle(
                (goal[0], goal[1]), robot_radius,
                color=color, alpha=0.2,
                edgecolor='black', linewidth=1.5,
                linestyle='--', zorder=2
            )
            ax.add_patch(goal_circle)

        # Draw start position marker (filled circle)
        ax.plot(start[0], start[1], 'o',
                color=color, markersize=10,
                markeredgecolor='black', markeredgewidth=1.5,
                label=f'{robot_name} start', zorder=3)

        # Draw goal position marker (star)
        ax.plot(goal[0], goal[1], '*',
                color=color, markersize=14,
                markeredgecolor='black', markeredgewidth=1,
                label=f'{robot_name} goal', zorder=3)

        # Draw line connecting start to goal (straight line reference)
        ax.plot([start[0], goal[0]], [start[1], goal[1]],
                '--', color=color, alpha=0.3, linewidth=1, zorder=1)


def draw_solution_paths(ax, env, solution):
    """Draw solution paths for all robots."""
    robots = env.get('robots', [])
    result = solution.get('result', [])

    if not result:
        return

    colors = plt.cm.tab10(np.linspace(0, 1, max(len(robots), 10)))

    for robot_idx, path_data in enumerate(result):
        if 'states' not in path_data:
            continue

        states = path_data['states']
        if len(states) < 2:
            continue

        color = colors[robot_idx % len(colors)]

        # Extract x, y coordinates
        xs = [state[0] for state in states]
        ys = [state[1] for state in states]

        # Draw path
        ax.plot(xs, ys, '-', color=color, linewidth=2, alpha=0.8, zorder=4)


def visualize_environment(env_file, solution_file=None, output_file=None,
                         title=None, show_legend=True, figsize=(10, 8)):
    """
    Visualize an environment with optional solution paths.

    Args:
        env_file: Path to environment/problem YAML file
        solution_file: Optional path to solution YAML file
        output_file: Optional path to save image (if None, displays interactively)
        title: Optional title for the plot
        show_legend: Whether to show legend
        figsize: Figure size tuple
    """
    env = load_yaml(env_file)
    solution = load_yaml(solution_file) if solution_file else None

    # Extract environment bounds
    env_min = env['environment']['min']
    env_max = env['environment']['max']

    # Create figure
    fig, ax = plt.subplots(figsize=figsize)
    ax.set_xlim(env_min[0], env_max[0])
    ax.set_ylim(env_min[1], env_max[1])
    ax.set_aspect('equal')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

    # Set title
    if title:
        ax.set_title(title)
    else:
        robots = env.get('robots', [])
        if robots:
            ax.set_title(f'Problem: {len(robots)} robots')
        else:
            ax.set_title('Environment')

    # Draw components
    draw_obstacles(ax, env)
    draw_robots(ax, env)

    if solution:
        draw_solution_paths(ax, env, solution)

        # Add solution info to title
        solved = solution.get('solved', solution.get('success', False))
        planning_time = solution.get('planning_time', 0)
        status = "Solved" if solved else "Failed"
        ax.set_title(f'{ax.get_title()} - {status} ({planning_time:.2f}s)')

    # Add legend
    if show_legend and env.get('robots'):
        ax.legend(loc='upper left', fontsize=8, ncol=2)

    # Add grid
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved to: {output_file}")
        plt.close()
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Visualize multi-robot planning problems and solutions',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('input', help='Input YAML file (environment or problem)')
    parser.add_argument('--solution', '-s', help='Solution YAML file')
    parser.add_argument('--output', '-o', help='Output image file (PNG, PDF, etc.)')
    parser.add_argument('--title', '-t', help='Custom title for the plot')
    parser.add_argument('--no-legend', action='store_true', help='Hide legend')
    parser.add_argument('--figsize', type=float, nargs=2, default=[10, 8],
                       help='Figure size (width height)')

    args = parser.parse_args()

    visualize_environment(
        args.input,
        solution_file=args.solution,
        output_file=args.output,
        title=args.title,
        show_legend=not args.no_legend,
        figsize=tuple(args.figsize)
    )


if __name__ == '__main__':
    main()
