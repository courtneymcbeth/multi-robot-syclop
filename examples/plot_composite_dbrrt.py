#!/usr/bin/env python3
"""
Plot composite_dbrrt solution trajectories
"""

import yaml
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import sys

def load_solution(filename):
    with open(filename, 'r') as f:
        return yaml.safe_load(f)

def load_problem(filename):
    with open(filename, 'r') as f:
        return yaml.safe_load(f)

def plot_solution(solution_file, problem_file=None):
    # Load solution
    sol = load_solution(solution_file)

    if not sol.get('solved', False):
        print("No solution found!")
        return

    # Create figure
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))

    # Load and plot environment if provided
    env_min = [0, 0]
    env_max = [10, 10]
    obstacles = []

    if problem_file:
        prob = load_problem(problem_file)
        env = prob.get('environment', {})
        env_min = env.get('min', [0, 0])
        env_max = env.get('max', [10, 10])
        obstacles = env.get('obstacles', [])

    # Set axis limits
    ax.set_xlim(env_min[0] - 0.5, env_max[0] + 0.5)
    ax.set_ylim(env_min[1] - 0.5, env_max[1] + 0.5)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

    # Plot obstacles
    for obs in obstacles:
        if obs.get('type') == 'box':
            center = obs['center']
            size = obs['size']
            rect = patches.Rectangle(
                (center[0] - size[0]/2, center[1] - size[1]/2),
                size[0], size[1],
                linewidth=2, edgecolor='black', facecolor='gray', alpha=0.7
            )
            ax.add_patch(rect)

    # Colors for different robots
    colors = ['blue', 'red', 'green', 'orange', 'purple', 'cyan', 'magenta', 'yellow']

    # Plot trajectories
    result = sol.get('result', [])
    for i, robot_traj in enumerate(result):
        states = robot_traj.get('states', [])
        if not states:
            continue

        color = colors[i % len(colors)]

        # Extract x, y positions
        x = [s[0] for s in states]
        y = [s[1] for s in states]

        # Plot trajectory line
        ax.plot(x, y, '-', color=color, linewidth=2, alpha=0.7, label=f'Robot {i}')

        # Plot start point (larger)
        ax.plot(x[0], y[0], 'o', color=color, markersize=15, markeredgecolor='black', markeredgewidth=2)
        ax.annotate(f'S{i}', (x[0], y[0]), fontsize=10, ha='center', va='center', color='white', fontweight='bold')

        # Plot goal point (star)
        ax.plot(x[-1], y[-1], '*', color=color, markersize=20, markeredgecolor='black', markeredgewidth=1)
        ax.annotate(f'G{i}', (x[-1] + 0.3, y[-1] + 0.3), fontsize=10, ha='left', va='bottom', color=color, fontweight='bold')

        # Plot intermediate points with direction arrows
        step = max(1, len(states) // 10)  # Show ~10 arrows
        for j in range(0, len(states) - 1, step):
            if len(states[j]) >= 3:
                theta = states[j][2]
                dx = 0.3 * np.cos(theta)
                dy = 0.3 * np.sin(theta)
                ax.arrow(x[j], y[j], dx, dy, head_width=0.15, head_length=0.1,
                        fc=color, ec=color, alpha=0.5)

    ax.legend(loc='upper left')

    planning_time = sol.get('planning_time', 0)
    ax.set_title(f'Composite DB-RRT Solution (planning time: {planning_time:.3f}s)')

    plt.tight_layout()

    # Save figure
    output_file = solution_file.replace('.yaml', '.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {output_file}")

    plt.show()

if __name__ == '__main__':
    solution_file = '/tmp/composite_dbrrt_output.yaml'
    problem_file = 'examples/composite_dbrrt_test.yaml'

    if len(sys.argv) > 1:
        solution_file = sys.argv[1]
    if len(sys.argv) > 2:
        problem_file = sys.argv[2]

    plot_solution(solution_file, problem_file)
