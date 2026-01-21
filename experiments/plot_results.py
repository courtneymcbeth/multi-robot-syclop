#!/usr/bin/env python3
"""
Generate plots from aggregated experiment results.

Usage:
    # Plot from aggregated CSV
    python plot_results.py analysis/summary.csv

    # Plot from results directory
    python plot_results.py results/ -o analysis/plots/

    # Specific plot types
    python plot_results.py summary.csv --type success_rate
"""

import json
import csv
import argparse
from pathlib import Path
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np

from aggregate_results import aggregate_results
from parse_results import parse_results_directory


def load_aggregated_data(input_path):
    """Load aggregated data from CSV, JSON, or compute from results directory."""
    input_path = Path(input_path)

    if input_path.is_file():
        if input_path.suffix == '.csv':
            data = []
            with open(input_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    # Convert types
                    for key in ['num_robots', 'seeds_run', 'seeds_solved']:
                        if key in row and row[key]:
                            row[key] = int(row[key])
                    for key in ['success_rate', 'mean_time', 'std_time', 'median_time',
                               'mean_path_length', 'std_path_length', 'mean_makespan']:
                        if key in row and row[key]:
                            try:
                                row[key] = float(row[key])
                            except ValueError:
                                row[key] = None
                    data.append(row)
            return data
        elif input_path.suffix == '.json':
            with open(input_path, 'r') as f:
                return json.load(f)
    elif input_path.is_dir():
        results = parse_results_directory(input_path)
        return aggregate_results(results)

    raise ValueError(f"Cannot load data from: {input_path}")


def plot_success_rate(data, output_dir, by_scenario=True):
    """
    Plot success rate vs. number of robots.

    Creates one plot per scenario (or one combined plot).
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Group by scenario
    scenarios = defaultdict(list)
    for row in data:
        scenarios[row['scenario']].append(row)

    # Color map for planners
    all_planners = sorted(set(row['planner'] for row in data))
    colors = plt.cm.tab10(np.linspace(0, 1, len(all_planners)))
    planner_colors = dict(zip(all_planners, colors))

    if by_scenario:
        for scenario, scenario_data in scenarios.items():
            fig, ax = plt.subplots(figsize=(10, 6))

            # Group by planner
            planners = defaultdict(list)
            for row in scenario_data:
                planners[row['planner']].append(row)

            for planner, planner_data in sorted(planners.items()):
                # Sort by robot count
                planner_data.sort(key=lambda x: x['num_robots'])

                robots = [r['num_robots'] for r in planner_data]
                success = [r['success_rate'] * 100 for r in planner_data]

                ax.plot(robots, success, 'o-', label=planner,
                       color=planner_colors[planner], linewidth=2, markersize=8)

            ax.set_xlabel('Number of Robots', fontsize=12)
            ax.set_ylabel('Success Rate (%)', fontsize=12)
            ax.set_title(f'Success Rate vs. Robot Count - {scenario}', fontsize=14)
            ax.legend(loc='best')
            ax.grid(True, alpha=0.3)
            ax.set_ylim(-5, 105)

            plt.tight_layout()
            plt.savefig(output_dir / f'success_rate_{scenario}.png', dpi=150)
            plt.close()

            print(f"Saved: success_rate_{scenario}.png")
    else:
        # Combined plot
        fig, ax = plt.subplots(figsize=(12, 8))

        for scenario, scenario_data in scenarios.items():
            planners = defaultdict(list)
            for row in scenario_data:
                planners[row['planner']].append(row)

            for planner, planner_data in sorted(planners.items()):
                planner_data.sort(key=lambda x: x['num_robots'])
                robots = [r['num_robots'] for r in planner_data]
                success = [r['success_rate'] * 100 for r in planner_data]

                ax.plot(robots, success, 'o-',
                       label=f'{scenario}/{planner}',
                       linewidth=2, markersize=6, alpha=0.7)

        ax.set_xlabel('Number of Robots', fontsize=12)
        ax.set_ylabel('Success Rate (%)', fontsize=12)
        ax.set_title('Success Rate vs. Robot Count', fontsize=14)
        ax.legend(loc='best', fontsize=8, ncol=2)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-5, 105)

        plt.tight_layout()
        plt.savefig(output_dir / 'success_rate_combined.png', dpi=150)
        plt.close()

        print("Saved: success_rate_combined.png")


def plot_planning_time(data, output_dir):
    """Plot planning time vs. number of robots with error bars."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Group by scenario
    scenarios = defaultdict(list)
    for row in data:
        if row.get('mean_time') is not None:
            scenarios[row['scenario']].append(row)

    all_planners = sorted(set(row['planner'] for row in data))
    colors = plt.cm.tab10(np.linspace(0, 1, len(all_planners)))
    planner_colors = dict(zip(all_planners, colors))

    for scenario, scenario_data in scenarios.items():
        fig, ax = plt.subplots(figsize=(10, 6))

        planners = defaultdict(list)
        for row in scenario_data:
            planners[row['planner']].append(row)

        for planner, planner_data in sorted(planners.items()):
            planner_data.sort(key=lambda x: x['num_robots'])

            robots = [r['num_robots'] for r in planner_data]
            times = [r['mean_time'] for r in planner_data]
            stds = [r.get('std_time', 0) or 0 for r in planner_data]

            ax.errorbar(robots, times, yerr=stds, fmt='o-',
                       label=planner, color=planner_colors[planner],
                       linewidth=2, markersize=8, capsize=4)

        ax.set_xlabel('Number of Robots', fontsize=12)
        ax.set_ylabel('Planning Time (s)', fontsize=12)
        ax.set_title(f'Planning Time vs. Robot Count - {scenario}', fontsize=14)
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(output_dir / f'planning_time_{scenario}.png', dpi=150)
        plt.close()

        print(f"Saved: planning_time_{scenario}.png")


def plot_max_robots_bar(data, output_dir, min_success_rate=0.5):
    """Bar chart showing maximum robots achieved per planner/scenario."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Find max robots for each (scenario, planner)
    max_robots = {}
    for row in data:
        if row['success_rate'] >= min_success_rate:
            key = (row['scenario'], row['planner'])
            current = max_robots.get(key, 0)
            max_robots[key] = max(current, row['num_robots'])

    if not max_robots:
        print("No data for max robots bar chart")
        return

    # Organize data
    scenarios = sorted(set(k[0] for k in max_robots.keys()))
    planners = sorted(set(k[1] for k in max_robots.keys()))

    x = np.arange(len(scenarios))
    width = 0.8 / len(planners)

    fig, ax = plt.subplots(figsize=(12, 6))

    colors = plt.cm.tab10(np.linspace(0, 1, len(planners)))

    for i, planner in enumerate(planners):
        values = [max_robots.get((s, planner), 0) for s in scenarios]
        offset = (i - len(planners)/2 + 0.5) * width
        bars = ax.bar(x + offset, values, width, label=planner, color=colors[i])

        # Add value labels on bars
        for bar, val in zip(bars, values):
            if val > 0:
                ax.annotate(f'{val}',
                           xy=(bar.get_x() + bar.get_width() / 2, bar.get_height()),
                           xytext=(0, 3),
                           textcoords="offset points",
                           ha='center', va='bottom', fontsize=9)

    ax.set_xlabel('Scenario', fontsize=12)
    ax.set_ylabel('Max Robots (≥50% success)', fontsize=12)
    ax.set_title('Maximum Robot Count by Planner and Scenario', fontsize=14)
    ax.set_xticks(x)
    ax.set_xticklabels(scenarios, rotation=45, ha='right')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    plt.savefig(output_dir / 'max_robots_bar.png', dpi=150)
    plt.close()

    print("Saved: max_robots_bar.png")


def plot_heatmap(data, output_dir, min_success_rate=0.5):
    """Heatmap showing max robots per (scenario, planner) combination."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Find max robots for each (scenario, planner)
    max_robots = {}
    for row in data:
        if row['success_rate'] >= min_success_rate:
            key = (row['scenario'], row['planner'])
            current = max_robots.get(key, 0)
            max_robots[key] = max(current, row['num_robots'])

    if not max_robots:
        print("No data for heatmap")
        return

    scenarios = sorted(set(k[0] for k in max_robots.keys()))
    planners = sorted(set(k[1] for k in max_robots.keys()))

    # Create matrix
    matrix = np.zeros((len(planners), len(scenarios)))
    for i, planner in enumerate(planners):
        for j, scenario in enumerate(scenarios):
            matrix[i, j] = max_robots.get((scenario, planner), 0)

    fig, ax = plt.subplots(figsize=(10, 6))

    im = ax.imshow(matrix, cmap='YlOrRd', aspect='auto')

    # Add colorbar
    cbar = ax.figure.colorbar(im, ax=ax)
    cbar.ax.set_ylabel('Max Robots', rotation=-90, va="bottom")

    # Set ticks
    ax.set_xticks(np.arange(len(scenarios)))
    ax.set_yticks(np.arange(len(planners)))
    ax.set_xticklabels(scenarios, rotation=45, ha='right')
    ax.set_yticklabels(planners)

    # Add text annotations
    for i in range(len(planners)):
        for j in range(len(scenarios)):
            val = int(matrix[i, j])
            if val > 0:
                text = ax.text(j, i, val, ha="center", va="center",
                              color="white" if val > matrix.max()/2 else "black")

    ax.set_title('Max Robots per Planner/Scenario (≥50% success rate)', fontsize=14)
    ax.set_xlabel('Scenario')
    ax.set_ylabel('Planner')

    plt.tight_layout()
    plt.savefig(output_dir / 'heatmap_max_robots.png', dpi=150)
    plt.close()

    print("Saved: heatmap_max_robots.png")


def generate_all_plots(data, output_dir):
    """Generate all plot types."""
    plot_success_rate(data, output_dir, by_scenario=True)
    plot_planning_time(data, output_dir)
    plot_max_robots_bar(data, output_dir)
    plot_heatmap(data, output_dir)


def main():
    parser = argparse.ArgumentParser(
        description='Generate plots from experiment results',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument('input', help='Aggregated CSV/JSON or results directory')
    parser.add_argument('--output', '-o', default='plots',
                       help='Output directory for plots')
    parser.add_argument('--type', '-t',
                       choices=['all', 'success_rate', 'planning_time', 'max_robots', 'heatmap'],
                       default='all', help='Type of plot to generate')

    args = parser.parse_args()

    # Load data
    data = load_aggregated_data(args.input)

    if not data:
        print("No data to plot")
        return

    output_dir = Path(args.output)

    if args.type == 'all':
        generate_all_plots(data, output_dir)
    elif args.type == 'success_rate':
        plot_success_rate(data, output_dir)
    elif args.type == 'planning_time':
        plot_planning_time(data, output_dir)
    elif args.type == 'max_robots':
        plot_max_robots_bar(data, output_dir)
    elif args.type == 'heatmap':
        plot_heatmap(data, output_dir)


if __name__ == '__main__':
    main()
