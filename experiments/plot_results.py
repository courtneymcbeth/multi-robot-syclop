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

# Geometric planners; everything else is kinodynamic
GEOMETRIC_PLANNERS = {'arc', 'srrt', 'drrt', 'coupled_rrt_geometric',
                      'decoupled_rrt_geometric', 'cipher_geometric'}


def split_by_type(data):
    """Split data into geometric and kinodynamic subsets."""
    geometric = [row for row in data if row['planner'] in GEOMETRIC_PLANNERS]
    kinodynamic = [row for row in data if row['planner'] not in GEOMETRIC_PLANNERS]
    groups = []
    if geometric:
        groups.append(('geometric', geometric))
    if kinodynamic:
        groups.append(('kinodynamic', kinodynamic))
    return groups


# Fixed color mapping so each planner always gets the same color across all plots
PLANNER_COLORS = {
    'arc': '#1f77b4',
    'k_arc': '#1f77b4',
    'coupled_rrt_geometric': '#ff7f0e',
    'coupled_rrt_kinodynamic': '#ff7f0e',
    'decoupled_rrt_geometric': '#d62728',
    'decoupled_rrt_kinodynamic': '#d62728',
    'drrt': '#8c564b',
    'cipher_geometric': '#e377c2',
    'cipher_kinodynamic': '#e377c2',
    'srrt': '#bcbd22',
    'WoDaSH': '#2ca02c',
    'K-WoDaSH': '#17becf',
}

def get_planner_colors(data):
    """Get color mapping for all planners in data, using fixed colors."""
    all_planners = sorted(set(row['planner'] for row in data))
    colors = {}
    # Fallback colors for any planners not in the fixed mapping
    fallback = plt.cm.tab20(np.linspace(0, 1, 20))
    fallback_idx = 0
    for planner in all_planners:
        if planner in PLANNER_COLORS:
            colors[planner] = PLANNER_COLORS[planner]
        else:
            colors[planner] = fallback[fallback_idx % len(fallback)]
            fallback_idx += 1
    return colors


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
                               'mean_path_length', 'std_path_length',
                               'mean_makespan', 'std_makespan']:
                        if key in row:
                            if row[key]:
                                try:
                                    row[key] = float(row[key])
                                except ValueError:
                                    row[key] = None
                            else:
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

    planner_colors = get_planner_colors(data)

    if by_scenario:
        for scenario, scenario_data in scenarios.items():
            for ptype, type_data in split_by_type(scenario_data):
                fig, ax = plt.subplots(figsize=(10, 6))

                # Group by planner
                planners = defaultdict(list)
                for row in type_data:
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
                ax.set_title(f'Success Rate vs. Robot Count - {scenario} ({ptype})', fontsize=14)
                ax.legend(loc='best')
                ax.grid(True, alpha=0.3)
                ax.set_ylim(-5, 105)

                plt.tight_layout()
                fname = f'success_rate_{scenario}_{ptype}.png'
                plt.savefig(output_dir / fname, dpi=150)
                plt.close()

                print(f"Saved: {fname}")
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

    planner_colors = get_planner_colors(data)

    for scenario, scenario_data in scenarios.items():
        for ptype, type_data in split_by_type(scenario_data):
            fig, ax = plt.subplots(figsize=(10, 6))

            planners = defaultdict(list)
            for row in type_data:
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
            ax.set_yscale('log')
            ax.set_title(f'Planning Time vs. Robot Count - {scenario} ({ptype})', fontsize=14)
            ax.legend(loc='best')
            ax.grid(True, alpha=0.3)

            plt.tight_layout()
            fname = f'planning_time_{scenario}_{ptype}.png'
            plt.savefig(output_dir / fname, dpi=150)
            plt.close()

            print(f"Saved: {fname}")


def plot_path_cost_sum(data, output_dir):
    """Plot sum of path costs (total path length) vs. number of robots with error bars."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Group by scenario
    scenarios = defaultdict(list)
    for row in data:
        if row.get('mean_path_length') is not None:
            scenarios[row['scenario']].append(row)

    planner_colors = get_planner_colors(data)

    for scenario, scenario_data in scenarios.items():
        for ptype, type_data in split_by_type(scenario_data):
            fig, ax = plt.subplots(figsize=(10, 6))

            planners = defaultdict(list)
            for row in type_data:
                planners[row['planner']].append(row)

            for planner, planner_data in sorted(planners.items()):
                planner_data.sort(key=lambda x: x['num_robots'])

                robots = [r['num_robots'] for r in planner_data]
                costs = [r['mean_path_length'] for r in planner_data]
                stds = [r.get('std_path_length', 0) or 0 for r in planner_data]

                ax.errorbar(robots, costs, yerr=stds, fmt='o-',
                           label=planner, color=planner_colors[planner],
                           linewidth=2, markersize=8, capsize=4)

            ax.set_xlabel('Number of Robots', fontsize=12)
            ax.set_ylabel('Sum of Path Costs', fontsize=12)
            ax.set_title(f'Sum of Path Costs vs. Robot Count - {scenario} ({ptype})', fontsize=14)
            ax.legend(loc='best')
            ax.grid(True, alpha=0.3)

            plt.tight_layout()
            fname = f'path_cost_sum_{scenario}_{ptype}.png'
            plt.savefig(output_dir / fname, dpi=150)
            plt.close()

            print(f"Saved: {fname}")


def plot_makespan(data, output_dir):
    """Plot makespan (max path cost for a robot) vs. number of robots with error bars."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Group by scenario
    scenarios = defaultdict(list)
    for row in data:
        if row.get('mean_makespan') is not None:
            scenarios[row['scenario']].append(row)

    planner_colors = get_planner_colors(data)

    for scenario, scenario_data in scenarios.items():
        for ptype, type_data in split_by_type(scenario_data):
            fig, ax = plt.subplots(figsize=(10, 6))

            planners = defaultdict(list)
            for row in type_data:
                planners[row['planner']].append(row)

            for planner, planner_data in sorted(planners.items()):
                planner_data.sort(key=lambda x: x['num_robots'])

                robots = [r['num_robots'] for r in planner_data]
                makespans = [r['mean_makespan'] for r in planner_data]
                stds = [r.get('std_makespan', 0) or 0 for r in planner_data]

                ax.errorbar(robots, makespans, yerr=stds, fmt='o-',
                           label=planner, color=planner_colors[planner],
                           linewidth=2, markersize=8, capsize=4)

            ax.set_xlabel('Number of Robots', fontsize=12)
            ax.set_ylabel('Makespan (Max Path Cost)', fontsize=12)
            ax.set_title(f'Makespan vs. Robot Count - {scenario} ({ptype})', fontsize=14)
            ax.legend(loc='best')
            ax.grid(True, alpha=0.3)

            plt.tight_layout()
            fname = f'makespan_{scenario}_{ptype}.png'
            plt.savefig(output_dir / fname, dpi=150)
            plt.close()

            print(f"Saved: {fname}")


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

    planner_colors = get_planner_colors(data)

    for ptype, _ in split_by_type(data):
        type_max = {k: v for k, v in max_robots.items() if
                    (k[1] in GEOMETRIC_PLANNERS) == (ptype == 'geometric')}
        if not type_max:
            continue

        scenarios = sorted(set(k[0] for k in type_max.keys()))
        planners = sorted(set(k[1] for k in type_max.keys()))

        x = np.arange(len(scenarios))
        width = 0.8 / len(planners)

        fig, ax = plt.subplots(figsize=(12, 6))

        for i, planner in enumerate(planners):
            values = [type_max.get((s, planner), 0) for s in scenarios]
            offset = (i - len(planners)/2 + 0.5) * width
            bars = ax.bar(x + offset, values, width, label=planner,
                          color=planner_colors.get(planner, '#333333'))

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
        ax.set_title(f'Maximum Robot Count by Planner and Scenario ({ptype})', fontsize=14)
        ax.set_xticks(x)
        ax.set_xticklabels(scenarios, rotation=45, ha='right')
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3, axis='y')

        plt.tight_layout()
        fname = f'max_robots_bar_{ptype}.png'
        plt.savefig(output_dir / fname, dpi=150)
        plt.close()

        print(f"Saved: {fname}")


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

    for ptype, _ in split_by_type(data):
        type_max = {k: v for k, v in max_robots.items() if
                    (k[1] in GEOMETRIC_PLANNERS) == (ptype == 'geometric')}
        if not type_max:
            continue

        scenarios = sorted(set(k[0] for k in type_max.keys()))
        planners = sorted(set(k[1] for k in type_max.keys()))

        # Create matrix
        matrix = np.zeros((len(planners), len(scenarios)))
        for i, planner in enumerate(planners):
            for j, scenario in enumerate(scenarios):
                matrix[i, j] = type_max.get((scenario, planner), 0)

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
                    ax.text(j, i, val, ha="center", va="center",
                            color="white" if val > matrix.max()/2 else "black")

        ax.set_title(f'Max Robots per Planner/Scenario ({ptype}, ≥50% success rate)', fontsize=14)
        ax.set_xlabel('Scenario')
        ax.set_ylabel('Planner')

        plt.tight_layout()
        fname = f'heatmap_max_robots_{ptype}.png'
        plt.savefig(output_dir / fname, dpi=150)
        plt.close()

        print(f"Saved: {fname}")


def generate_all_plots(data, output_dir):
    """Generate all plot types."""
    plot_success_rate(data, output_dir, by_scenario=True)
    plot_planning_time(data, output_dir)
    plot_path_cost_sum(data, output_dir)
    plot_makespan(data, output_dir)
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
                       choices=['all', 'success_rate', 'planning_time', 'path_cost_sum',
                                'makespan', 'max_robots', 'heatmap'],
                       default='all', help='Type of plot to generate')
    parser.add_argument('--extra-csv', nargs='+',
                       help='Additional aggregated CSV files to merge (e.g., PPL results)')

    args = parser.parse_args()

    # Load data
    data = load_aggregated_data(args.input)

    # Merge extra CSV files if provided
    if args.extra_csv:
        for extra_path in args.extra_csv:
            extra_data = load_aggregated_data(extra_path)
            data.extend(extra_data)
            print(f"Merged {len(extra_data)} rows from {extra_path}")

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
    elif args.type == 'path_cost_sum':
        plot_path_cost_sum(data, output_dir)
    elif args.type == 'makespan':
        plot_makespan(data, output_dir)
    elif args.type == 'max_robots':
        plot_max_robots_bar(data, output_dir)
    elif args.type == 'heatmap':
        plot_heatmap(data, output_dir)


if __name__ == '__main__':
    main()
