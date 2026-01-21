#!/usr/bin/env python3
"""
Aggregate experiment results across seeds.

Computes statistics (mean, std, success rate) grouped by
(scenario, planner, num_robots).

Usage:
    # Aggregate from results directory
    python aggregate_results.py results/

    # Output to CSV
    python aggregate_results.py results/ -o analysis/summary.csv

    # Use pre-parsed JSON
    python aggregate_results.py results_parsed.json -o summary.csv
"""

import json
import csv
import argparse
import statistics
from pathlib import Path
from collections import defaultdict

from parse_results import parse_results_directory


def aggregate_results(results):
    """
    Aggregate results by (scenario, planner, num_robots).

    Returns:
        List of dicts with aggregated statistics
    """
    # Group results
    groups = defaultdict(list)

    for r in results:
        if 'error' in r:
            continue

        key = (
            r.get('scenario', 'unknown'),
            r.get('planner', 'unknown'),
            r.get('num_robots_config', 0)
        )
        groups[key].append(r)

    # Compute statistics for each group
    aggregated = []

    for (scenario, planner, num_robots), group_results in sorted(groups.items()):
        seeds_run = len(group_results)
        seeds_solved = sum(1 for r in group_results if r.get('solved', False))
        success_rate = seeds_solved / seeds_run if seeds_run > 0 else 0

        # Planning times (only for successful runs for meaningful stats)
        successful_times = [r['planning_time'] for r in group_results
                          if r.get('solved', False) and 'planning_time' in r]
        all_times = [r['planning_time'] for r in group_results if 'planning_time' in r]

        # Path lengths (only for successful runs)
        path_lengths = [r['total_path_length'] for r in group_results
                       if r.get('solved', False) and 'total_path_length' in r]

        # Makespans (only for successful runs)
        makespans = [r['makespan'] for r in group_results
                    if r.get('solved', False) and 'makespan' in r]

        agg = {
            'scenario': scenario,
            'planner': planner,
            'num_robots': num_robots,
            'seeds_run': seeds_run,
            'seeds_solved': seeds_solved,
            'success_rate': round(success_rate, 4),
        }

        # Time statistics
        if successful_times:
            agg['mean_time'] = round(statistics.mean(successful_times), 4)
            agg['median_time'] = round(statistics.median(successful_times), 4)
            if len(successful_times) > 1:
                agg['std_time'] = round(statistics.stdev(successful_times), 4)
            else:
                agg['std_time'] = 0.0
            agg['min_time'] = round(min(successful_times), 4)
            agg['max_time'] = round(max(successful_times), 4)
        else:
            agg['mean_time'] = None
            agg['median_time'] = None
            agg['std_time'] = None
            agg['min_time'] = None
            agg['max_time'] = None

        # Path length statistics
        if path_lengths:
            agg['mean_path_length'] = round(statistics.mean(path_lengths), 4)
            if len(path_lengths) > 1:
                agg['std_path_length'] = round(statistics.stdev(path_lengths), 4)
            else:
                agg['std_path_length'] = 0.0
        else:
            agg['mean_path_length'] = None
            agg['std_path_length'] = None

        # Makespan statistics
        if makespans:
            agg['mean_makespan'] = round(statistics.mean(makespans), 2)
            if len(makespans) > 1:
                agg['std_makespan'] = round(statistics.stdev(makespans), 2)
            else:
                agg['std_makespan'] = 0.0
        else:
            agg['mean_makespan'] = None
            agg['std_makespan'] = None

        aggregated.append(agg)

    return aggregated


def save_to_csv(aggregated, output_path):
    """Save aggregated results to CSV."""
    if not aggregated:
        print("No results to save")
        return

    fieldnames = [
        'scenario', 'planner', 'num_robots',
        'seeds_run', 'seeds_solved', 'success_rate',
        'mean_time', 'std_time', 'median_time', 'min_time', 'max_time',
        'mean_path_length', 'std_path_length',
        'mean_makespan', 'std_makespan'
    ]

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
        writer.writeheader()
        for row in aggregated:
            writer.writerow(row)

    print(f"Saved aggregated results to {output_path}")


def save_to_json(aggregated, output_path):
    """Save aggregated results to JSON."""
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, 'w') as f:
        json.dump(aggregated, f, indent=2)

    print(f"Saved aggregated results to {output_path}")


def print_summary_table(aggregated):
    """Print a formatted summary table."""
    if not aggregated:
        print("No results to display")
        return

    # Header
    print("\n" + "=" * 100)
    print(f"{'Scenario':<20} {'Planner':<15} {'Robots':>6} {'Success':>8} "
          f"{'Mean Time':>10} {'Std Time':>10} {'Path Len':>10}")
    print("=" * 100)

    current_scenario = None
    for row in aggregated:
        # Add separator between scenarios
        if current_scenario and row['scenario'] != current_scenario:
            print("-" * 100)
        current_scenario = row['scenario']

        success_str = f"{row['seeds_solved']}/{row['seeds_run']} ({row['success_rate']:.0%})"
        time_str = f"{row['mean_time']:.2f}s" if row['mean_time'] is not None else "-"
        std_str = f"±{row['std_time']:.2f}" if row['std_time'] is not None else ""
        path_str = f"{row['mean_path_length']:.1f}" if row['mean_path_length'] is not None else "-"

        print(f"{row['scenario']:<20} {row['planner']:<15} {row['num_robots']:>6} "
              f"{success_str:>8} {time_str:>10} {std_str:>10} {path_str:>10}")

    print("=" * 100 + "\n")


def find_max_robots(aggregated, min_success_rate=0.5):
    """
    Find the maximum number of robots each planner can handle per scenario.

    Args:
        aggregated: Aggregated results
        min_success_rate: Minimum success rate to consider "successful"

    Returns:
        Dict mapping (scenario, planner) -> max_robots
    """
    max_robots = {}

    for row in aggregated:
        key = (row['scenario'], row['planner'])
        if row['success_rate'] >= min_success_rate:
            current_max = max_robots.get(key, 0)
            max_robots[key] = max(current_max, row['num_robots'])

    return max_robots


def main():
    parser = argparse.ArgumentParser(
        description='Aggregate experiment results across seeds',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument('input', help='Results directory or pre-parsed JSON file')
    parser.add_argument('--output', '-o', help='Output file (CSV or JSON)')
    parser.add_argument('--format', '-f', choices=['csv', 'json', 'print'],
                       default='print', help='Output format')

    args = parser.parse_args()

    input_path = Path(args.input)

    # Load results
    if input_path.is_file() and input_path.suffix == '.json':
        with open(input_path, 'r') as f:
            results = json.load(f)
    elif input_path.is_dir():
        results = parse_results_directory(input_path)
    else:
        print(f"Error: Input not found or invalid: {input_path}")
        return

    # Aggregate
    aggregated = aggregate_results(results)

    # Output
    if args.output:
        output_path = Path(args.output)
        if output_path.suffix == '.csv':
            save_to_csv(aggregated, output_path)
        elif output_path.suffix == '.json':
            save_to_json(aggregated, output_path)
        else:
            save_to_csv(aggregated, output_path)
    elif args.format == 'csv':
        save_to_csv(aggregated, 'summary.csv')
    elif args.format == 'json':
        save_to_json(aggregated, 'summary.json')
    else:
        print_summary_table(aggregated)

        # Print max robots summary
        max_robots = find_max_robots(aggregated)
        if max_robots:
            print("\nMax robots with ≥50% success rate:")
            for (scenario, planner), robots in sorted(max_robots.items()):
                print(f"  {scenario}/{planner}: {robots} robots")


if __name__ == '__main__':
    main()
