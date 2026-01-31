#!/usr/bin/env python3
"""
Extract PPL experiment results and aggregate them for plotting.

Scans PPL experiment directories, extracts timing and path cost metrics,
and outputs an aggregated CSV compatible with plot_results.py.

Supports multiple strategies (e.g., WoDaSH, K-WoDaSH) in a single run.

Usage:
    # Extract both WoDaSH and K-WoDaSH (default)
    python extract_ppl_results.py \
      --ppl-dir /path/to/PPL/multi-robot-narrow-passage-experiments \
      -o ppl_results.csv

    # Extract specific strategies
    python extract_ppl_results.py \
      --ppl-dir /path/to/PPL/multi-robot-narrow-passage-experiments \
      --strategies WoDaSH \
      -o ppl_results.csv
"""

import csv
import math
import re
import argparse
import statistics
from pathlib import Path
from collections import defaultdict


def parse_scenario_dir_name(dirname):
    """
    Parse a PPL scenario directory name into base scenario and seed number.

    Examples:
        'random5_20x20_seed0' -> ('random5_20x20', 0)
        'grid5_20x20_seed12' -> ('grid5_20x20', 12)
        'Warehouse' -> ('Warehouse', None)  (legacy format)
    """
    match = re.match(r'^(.+)_seed(\d+)$', dirname)
    if match:
        return match.group(1), int(match.group(2))
    return dirname, None


def read_planner_seeds(ppl_dir):
    """Read planner seeds from the seeds file."""
    seeds_file = ppl_dir / 'seeds'
    if seeds_file.exists():
        with open(seeds_file, 'r') as f:
            return [line.strip() for line in f if line.strip()]
    return ['51075276']  # default


def extract_planning_time(stat_file):
    """
    Extract TotalTMPStrategyTime from a PPL stat file.

    Returns:
        float or None if not found
    """
    try:
        with open(stat_file, 'r') as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 2 and parts[0] == 'TotalTMPStrategyTime':
                    return float(parts[-1])
    except (OSError, ValueError):
        pass
    return None


def extract_path_costs(path_file):
    """
    Extract path costs from a PPL solution path file.

    The path file format:
        VIZMO_PATH_FILE   Path Version 2012
        1
        <num_timesteps>
        0  x1 y1 θ1  x2 y2 θ2  ...  xN yN θN
        ...

    Returns:
        (total_path_length, makespan) or (None, None) if file can't be parsed.
        total_path_length: sum of Euclidean distances for all robots
        makespan: max individual robot Euclidean path length
    """
    try:
        with open(path_file, 'r') as f:
            lines = f.readlines()
    except OSError:
        return None, None

    if len(lines) < 4:
        return None, None

    # Skip header (3 lines), parse data lines
    data_lines = lines[3:]

    # Parse all timesteps into per-robot positions
    robot_positions = None  # Will be list of lists: robot_positions[robot_idx] = [(x, y), ...]

    for line in data_lines:
        parts = line.split()
        if len(parts) < 4:
            continue

        # Skip prefix '0', remaining values are groups of 3 (x, y, θ) per robot
        values = [float(v) for v in parts[1:]]
        num_robots = len(values) // 3

        if num_robots < 1:
            continue

        if robot_positions is None:
            robot_positions = [[] for _ in range(num_robots)]

        for r in range(num_robots):
            x = values[r * 3]
            y = values[r * 3 + 1]
            robot_positions[r].append((x, y))

    if robot_positions is None or all(len(rp) < 2 for rp in robot_positions):
        return None, None

    # Compute per-robot Euclidean path lengths
    robot_path_lengths = []
    for positions in robot_positions:
        length = 0.0
        for i in range(1, len(positions)):
            dx = positions[i][0] - positions[i - 1][0]
            dy = positions[i][1] - positions[i - 1][1]
            length += math.sqrt(dx * dx + dy * dy)
        robot_path_lengths.append(length)

    total_path_length = sum(robot_path_lengths)
    makespan = max(robot_path_lengths)

    return total_path_length, makespan


def discover_runs(ppl_dir, strategies, planner_seeds):
    """
    Scan PPL directory structure and discover all experiment runs.

    Returns:
        List of dicts with run metadata and results
    """
    ppl_dir = Path(ppl_dir)
    runs = []

    # Find all scenario_seed directories
    for scenario_dir in sorted(ppl_dir.iterdir()):
        if not scenario_dir.is_dir():
            continue

        dirname = scenario_dir.name
        base_scenario, seed_num = parse_scenario_dir_name(dirname)

        # Skip non-experiment directories (envs, etc.)
        if base_scenario in ('envs', 'objs', 'skeletons'):
            continue

        # Scan robot count subdirectories
        for robot_dir in sorted(scenario_dir.iterdir()):
            if not robot_dir.is_dir():
                continue

            try:
                num_robots = int(robot_dir.name)
            except ValueError:
                continue

            for strategy in strategies:
                strategy_dir = robot_dir / strategy
                if not strategy_dir.is_dir():
                    continue

                for planner_seed in planner_seeds:
                    out_file = strategy_dir / f'{planner_seed}.out'
                    stat_file = strategy_dir / f'{strategy}.{planner_seed}-ppl.stat'
                    path_file = strategy_dir / f'{strategy}.{planner_seed}.path'

                    attempted = out_file.exists()
                    succeeded = stat_file.exists()

                    if not attempted:
                        continue

                    run = {
                        'base_scenario': base_scenario,
                        'seed': seed_num,
                        'num_robots': num_robots,
                        'strategy': strategy,
                        'planner_seed': planner_seed,
                        'attempted': attempted,
                        'succeeded': succeeded,
                    }

                    if succeeded:
                        # Extract planning time
                        planning_time = extract_planning_time(stat_file)
                        run['planning_time'] = planning_time

                        # Extract path costs
                        total_path_length, makespan = extract_path_costs(path_file)
                        run['total_path_length'] = total_path_length
                        run['makespan'] = makespan

                    runs.append(run)

    return runs


def aggregate_runs(runs):
    """
    Aggregate individual runs by (base_scenario, strategy, num_robots).

    Returns:
        List of dicts with aggregated statistics matching aggregate_results.py format
    """
    groups = defaultdict(list)

    for r in runs:
        key = (r['base_scenario'], r['strategy'], r['num_robots'])
        groups[key].append(r)

    aggregated = []

    for (scenario, strategy, num_robots), group_runs in sorted(groups.items()):
        seeds_run = len(group_runs)
        seeds_solved = sum(1 for r in group_runs if r['succeeded'])
        success_rate = seeds_solved / seeds_run if seeds_run > 0 else 0

        # Planning times (successful runs only)
        successful_times = [
            r['planning_time'] for r in group_runs
            if r['succeeded'] and r.get('planning_time') is not None
        ]

        # Path lengths (successful runs only)
        path_lengths = [
            r['total_path_length'] for r in group_runs
            if r['succeeded'] and r.get('total_path_length') is not None
        ]

        # Makespans (successful runs only)
        makespans = [
            r['makespan'] for r in group_runs
            if r['succeeded'] and r.get('makespan') is not None
        ]

        agg = {
            'scenario': scenario,
            'planner': strategy,
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


def print_summary_table(aggregated):
    """Print a formatted summary table."""
    if not aggregated:
        print("No results to display")
        return

    print("\n" + "=" * 110)
    print(f"{'Scenario':<20} {'Planner':<12} {'Robots':>6} {'Success':>12} "
          f"{'Mean Time':>10} {'Std Time':>10} {'Path Len':>10} {'Makespan':>10}")
    print("=" * 110)

    current_scenario = None
    for row in aggregated:
        if current_scenario and row['scenario'] != current_scenario:
            print("-" * 110)
        current_scenario = row['scenario']

        success_str = f"{row['seeds_solved']}/{row['seeds_run']} ({row['success_rate']:.0%})"
        time_str = f"{row['mean_time']:.3f}s" if row['mean_time'] is not None else "-"
        std_str = f"±{row['std_time']:.3f}" if row['std_time'] is not None else ""
        path_str = f"{row['mean_path_length']:.1f}" if row['mean_path_length'] is not None else "-"
        makespan_str = f"{row['mean_makespan']:.1f}" if row['mean_makespan'] is not None else "-"

        print(f"{row['scenario']:<20} {row['planner']:<12} {row['num_robots']:>6} "
              f"{success_str:>12} {time_str:>10} {std_str:>10} {path_str:>10} {makespan_str:>10}")

    print("=" * 110 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description='Extract PPL experiment results for plotting',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument('--ppl-dir', required=True,
                        help='Path to PPL experiments base directory')
    parser.add_argument('--strategies', nargs='+', default=['WoDaSH', 'K-WoDaSH'],
                        help='Strategies to extract (default: WoDaSH K-WoDaSH)')
    parser.add_argument('--output', '-o', help='Output CSV file')
    parser.add_argument('--format', '-f', choices=['csv', 'print'],
                        default='print', help='Output format (default: print)')

    args = parser.parse_args()

    ppl_dir = Path(args.ppl_dir)
    if not ppl_dir.is_dir():
        print(f"Error: Directory not found: {ppl_dir}")
        return

    # Read planner seeds
    planner_seeds = read_planner_seeds(ppl_dir)
    print(f"Planner seeds: {planner_seeds}")
    print(f"Strategies: {args.strategies}")

    # Discover and extract all runs
    runs = discover_runs(ppl_dir, args.strategies, planner_seeds)
    print(f"Found {len(runs)} total runs "
          f"({sum(1 for r in runs if r['succeeded'])} succeeded)")

    # Aggregate
    aggregated = aggregate_runs(runs)

    # Output
    if args.output:
        save_to_csv(aggregated, args.output)
    elif args.format == 'csv':
        save_to_csv(aggregated, 'ppl_results.csv')
    else:
        print_summary_table(aggregated)


if __name__ == '__main__':
    main()
