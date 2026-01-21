#!/usr/bin/env python3
"""
Parse experiment results from YAML output files.

Extracts metrics from planner output and computes derived metrics
like path length and makespan from the states array.

Usage:
    # Parse all results in a directory
    python parse_results.py results/

    # Parse a single result file
    python parse_results.py results/open_20x20/arc/robots_4/seed_0.yaml

    # Output as CSV
    python parse_results.py results/ --output results.csv
"""

import yaml
import json
import csv
import argparse
import math
from pathlib import Path


def compute_path_length(states):
    """
    Compute the total Euclidean path length from a sequence of states.

    Args:
        states: List of [x, y, ...] states

    Returns:
        Total path length
    """
    if not states or len(states) < 2:
        return 0.0

    total = 0.0
    for i in range(1, len(states)):
        dx = states[i][0] - states[i-1][0]
        dy = states[i][1] - states[i-1][1]
        total += math.sqrt(dx*dx + dy*dy)

    return total


def compute_makespan(result):
    """
    Compute makespan as the maximum number of states across all robots.

    This is a proxy for completion time when all robots have the same
    time step duration.

    Args:
        result: List of robot path data with 'states' key

    Returns:
        Maximum number of states (makespan proxy)
    """
    if not result:
        return 0

    max_states = 0
    for robot_data in result:
        if 'states' in robot_data:
            max_states = max(max_states, len(robot_data['states']))

    return max_states


def compute_total_path_length(result):
    """
    Compute sum of path lengths across all robots.

    Args:
        result: List of robot path data with 'states' key

    Returns:
        Sum of all robot path lengths
    """
    if not result:
        return 0.0

    total = 0.0
    for robot_data in result:
        if 'states' in robot_data:
            total += compute_path_length(robot_data['states'])

    return total


def parse_result_file(filepath):
    """
    Parse a single planner output YAML file.

    Returns:
        dict with extracted and computed metrics
    """
    filepath = Path(filepath)

    try:
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
    except Exception as e:
        return {'error': str(e), 'filepath': str(filepath)}

    if not data:
        return {'error': 'Empty file', 'filepath': str(filepath)}

    # Extract basic metrics (handle naming inconsistency)
    solved = data.get('solved', data.get('success', False))
    planning_time = data.get('planning_time', 0.0)

    metrics = {
        'filepath': str(filepath),
        'solved': solved,
        'planning_time': planning_time,
    }

    # Extract additional timing info if available
    if 'roadmap_build_time' in data:
        metrics['roadmap_build_time'] = data['roadmap_build_time']
    if 'total_time' in data:
        metrics['total_time'] = data['total_time']

    # Extract ARC-specific metrics
    if 'num_conflicts_found' in data:
        metrics['num_conflicts_found'] = data['num_conflicts_found']
    if 'num_conflicts_resolved' in data:
        metrics['num_conflicts_resolved'] = data['num_conflicts_resolved']
    if 'num_subproblems_created' in data:
        metrics['num_subproblems_created'] = data['num_subproblems_created']

    # Compute metrics from solution paths
    result = data.get('result', [])
    if result and solved:
        metrics['num_robots'] = len(result)
        metrics['makespan'] = compute_makespan(result)
        metrics['total_path_length'] = round(compute_total_path_length(result), 4)

        # Per-robot path lengths
        path_lengths = []
        for robot_data in result:
            if 'states' in robot_data:
                path_lengths.append(compute_path_length(robot_data['states']))
        metrics['path_lengths'] = [round(pl, 4) for pl in path_lengths]
        if path_lengths:
            metrics['mean_path_length'] = round(sum(path_lengths) / len(path_lengths), 4)
            metrics['max_path_length'] = round(max(path_lengths), 4)

    return metrics


def parse_results_directory(results_dir, pattern='**/*.yaml'):
    """
    Parse all result files in a directory.

    Expected structure:
        results/{scenario}/{planner}/robots_{N}/seed_{S}.yaml

    Returns:
        List of dicts with metrics for each result file
    """
    results_dir = Path(results_dir)
    all_results = []

    for filepath in results_dir.glob(pattern):
        # Extract metadata from path
        parts = filepath.relative_to(results_dir).parts

        metadata = {}
        if len(parts) >= 4:
            metadata['scenario'] = parts[0]
            metadata['planner'] = parts[1]

            # Parse robots_N
            robots_part = parts[2]
            if robots_part.startswith('robots_'):
                try:
                    metadata['num_robots_config'] = int(robots_part.split('_')[1])
                except ValueError:
                    pass

            # Parse seed_S
            seed_part = filepath.stem
            if seed_part.startswith('seed_'):
                try:
                    metadata['seed'] = int(seed_part.split('_')[1])
                except ValueError:
                    pass

        # Parse the result file
        metrics = parse_result_file(filepath)
        metrics.update(metadata)

        all_results.append(metrics)

    return all_results


def results_to_csv(results, output_path):
    """Save results to CSV file."""
    if not results:
        print("No results to save")
        return

    # Determine all keys
    all_keys = set()
    for r in results:
        all_keys.update(r.keys())

    # Order keys sensibly
    key_order = ['scenario', 'planner', 'num_robots_config', 'seed', 'solved',
                 'planning_time', 'makespan', 'total_path_length', 'mean_path_length',
                 'num_conflicts_found', 'num_conflicts_resolved']
    ordered_keys = [k for k in key_order if k in all_keys]
    ordered_keys += sorted(all_keys - set(ordered_keys))

    # Remove complex types (lists)
    ordered_keys = [k for k in ordered_keys if k != 'path_lengths']

    with open(output_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=ordered_keys, extrasaction='ignore')
        writer.writeheader()
        for r in results:
            # Convert any non-serializable values
            row = {k: v for k, v in r.items() if k in ordered_keys}
            writer.writerow(row)

    print(f"Saved {len(results)} results to {output_path}")


def results_to_json(results, output_path):
    """Save results to JSON file."""
    with open(output_path, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"Saved {len(results)} results to {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description='Parse experiment results and extract metrics',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument('input', help='Input file or directory')
    parser.add_argument('--output', '-o', help='Output file (CSV or JSON based on extension)')
    parser.add_argument('--format', '-f', choices=['csv', 'json', 'print'],
                       default='print', help='Output format')

    args = parser.parse_args()

    input_path = Path(args.input)

    if input_path.is_file():
        results = [parse_result_file(input_path)]
    elif input_path.is_dir():
        results = parse_results_directory(input_path)
    else:
        print(f"Error: Input not found: {input_path}")
        return

    # Determine output format
    if args.output:
        output_path = Path(args.output)
        if output_path.suffix == '.csv':
            results_to_csv(results, output_path)
        elif output_path.suffix == '.json':
            results_to_json(results, output_path)
        else:
            # Default to CSV
            results_to_csv(results, output_path)
    elif args.format == 'csv':
        results_to_csv(results, 'results.csv')
    elif args.format == 'json':
        results_to_json(results, 'results.json')
    else:
        # Print summary
        for r in results:
            if 'error' in r:
                print(f"Error parsing {r.get('filepath', 'unknown')}: {r['error']}")
            else:
                solved_str = "SOLVED" if r.get('solved') else "FAILED"
                print(f"{r.get('scenario', '?')}/{r.get('planner', '?')}/robots_{r.get('num_robots_config', '?')}/seed_{r.get('seed', '?')}: "
                      f"{solved_str} in {r.get('planning_time', 0):.2f}s")
                if r.get('solved') and 'total_path_length' in r:
                    print(f"  Path length: {r['total_path_length']:.2f}, Makespan: {r.get('makespan', '?')}")


if __name__ == '__main__':
    main()
