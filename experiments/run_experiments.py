#!/usr/bin/env python3
"""
Experiment runner for multi-robot planning benchmarks.

Features:
- Adaptive scaling: increases robot count until all seeds fail
- Deterministic robot placement based on seed
- Timeout handling
- Resume capability
- Progress logging
- Verbose mode with detailed failure diagnostics

Usage:
    # Run all experiments defined in config
    python run_experiments.py

    # Run specific scenarios and planners
    python run_experiments.py --scenarios open_20x20 corridor --planners arc mr_syclop

    # Quick test with fewer seeds
    python run_experiments.py --seeds 3 --timeout 60

    # Resume from previous run
    python run_experiments.py --resume

    # Verbose mode: show detailed failure info and save logs
    python run_experiments.py --verbose
    python run_experiments.py -v
"""

import yaml
import subprocess
import random
import shutil
import json
import argparse
import sys
import time
import hashlib
from pathlib import Path
from datetime import datetime
from concurrent.futures import ProcessPoolExecutor, as_completed
import tempfile
import fcl  # For collision checking - may need to handle import error


# Default robot geometry for collision-free placement
DEFAULT_ROBOT_RADIUS = 0.5


def load_config(config_path='config.yaml'):
    """Load experiment configuration."""
    config_path = Path(config_path)
    if not config_path.exists():
        print(f"Error: Config file not found: {config_path}")
        sys.exit(1)

    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def load_scenario(scenario_path):
    """Load a scenario YAML file."""
    with open(scenario_path, 'r') as f:
        return yaml.safe_load(f)


def compute_config_hash(config_dict):
    """Compute a hash of the configuration dictionary."""
    config_json = json.dumps(config_dict, sort_keys=True, default=str)
    return hashlib.md5(config_json.encode()).hexdigest()


def point_in_obstacle(x, y, obstacles, radius=0.0):
    """Check if a point (with radius) collides with any obstacle."""
    for obs in obstacles:
        if obs.get('type') != 'box':
            continue
        cx, cy = obs['center']
        w, h = obs['size']

        # Expand obstacle by robot radius
        half_w = w / 2 + radius
        half_h = h / 2 + radius

        if (cx - half_w <= x <= cx + half_w and
            cy - half_h <= y <= cy + half_h):
            return True
    return False


def robots_collide(x1, y1, x2, y2, radius):
    """Check if two robots at given positions collide."""
    dist = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
    return dist < 2 * radius


def generate_robot_positions(scenario, num_robots, seed, robot_type='unicycle_first_order_0_sphere',
                            robot_radius=None, max_attempts=1000):
    """
    Generate collision-free start and goal positions for robots.

    Uses the seed for deterministic placement.
    """
    random.seed(seed)

    env = scenario['environment']
    env_min = env['min']
    env_max = env['max']
    obstacles = env.get('obstacles', [])

    if robot_radius is None:
        # Get radius from robot type
        from visualize_problem import DEFAULT_GEOMETRIES
        if robot_type in DEFAULT_GEOMETRIES:
            geom = DEFAULT_GEOMETRIES[robot_type][0]
            if geom[0] == 'sphere':
                robot_radius = geom[1]
            else:
                robot_radius = 0.5 * (geom[1] ** 2 + geom[2] ** 2) ** 0.5
        else:
            robot_radius = DEFAULT_ROBOT_RADIUS

    # Margin from environment edges
    margin = robot_radius + 0.5

    robots = []

    for i in range(num_robots):
        # Generate start position
        for attempt in range(max_attempts):
            start_x = random.uniform(env_min[0] + margin, env_max[0] - margin)
            start_y = random.uniform(env_min[1] + margin, env_max[1] - margin)
            start_theta = random.uniform(-3.14159, 3.14159)

            # Check obstacle collision
            if point_in_obstacle(start_x, start_y, obstacles, robot_radius):
                continue

            # Check collision with other robots' starts
            collision = False
            for other in robots:
                if robots_collide(start_x, start_y, other['start'][0], other['start'][1], robot_radius):
                    collision = True
                    break
            if collision:
                continue

            break
        else:
            raise RuntimeError(f"Could not place start for robot {i} after {max_attempts} attempts")

        # Generate goal position
        for attempt in range(max_attempts):
            goal_x = random.uniform(env_min[0] + margin, env_max[0] - margin)
            goal_y = random.uniform(env_min[1] + margin, env_max[1] - margin)
            goal_theta = random.uniform(-3.14159, 3.14159)

            # Check obstacle collision
            if point_in_obstacle(goal_x, goal_y, obstacles, robot_radius):
                continue

            # Check collision with other robots' goals
            collision = False
            for other in robots:
                if robots_collide(goal_x, goal_y, other['goal'][0], other['goal'][1], robot_radius):
                    collision = True
                    break
            if collision:
                continue

            # Ensure start and goal are not too close
            if robots_collide(start_x, start_y, goal_x, goal_y, robot_radius * 2):
                continue

            break
        else:
            raise RuntimeError(f"Could not place goal for robot {i} after {max_attempts} attempts")

        robots.append({
            'type': robot_type,
            'start': [round(start_x, 4), round(start_y, 4), round(start_theta, 4)],
            'goal': [round(goal_x, 4), round(goal_y, 4), round(goal_theta, 4)]
        })

    return robots


def create_problem_file(scenario, robots, output_path):
    """Create a complete problem YAML file with environment and robots."""
    problem = {
        'environment': scenario['environment'],
        'robots': robots
    }

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, 'w') as f:
        yaml.dump(problem, f, default_flow_style=None, sort_keys=False)

    return output_path


def create_config_with_seed(base_config_path, seed, output_path):
    """Create a planner config file with the specified seed."""
    with open(base_config_path, 'r') as f:
        config = yaml.safe_load(f)

    config['seed'] = seed

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=None, sort_keys=False)

    return output_path


def run_planner(executable, problem_file, output_file, config_file, timeout, log_dir=None):
    """
    Run a planner executable and return the result.

    Returns:
        dict with keys: solved, planning_time, timed_out, error, failure_reason, stdout, stderr
    """
    # Resolve all paths to absolute so they work regardless of cwd
    executable = str(Path(executable).resolve())
    problem_file = str(Path(problem_file).resolve())
    output_file = str(Path(output_file).resolve())
    config_file = str(Path(config_file).resolve())

    # Run from the project root so relative paths in configs (e.g. db-CBS/...) resolve correctly
    project_root = str(Path(__file__).resolve().parent.parent)

    cmd = [
        executable,
        '-i', problem_file,
        '-o', output_file,
        '-c', config_file,
        '-t', str(timeout)
    ]

    result = {
        'solved': False,
        'planning_time': timeout,
        'timed_out': False,
        'error': None,
        'failure_reason': None,
        'stdout': '',
        'stderr': ''
    }

    try:
        proc = subprocess.run(
            cmd,
            timeout=timeout + 30,  # Extra buffer for cleanup
            capture_output=True,
            text=True,
            cwd=project_root
        )

        result['stdout'] = proc.stdout
        result['stderr'] = proc.stderr

        # Save logs if log directory provided
        if log_dir:
            log_dir = Path(log_dir)
            log_dir.mkdir(parents=True, exist_ok=True)

            stdout_file = log_dir / f"{Path(output_file).stem}_stdout.txt"
            stderr_file = log_dir / f"{Path(output_file).stem}_stderr.txt"

            with open(stdout_file, 'w') as f:
                f.write(proc.stdout)
            with open(stderr_file, 'w') as f:
                f.write(proc.stderr)

        # Parse output file
        if Path(output_file).exists():
            with open(output_file, 'r') as f:
                output = yaml.safe_load(f)
                result['solved'] = output.get('solved', output.get('success', False))
                result['planning_time'] = output.get('planning_time', timeout)
                result['failure_reason'] = output.get('failure_reason', output.get('error'))
        else:
            result['error'] = 'No output file generated'

    except subprocess.TimeoutExpired as e:
        result['timed_out'] = True
        result['error'] = 'Process timed out'
        if e.stdout:
            result['stdout'] = e.stdout.decode() if isinstance(e.stdout, bytes) else e.stdout
        if e.stderr:
            result['stderr'] = e.stderr.decode() if isinstance(e.stderr, bytes) else e.stderr
    except Exception as e:
        result['error'] = str(e)

    return result


def run_single_experiment(scenario_name, scenario_path, planner_name, planner_config,
                         num_robots, seed, config, base_dir, verbose=False, overwrite=False):
    """Run a single experiment (one scenario, one planner, one robot count, one seed)."""
    # Paths
    problems_dir = base_dir / 'problems' / scenario_name / f'robots_{num_robots}'
    results_dir = base_dir / 'results' / scenario_name / planner_name / f'robots_{num_robots}'
    configs_dir = base_dir / 'configs' / planner_name
    logs_dir = base_dir / 'logs' / scenario_name / planner_name / f'robots_{num_robots}'

    problem_file = problems_dir / f'seed_{seed}.yaml'
    result_file = results_dir / f'seed_{seed}.yaml'
    config_file = configs_dir / f'seed_{seed}.yaml'

    # Skip if result already exists (resume support) and config hasn't changed
    if result_file.exists() and not overwrite:
        with open(result_file, 'r') as f:
            existing = yaml.safe_load(f)
            # Check if planner config has changed by comparing hashes
            with open(planner_config['config'], 'r') as f:
                planner_config_content = yaml.safe_load(f)
            current_config_hash = compute_config_hash(planner_config_content)
            cached_config_hash = existing.get('config_hash')
            
            # Only use cached result if config hash matches
            if cached_config_hash == current_config_hash:
                return {
                    'scenario': scenario_name,
                    'planner': planner_name,
                    'robots': num_robots,
                    'seed': seed,
                    'solved': existing.get('solved', existing.get('success', False)),
                    'planning_time': existing.get('planning_time', 0),
                    'skipped': True
                }

    # Load scenario and generate robots if problem file doesn't exist
    if not problem_file.exists():
        scenario = load_scenario(scenario_path)
        robot_type = config.get('robot_type', 'unicycle_first_order_0_sphere')
        robots = generate_robot_positions(scenario, num_robots, seed, robot_type)
        create_problem_file(scenario, robots, problem_file)

    # Create config with seed
    create_config_with_seed(planner_config['config'], seed, config_file)

    # Ensure results directory exists
    results_dir.mkdir(parents=True, exist_ok=True)

    # Run planner
    timeout = config.get('timeout', 300)
    log_dir = logs_dir if verbose else None
    result = run_planner(
        planner_config['executable'],
        problem_file,
        result_file,
        config_file,
        timeout,
        log_dir=log_dir
    )

    # Add config hash to result for future cache validation
    with open(planner_config['config'], 'r') as f:
        planner_config_content = yaml.safe_load(f)
    result['config_hash'] = compute_config_hash(planner_config_content)

    # Write result with config hash
    if result_file.exists():
        with open(result_file, 'r') as f:
            result_data = yaml.safe_load(f)
        result_data['config_hash'] = result['config_hash']
        with open(result_file, 'w') as f:
            yaml.dump(result_data, f, default_flow_style=None, sort_keys=False)

    return {
        'scenario': scenario_name,
        'planner': planner_name,
        'robots': num_robots,
        'seed': seed,
        'solved': result['solved'],
        'planning_time': result['planning_time'],
        'timed_out': result.get('timed_out', False),
        'error': result.get('error'),
        'failure_reason': result.get('failure_reason'),
        'stdout': result.get('stdout', ''),
        'stderr': result.get('stderr', ''),
        'skipped': False
    }


def run_experiments(config, scenarios=None, planners=None, resume=False, verbose=False, overwrite=False):
    """
    Run all experiments with adaptive scaling.

    For each (scenario, planner) pair:
    1. Start with start_robots (default 2)
    2. Run all seeds
    3. If any seed succeeds, increment robot count
    4. If all seeds fail, stop testing this pair

    Args:
        verbose: If True, print detailed failure information and save logs
    """
    base_dir = Path(config.get('base_dir', '.'))
    num_seeds = config.get('num_seeds', 10)
    start_robots = config.get('start_robots', 2)
    robot_increment = config.get('robot_increment', 2)
    timeout = config.get('timeout', 300)

    # Filter scenarios and planners if specified
    scenario_list = scenarios if scenarios else config.get('scenarios', [])
    planner_dict = config.get('planners', {})
    if planners:
        planner_dict = {k: v for k, v in planner_dict.items() if k in planners}

    # Results summary
    summary = []
    log_file = base_dir / 'experiment_log.txt'

    def log(msg):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        line = f"[{timestamp}] {msg}"
        print(line)
        with open(log_file, 'a') as f:
            f.write(line + '\n')

    log(f"Starting experiments: {len(scenario_list)} scenarios x {len(planner_dict)} planners")
    log(f"Seeds: {num_seeds}, Timeout: {timeout}s, Start robots: {start_robots}")

    scenarios_dir = base_dir / 'scenarios'

    for scenario_name in scenario_list:
        scenario_path = scenarios_dir / f'{scenario_name}.yaml'
        if not scenario_path.exists():
            log(f"Warning: Scenario not found: {scenario_path}")
            continue

        for planner_name, planner_config in planner_dict.items():
            # Check executable exists
            if not Path(planner_config['executable']).exists():
                log(f"Warning: Executable not found: {planner_config['executable']}")
                continue

            log(f"\n{'='*60}")
            log(f"Testing {planner_name} on {scenario_name}")
            log(f"{'='*60}")

            num_robots = start_robots

            while True:
                log(f"\n  Robots: {num_robots}")

                successes = 0
                failures = 0

                for seed in range(num_seeds):
                    result = run_single_experiment(
                        scenario_name, scenario_path,
                        planner_name, planner_config,
                        num_robots, seed, config, base_dir, verbose,
                        overwrite=overwrite
                    )

                    summary.append(result)

                    if result['solved']:
                        successes += 1
                        status = "SOLVED"
                    else:
                        failures += 1
                        status = "FAILED"

                    if result.get('skipped'):
                        status += " (cached)"

                    log(f"    Seed {seed}: {status} ({result['planning_time']:.2f}s)")

                    # Print detailed failure info in verbose mode
                    if verbose and not result['solved'] and not result.get('skipped'):
                        if result.get('failure_reason'):
                            log(f"      Failure reason: {result['failure_reason']}")
                        if result.get('error'):
                            log(f"      Error: {result['error']}")
                        if result.get('stderr') and result['stderr'].strip():
                            log(f"      stderr: {result['stderr'][:200]}...")  # First 200 chars
                        log_path = base_dir / 'logs' / scenario_name / planner_name / f'robots_{num_robots}'
                        if log_path.exists():
                            log(f"      Full logs: {log_path}/seed_{seed}_*.txt")

                success_rate = successes / num_seeds
                log(f"  Results: {successes}/{num_seeds} solved ({success_rate:.1%})")

                # Save intermediate results
                save_summary(summary, base_dir / 'results_summary.json')

                # Check if we should continue scaling
                if successes == 0:
                    log(f"  All seeds failed at {num_robots} robots - stopping scaling")
                    break

                num_robots += robot_increment

    log(f"\n{'='*60}")
    log(f"Experiments complete!")
    log(f"Total runs: {len(summary)}")
    log(f"Results saved to: {base_dir / 'results_summary.json'}")

    return summary


def save_summary(summary, output_path):
    """Save experiment summary to JSON."""
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, 'w') as f:
        json.dump(summary, f, indent=2)


def main():
    parser = argparse.ArgumentParser(
        description='Run multi-robot planning experiments',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument('--config', '-c', default='config.yaml',
                       help='Path to experiment config file')
    parser.add_argument('--scenarios', '-s', nargs='+',
                       help='Specific scenarios to run')
    parser.add_argument('--planners', '-p', nargs='+',
                       help='Specific planners to run')
    parser.add_argument('--seeds', type=int,
                       help='Override number of seeds')
    parser.add_argument('--timeout', type=int,
                       help='Override timeout (seconds)')
    parser.add_argument('--resume', action='store_true',
                       help='Resume from previous run (skip existing results)')
    parser.add_argument('--overwrite', action='store_true',
                       help='Overwrite existing result files instead of using cached results')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Print detailed failure information and save logs')

    args = parser.parse_args()

    # Load config
    config = load_config(args.config)

    # Override config with command line args
    if args.seeds:
        config['num_seeds'] = args.seeds
    if args.timeout:
        config['timeout'] = args.timeout

    # Run experiments
    run_experiments(
        config,
        scenarios=args.scenarios,
        planners=args.planners,
        resume=args.resume,
        verbose=args.verbose,
        overwrite=args.overwrite
    )


if __name__ == '__main__':
    main()
