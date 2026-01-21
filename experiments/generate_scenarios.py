#!/usr/bin/env python3
"""
Scenario generator for multi-robot planning experiments.

Creates base environment YAML files (obstacles + bounds, no robots).
Robots are added dynamically at experiment runtime.

Usage:
    # Create a random environment
    python generate_scenarios.py --type random --size 30 30 --density 0.15 --name open_30x30

    # Create a corridor environment
    python generate_scenarios.py --type corridor --width 40 --height 8 --name corridor_40x8

    # Create an empty environment
    python generate_scenarios.py --type empty --size 20 20 --name empty_20x20

    # List available scenario types
    python generate_scenarios.py --list-types
"""

import yaml
import random
import argparse
import sys
from pathlib import Path


def create_empty_scenario(width, height, margin=0.0):
    """Create an empty environment with no obstacles."""
    return {
        'environment': {
            'min': [margin, margin],
            'max': [width - margin, height - margin],
            'obstacles': []
        }
    }


def create_random_scenario(width, height, density=0.15, min_obstacle_size=1.0,
                           max_obstacle_size=3.0, seed=None, margin=1.0):
    """
    Create an environment with randomly placed box obstacles.

    Args:
        width, height: Environment dimensions
        density: Target obstacle coverage (0.0 to 1.0)
        min_obstacle_size: Minimum obstacle dimension
        max_obstacle_size: Maximum obstacle dimension
        seed: Random seed for reproducibility
        margin: Margin around environment edges
    """
    if seed is not None:
        random.seed(seed)

    obstacles = []
    total_area = width * height
    target_obstacle_area = total_area * density
    current_obstacle_area = 0

    max_attempts = 1000
    attempts = 0

    while current_obstacle_area < target_obstacle_area and attempts < max_attempts:
        attempts += 1

        # Generate random obstacle
        obs_width = random.uniform(min_obstacle_size, max_obstacle_size)
        obs_height = random.uniform(min_obstacle_size, max_obstacle_size)

        # Random position (keeping within bounds with margin)
        cx = random.uniform(margin + obs_width/2, width - margin - obs_width/2)
        cy = random.uniform(margin + obs_height/2, height - margin - obs_height/2)

        # Check for overlap with existing obstacles (with small gap)
        gap = 0.5
        overlaps = False
        for obs in obstacles:
            existing_cx, existing_cy = obs['center']
            existing_w, existing_h = obs['size']

            # Check if bounding boxes overlap
            if (abs(cx - existing_cx) < (obs_width + existing_w) / 2 + gap and
                abs(cy - existing_cy) < (obs_height + existing_h) / 2 + gap):
                overlaps = True
                break

        if not overlaps:
            obstacles.append({
                'type': 'box',
                'center': [round(cx, 2), round(cy, 2)],
                'size': [round(obs_width, 2), round(obs_height, 2)]
            })
            current_obstacle_area += obs_width * obs_height

    return {
        'environment': {
            'min': [0.0, 0.0],
            'max': [float(width), float(height)],
            'obstacles': obstacles
        }
    }


def create_corridor_scenario(width, height, corridor_width=4.0, num_gaps=2):
    """
    Create a corridor environment with walls and gaps.

    Args:
        width: Total width
        height: Total height (corridor runs along width)
        corridor_width: Width of the passage
        num_gaps: Number of gaps in the walls
    """
    obstacles = []

    # Wall thickness
    wall_thickness = 1.0

    # Calculate corridor position (centered vertically)
    corridor_center_y = height / 2

    # Top wall (with gaps)
    top_wall_y = corridor_center_y + corridor_width / 2 + wall_thickness / 2
    if top_wall_y + wall_thickness / 2 < height:
        wall_top = {
            'type': 'box',
            'center': [width / 2, (top_wall_y + height) / 2],
            'size': [width, height - top_wall_y - wall_thickness / 2]
        }
        if wall_top['size'][1] > 0.1:
            obstacles.append(wall_top)

    # Bottom wall (with gaps)
    bottom_wall_y = corridor_center_y - corridor_width / 2 - wall_thickness / 2
    if bottom_wall_y - wall_thickness / 2 > 0:
        wall_bottom = {
            'type': 'box',
            'center': [width / 2, bottom_wall_y / 2],
            'size': [width, bottom_wall_y - wall_thickness / 2]
        }
        if wall_bottom['size'][1] > 0.1:
            obstacles.append(wall_bottom)

    # Add vertical obstacles in the corridor to create gaps
    if num_gaps > 0:
        segment_width = width / (num_gaps + 1)
        gap_size = corridor_width * 0.6  # Gap slightly smaller than corridor

        for i in range(1, num_gaps + 1):
            obs_x = i * segment_width
            # Alternate between top and bottom obstacles
            if i % 2 == 1:
                obs_y = corridor_center_y + corridor_width / 4
            else:
                obs_y = corridor_center_y - corridor_width / 4

            obstacles.append({
                'type': 'box',
                'center': [round(obs_x, 2), round(obs_y, 2)],
                'size': [wall_thickness, round(corridor_width / 2 - 0.5, 2)]
            })

    return {
        'environment': {
            'min': [0.0, 0.0],
            'max': [float(width), float(height)],
            'obstacles': obstacles
        }
    }


def create_grid_scenario(width, height, grid_size=4, passage_width=2.0):
    """
    Create a grid-like environment with regular passages.

    Args:
        width, height: Environment dimensions
        grid_size: Number of cells in each dimension
        passage_width: Width of passages between cells
    """
    obstacles = []

    cell_width = width / grid_size
    cell_height = height / grid_size
    wall_thickness = min(cell_width, cell_height) - passage_width

    if wall_thickness < 0.5:
        wall_thickness = 0.5

    # Create intersection posts
    for i in range(1, grid_size):
        for j in range(1, grid_size):
            cx = i * cell_width
            cy = j * cell_height
            obstacles.append({
                'type': 'box',
                'center': [round(cx, 2), round(cy, 2)],
                'size': [round(wall_thickness, 2), round(wall_thickness, 2)]
            })

    return {
        'environment': {
            'min': [0.0, 0.0],
            'max': [float(width), float(height)],
            'obstacles': obstacles
        }
    }


def create_rooms_scenario(width, height, num_rooms_x=2, num_rooms_y=2, door_width=2.0):
    """
    Create a rooms environment with doors between adjacent rooms.
    """
    obstacles = []
    wall_thickness = 0.5

    room_width = width / num_rooms_x
    room_height = height / num_rooms_y

    # Vertical walls
    for i in range(1, num_rooms_x):
        wall_x = i * room_width
        for j in range(num_rooms_y):
            # Wall segment above door
            door_center_y = (j + 0.5) * room_height
            segment_height = (room_height - door_width) / 2

            if segment_height > 0.1:
                # Bottom segment
                obstacles.append({
                    'type': 'box',
                    'center': [round(wall_x, 2), round(j * room_height + segment_height / 2, 2)],
                    'size': [wall_thickness, round(segment_height, 2)]
                })
                # Top segment
                obstacles.append({
                    'type': 'box',
                    'center': [round(wall_x, 2), round((j + 1) * room_height - segment_height / 2, 2)],
                    'size': [wall_thickness, round(segment_height, 2)]
                })

    # Horizontal walls
    for j in range(1, num_rooms_y):
        wall_y = j * room_height
        for i in range(num_rooms_x):
            door_center_x = (i + 0.5) * room_width
            segment_width = (room_width - door_width) / 2

            if segment_width > 0.1:
                # Left segment
                obstacles.append({
                    'type': 'box',
                    'center': [round(i * room_width + segment_width / 2, 2), round(wall_y, 2)],
                    'size': [round(segment_width, 2), wall_thickness]
                })
                # Right segment
                obstacles.append({
                    'type': 'box',
                    'center': [round((i + 1) * room_width - segment_width / 2, 2), round(wall_y, 2)],
                    'size': [round(segment_width, 2), wall_thickness]
                })

    return {
        'environment': {
            'min': [0.0, 0.0],
            'max': [float(width), float(height)],
            'obstacles': obstacles
        }
    }


SCENARIO_TYPES = {
    'empty': create_empty_scenario,
    'random': create_random_scenario,
    'corridor': create_corridor_scenario,
    'grid': create_grid_scenario,
    'rooms': create_rooms_scenario,
}


def save_scenario(scenario, output_path, visualize=False):
    """Save scenario to YAML file and optionally visualize."""
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, 'w') as f:
        yaml.dump(scenario, f, default_flow_style=None, sort_keys=False)

    print(f"Saved scenario to: {output_path}")

    if visualize:
        try:
            from visualize_problem import visualize_environment
            png_path = output_path.with_suffix('.png')
            visualize_environment(str(output_path), output_file=str(png_path),
                                title=output_path.stem)
        except ImportError:
            print("Warning: Could not import visualize_problem for visualization")


def main():
    parser = argparse.ArgumentParser(
        description='Generate scenario files for multi-robot planning experiments',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument('--type', '-t', choices=list(SCENARIO_TYPES.keys()),
                       help='Type of scenario to generate')
    parser.add_argument('--name', '-n', required=False,
                       help='Name for the scenario (used as filename)')
    parser.add_argument('--output-dir', '-d', default='scenarios',
                       help='Output directory (default: scenarios)')

    # Dimension arguments
    parser.add_argument('--size', type=float, nargs=2, metavar=('WIDTH', 'HEIGHT'),
                       help='Environment size (width height)')
    parser.add_argument('--width', type=float, help='Environment width')
    parser.add_argument('--height', type=float, help='Environment height')

    # Random scenario arguments
    parser.add_argument('--density', type=float, default=0.15,
                       help='Obstacle density for random scenarios (0.0-1.0)')
    parser.add_argument('--seed', type=int, help='Random seed for reproducibility')

    # Corridor arguments
    parser.add_argument('--corridor-width', type=float, default=4.0,
                       help='Corridor width')
    parser.add_argument('--num-gaps', type=int, default=2,
                       help='Number of gaps in corridor')

    # Grid arguments
    parser.add_argument('--grid-size', type=int, default=4,
                       help='Grid size (cells per dimension)')
    parser.add_argument('--passage-width', type=float, default=2.0,
                       help='Passage width for grid scenarios')

    # Rooms arguments
    parser.add_argument('--rooms-x', type=int, default=2,
                       help='Number of rooms in x direction')
    parser.add_argument('--rooms-y', type=int, default=2,
                       help='Number of rooms in y direction')
    parser.add_argument('--door-width', type=float, default=2.0,
                       help='Door width for rooms scenarios')

    # Other options
    parser.add_argument('--visualize', '-v', action='store_true',
                       help='Generate visualization image')
    parser.add_argument('--list-types', action='store_true',
                       help='List available scenario types')

    args = parser.parse_args()

    if args.list_types:
        print("Available scenario types:")
        for name in SCENARIO_TYPES:
            print(f"  - {name}")
        return

    if not args.type:
        parser.error("--type is required")

    # Determine dimensions
    if args.size:
        width, height = args.size
    elif args.width and args.height:
        width, height = args.width, args.height
    else:
        # Defaults based on type
        if args.type == 'corridor':
            width, height = 40.0, 10.0
        else:
            width, height = 20.0, 20.0

    # Generate scenario
    if args.type == 'empty':
        scenario = create_empty_scenario(width, height)
    elif args.type == 'random':
        scenario = create_random_scenario(width, height, density=args.density,
                                         seed=args.seed)
    elif args.type == 'corridor':
        scenario = create_corridor_scenario(width, height,
                                           corridor_width=args.corridor_width,
                                           num_gaps=args.num_gaps)
    elif args.type == 'grid':
        scenario = create_grid_scenario(width, height,
                                       grid_size=args.grid_size,
                                       passage_width=args.passage_width)
    elif args.type == 'rooms':
        scenario = create_rooms_scenario(width, height,
                                        num_rooms_x=args.rooms_x,
                                        num_rooms_y=args.rooms_y,
                                        door_width=args.door_width)

    # Determine output name
    if args.name:
        name = args.name
    else:
        name = f"{args.type}_{int(width)}x{int(height)}"

    output_path = Path(args.output_dir) / f"{name}.yaml"
    save_scenario(scenario, output_path, visualize=args.visualize)


if __name__ == '__main__':
    main()
