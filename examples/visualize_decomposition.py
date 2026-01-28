#!/usr/bin/env python3
"""
Decomposition visualization script for Multi-Robot SyCLoP

Plots decomposition grids saved by MRSyCLoPPlanner::saveDecompositionToFile()
Can visualize single decomposition files or overlay multiple decompositions.

Requires: matplotlib, pyyaml
Install: pip install matplotlib pyyaml
"""

import yaml
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Slider
import numpy as np
import argparse
import sys
import os
import glob


def load_yaml(filename, exit_on_error=True):
    """Load YAML file"""
    try:
        with open(filename, 'r') as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        if exit_on_error:
            print(f"Error: File '{filename}' not found")
            sys.exit(1)
        return None
    except yaml.YAMLError as e:
        if exit_on_error:
            print(f"Error parsing YAML: {e}")
            sys.exit(1)
        return None


def draw_obstacles(ax, env):
    """Draw obstacles from environment file"""
    obstacles = env.get('environment', {}).get('obstacles', [])
    for obs in obstacles:
        obs_type = obs.get('type', '')
        if obs_type == 'box':
            size = obs.get('size', [0.0, 0.0])
            center = obs.get('center', [0.0, 0.0])
            width = float(size[0])
            height = float(size[1])
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


# ============================================================================
# Hierarchical Decomposition Visualization
# ============================================================================

def get_cell_center(region_id, grid_length, bounds_min, bounds_max):
    """Get the (x, y) center of a base grid cell given its region ID."""
    col = region_id % grid_length
    row = region_id // grid_length
    cell_w = (bounds_max[0] - bounds_min[0]) / grid_length
    cell_h = (bounds_max[1] - bounds_min[1]) / grid_length
    return (bounds_min[0] + (col + 0.5) * cell_w,
            bounds_min[1] + (row + 0.5) * cell_h)


def draw_leads(ax, leads, grid_length, bounds_min, bounds_max):
    """
    Draw high-level paths (leads) across the decomposition grid.

    Args:
        ax: Matplotlib axes
        leads: List of leads (one per robot), each a list of region IDs
        grid_length: Number of cells per dimension
        bounds_min: Environment min bounds
        bounds_max: Environment max bounds
    """
    if not leads:
        return

    cmap = plt.cm.tab10
    num_robots = len(leads)

    for robot_idx, lead in enumerate(leads):
        if not lead:
            continue

        color = cmap(robot_idx / max(num_robots, 1))

        # Compute cell centers along the path
        centers = [get_cell_center(rid, grid_length, bounds_min, bounds_max)
                    for rid in lead]

        # Small offset per robot so overlapping paths are visible
        offset = (robot_idx - num_robots / 2.0) * 0.3

        xs = [c[0] + offset for c in centers]
        ys = [c[1] + offset for c in centers]

        # Draw arrows between consecutive cells
        for i in range(len(xs) - 1):
            ax.annotate('', xy=(xs[i + 1], ys[i + 1]),
                        xytext=(xs[i], ys[i]),
                        arrowprops=dict(arrowstyle='->', color=color,
                                        lw=2.0, mutation_scale=15),
                        zorder=10)

        # Mark start and end
        ax.plot(xs[0], ys[0], 'o', color=color, markersize=10, zorder=11,
                label=f'Robot {robot_idx}')
        ax.plot(xs[-1], ys[-1], 's', color=color, markersize=10, zorder=11)
        ax.text(xs[0], ys[0], f' S{robot_idx}', color=color, fontsize=9,
                fontweight='bold', zorder=12, va='bottom')
        ax.text(xs[-1], ys[-1], f' G{robot_idx}', color=color, fontsize=9,
                fontweight='bold', zorder=12, va='bottom')


def get_color_for_depth(depth, max_depth=5):
    """Get a color based on refinement depth"""
    # Use a colormap that goes from blue (unrefined) to red (most refined)
    cmap = plt.cm.coolwarm
    normalized = min(depth / max_depth, 1.0)
    return cmap(normalized)


def plot_hierarchy_cell(ax, cell, bounds_min, bounds_max, grid_length, depth=0,
                        show_ids=True, show_bounds=True, max_depth=5):
    """
    Recursively plot a cell from the decomposition hierarchy.

    Args:
        ax: Matplotlib axes
        cell: Cell data - either an int (leaf) or [parent_id, [children...]]
        bounds_min: Environment min bounds
        bounds_max: Environment max bounds
        grid_length: Number of cells per dimension in the base decomposition
        depth: Current refinement depth (for coloring)
        show_ids: Whether to show region IDs
        show_bounds: Whether to draw cell boundaries
        max_depth: Maximum depth for color scaling

    Returns:
        Number of cells drawn (for statistics)
    """
    cells_drawn = 0

    # Calculate base cell size
    base_cell_width = (bounds_max[0] - bounds_min[0]) / grid_length
    base_cell_height = (bounds_max[1] - bounds_min[1]) / grid_length

    if isinstance(cell, int):
        # Leaf cell - not refined
        region_id = cell

        # Calculate cell position from region ID
        # Assuming row-major ordering: region_id = row * grid_length + col
        col = region_id % grid_length
        row = region_id // grid_length

        x = bounds_min[0] + col * base_cell_width
        y = bounds_min[1] + row * base_cell_height

        # Draw the cell
        color = get_color_for_depth(depth, max_depth)

        if show_bounds:
            rect = patches.Rectangle(
                (x, y), base_cell_width, base_cell_height,
                facecolor=color,
                edgecolor='black',
                linewidth=1.5,
                alpha=0.3,
                zorder=2
            )
            ax.add_patch(rect)

        # Show region ID
        if show_ids:
            center_x = x + base_cell_width / 2
            center_y = y + base_cell_height / 2
            ax.text(center_x, center_y, str(region_id),
                   ha='center', va='center',
                   fontsize=8, color='black', alpha=0.8,
                   fontweight='bold',
                   zorder=5)

        cells_drawn = 1

    elif isinstance(cell, list) and len(cell) >= 2:
        # Refined cell: [parent_id, [children...]]
        parent_id = cell[0]
        children = cell[1]

        # Calculate parent cell position
        col = parent_id % grid_length
        row = parent_id // grid_length

        parent_x = bounds_min[0] + col * base_cell_width
        parent_y = bounds_min[1] + row * base_cell_height

        # Draw parent cell with highlight fill and dashed outline
        if show_bounds and depth == 0:
            # Light yellow fill to highlight refined region
            rect_fill = patches.Rectangle(
                (parent_x, parent_y), base_cell_width, base_cell_height,
                facecolor='#FFFFCC',
                edgecolor='none',
                alpha=0.4,
                zorder=1
            )
            ax.add_patch(rect_fill)
            # Dashed outline
            rect = patches.Rectangle(
                (parent_x, parent_y), base_cell_width, base_cell_height,
                facecolor='none',
                edgecolor='darkorange',
                linewidth=2.0,
                linestyle='--',
                alpha=0.8,
                zorder=4
            )
            ax.add_patch(rect)
            # Label the parent ID above the cell
            if show_ids:
                ax.text(parent_x + base_cell_width / 2,
                        parent_y + base_cell_height + 0.15,
                        f'Cell {parent_id} (refined)',
                        ha='center', va='bottom',
                        fontsize=7, color='darkorange',
                        fontweight='bold', zorder=5)

        # Draw children
        if isinstance(children, list):
            # Calculate child grid size
            num_children = len(children)
            child_grid_length = int(np.sqrt(num_children))
            if child_grid_length * child_grid_length != num_children:
                # Not a perfect square, try to handle gracefully
                child_grid_length = int(np.ceil(np.sqrt(num_children)))

            child_width = base_cell_width / child_grid_length
            child_height = base_cell_height / child_grid_length

            for i, child in enumerate(children):
                child_col = i % child_grid_length
                child_row = i // child_grid_length

                child_x = parent_x + child_col * child_width
                child_y = parent_y + child_row * child_height

                if isinstance(child, int):
                    # Leaf child cell - use distinct colors per sub-cell
                    sub_cmap = plt.cm.Pastel1
                    color = sub_cmap(i / max(num_children, 1))

                    rect = patches.Rectangle(
                        (child_x, child_y), child_width, child_height,
                        facecolor=color,
                        edgecolor='darkorange',
                        linewidth=1.5,
                        alpha=0.6,
                        zorder=3 + depth
                    )
                    ax.add_patch(rect)

                    if show_ids:
                        center_x = child_x + child_width / 2
                        center_y = child_y + child_height / 2
                        label = f"{parent_id}.{i}"
                        ax.text(center_x, center_y, label,
                               ha='center', va='center',
                               fontsize=7, color='black', alpha=0.9,
                               fontweight='bold',
                               zorder=5 + depth)

                    cells_drawn += 1

                elif isinstance(child, list):
                    # Recursively refined child - this would require
                    # more complex bounds tracking
                    # For now, just draw it as a nested refinement
                    cells_drawn += plot_hierarchy_cell_nested(
                        ax, child,
                        child_x, child_y, child_width, child_height,
                        depth + 1, show_ids, show_bounds, max_depth
                    )

    return cells_drawn


def plot_hierarchy_cell_nested(ax, cell, x, y, width, height, depth,
                                show_ids, show_bounds, max_depth):
    """
    Plot a nested refined cell (for deeply nested hierarchies).

    Args:
        ax: Matplotlib axes
        cell: Cell data - [parent_id, [children...]]
        x, y: Position of this cell
        width, height: Size of this cell
        depth: Current refinement depth
        show_ids, show_bounds, max_depth: Display options

    Returns:
        Number of cells drawn
    """
    cells_drawn = 0

    if isinstance(cell, int):
        # Leaf cell
        color = get_color_for_depth(depth, max_depth)

        rect = patches.Rectangle(
            (x, y), width, height,
            facecolor=color,
            edgecolor='darkred',
            linewidth=0.8,
            alpha=0.5,
            zorder=3 + depth
        )
        ax.add_patch(rect)
        cells_drawn = 1

    elif isinstance(cell, list) and len(cell) >= 2:
        parent_id = cell[0]
        children = cell[1]

        if isinstance(children, list):
            num_children = len(children)
            child_grid_length = int(np.ceil(np.sqrt(num_children)))

            child_width = width / child_grid_length
            child_height = height / child_grid_length

            for i, child in enumerate(children):
                child_col = i % child_grid_length
                child_row = i // child_grid_length

                child_x = x + child_col * child_width
                child_y = y + child_row * child_height

                cells_drawn += plot_hierarchy_cell_nested(
                    ax, child,
                    child_x, child_y, child_width, child_height,
                    depth + 1, show_ids, show_bounds, max_depth
                )

    return cells_drawn


def plot_decomposition_hierarchy(output_file_path, env_file=None, save_path=None,
                                  show_ids=True):
    """
    Plot the hierarchical decomposition from an MR-SyCLoP output file.

    Args:
        output_file_path: Path to the MR-SyCLoP output YAML file
        env_file: Optional path to environment YAML file (for obstacles)
        save_path: Optional output file path (saves instead of showing)
        show_ids: Whether to show region IDs
    """
    data = load_yaml(output_file_path)

    if data is None:
        print(f"Error: Could not load output file {output_file_path}")
        return

    decomp = data.get('decomposition', {})
    hierarchy = decomp.get('hierarchy', [])
    grid_size = decomp.get('grid_size', [1, 1])
    bounds = decomp.get('bounds', {})

    if not hierarchy:
        print("No hierarchy data found in output file")
        print("Available keys in decomposition:", list(decomp.keys()))
        return

    bounds_min = bounds.get('min', [0, 0])
    bounds_max = bounds.get('max', [10, 10])
    grid_length = grid_size[0] if isinstance(grid_size, list) else grid_size

    print(f"Decomposition: {grid_length}x{grid_length} grid")
    print(f"Bounds: [{bounds_min[0]}, {bounds_min[1]}] to [{bounds_max[0]}, {bounds_max[1]}]")
    print(f"Hierarchy has {len(hierarchy)} top-level cells")

    # Count refined cells
    num_refined = sum(1 for cell in hierarchy if isinstance(cell, list))
    print(f"Refined cells: {num_refined}/{len(hierarchy)}")

    fig, ax = plt.subplots(figsize=(12, 12))

    # Load and draw obstacles if env file provided
    if env_file:
        env = load_yaml(env_file, exit_on_error=False)
        if env:
            draw_obstacles(ax, env)

    # Plot each cell in the hierarchy
    total_cells = 0
    for cell in hierarchy:
        cells = plot_hierarchy_cell(
            ax, cell,
            bounds_min, bounds_max, grid_length,
            depth=0,
            show_ids=show_ids,
            show_bounds=True,
            max_depth=5
        )
        total_cells += cells

    print(f"Total cells drawn: {total_cells}")

    # Draw high-level paths (leads) if present
    leads = decomp.get('leads', [])
    if leads:
        print(f"Drawing leads for {len(leads)} robots")
        draw_leads(ax, leads, grid_length, bounds_min, bounds_max)

    # Set axis limits
    margin = 0.5
    ax.set_xlim(bounds_min[0] - margin, bounds_max[0] + margin)
    ax.set_ylim(bounds_min[1] - margin, bounds_max[1] + margin)
    ax.set_aspect('equal')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

    # Title
    leads_info = f', {len(leads)} robot paths' if leads else ''
    ax.set_title(f'Hierarchical Decomposition\n'
                 f'Base Grid: {grid_length}x{grid_length}, '
                 f'Refined: {num_refined}/{len(hierarchy)} cells, '
                 f'Total: {total_cells} cells{leads_info}')

    ax.grid(True, alpha=0.3)

    # Add legend for robot paths
    if leads:
        ax.legend(loc='upper right', fontsize=8)

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved to {save_path}")
    else:
        plt.show()


def print_hierarchy_text(output_file_path):
    """
    Print the decomposition hierarchy as a text tree.

    Args:
        output_file_path: Path to the MR-SyCLoP output YAML file
    """
    data = load_yaml(output_file_path)

    if data is None:
        print(f"Error: Could not load output file {output_file_path}")
        return

    decomp = data.get('decomposition', {})
    hierarchy = decomp.get('hierarchy', [])

    if not hierarchy:
        print("No hierarchy data found")
        return

    print("Decomposition Hierarchy:")
    print("=" * 50)

    def print_cell(cell, indent=0):
        prefix = "  " * indent
        if isinstance(cell, int):
            print(f"{prefix}[{cell}] (leaf)")
        elif isinstance(cell, list) and len(cell) >= 2:
            parent_id = cell[0]
            children = cell[1]
            print(f"{prefix}[{parent_id}] (refined into {len(children)} cells)")
            if isinstance(children, list):
                for i, child in enumerate(children):
                    print(f"{prefix}  Child {i}:")
                    print_cell(child, indent + 2)

    for i, cell in enumerate(hierarchy):
        print(f"\nRegion {i}:")
        print_cell(cell, indent=1)

    print("\n" + "=" * 50)


def plot_decomposition(ax, decomp_data, color='blue', alpha=0.3,
                       show_regions=True, show_neighbors=False,
                       show_ids=True, linewidth=1.5):
    """
    Plot a single decomposition on the given axes.

    Args:
        ax: Matplotlib axes
        decomp_data: Decomposition data dict from YAML
        color: Color for region boundaries
        alpha: Alpha for region fill
        show_regions: Whether to fill regions
        show_neighbors: Whether to draw neighbor connections
        show_ids: Whether to show region IDs
        linewidth: Line width for region boundaries
    """
    if decomp_data is None:
        return

    regions = decomp_data.get('regions', [])
    bounds = decomp_data.get('bounds', {})
    grid_length = decomp_data.get('grid_length', 1)
    num_regions = decomp_data.get('num_regions', len(regions))

    # Get overall bounds
    bounds_min = bounds.get('min', [0, 0])
    bounds_max = bounds.get('max', [10, 10])

    # Draw each region
    for region in regions:
        region_id = region.get('id', 0)
        region_min = region.get('min', [0, 0])
        region_max = region.get('max', [1, 1])
        neighbors = region.get('neighbors', [])

        x = region_min[0]
        y = region_min[1]
        width = region_max[0] - region_min[0]
        height = region_max[1] - region_min[1]

        # Fill region with light color
        if show_regions:
            rect = patches.Rectangle(
                (x, y), width, height,
                facecolor=color,
                edgecolor='none',
                alpha=alpha * 0.3,
                zorder=0
            )
            ax.add_patch(rect)

        # Draw region boundary
        rect = patches.Rectangle(
            (x, y), width, height,
            facecolor='none',
            edgecolor=color,
            linewidth=linewidth,
            alpha=alpha,
            zorder=2
        )
        ax.add_patch(rect)

        # Show region ID at center
        if show_ids:
            center_x = x + width / 2
            center_y = y + height / 2
            ax.text(center_x, center_y, str(region_id),
                   ha='center', va='center',
                   fontsize=8, color='black', alpha=0.7,
                   zorder=3)

        # Draw neighbor connections
        if show_neighbors:
            center_x = x + width / 2
            center_y = y + height / 2

            for neighbor_id in neighbors:
                # Find neighbor region
                for other_region in regions:
                    if other_region.get('id') == neighbor_id:
                        other_min = other_region.get('min', [0, 0])
                        other_max = other_region.get('max', [1, 1])
                        other_cx = (other_min[0] + other_max[0]) / 2
                        other_cy = (other_min[1] + other_max[1]) / 2

                        # Draw line to neighbor (only draw once per pair)
                        if neighbor_id > region_id:
                            ax.plot([center_x, other_cx], [center_y, other_cy],
                                   '-', color=color, alpha=alpha * 0.5,
                                   linewidth=0.5, zorder=1)
                        break

    return bounds_min, bounds_max


def plot_single_decomposition(decomp_file, env_file=None, output_file=None,
                              show_neighbors=False, show_ids=True):
    """
    Plot a single decomposition file.

    Args:
        decomp_file: Path to decomposition YAML file
        env_file: Optional path to environment YAML file (for obstacles)
        output_file: Optional output file path (saves instead of showing)
        show_neighbors: Whether to show neighbor connections
        show_ids: Whether to show region IDs
    """
    decomp_data = load_yaml(decomp_file)

    if decomp_data is None:
        print(f"Error: Could not load decomposition from {decomp_file}")
        return

    fig, ax = plt.subplots(figsize=(10, 10))

    # Load and draw obstacles if env file provided
    if env_file:
        env = load_yaml(env_file, exit_on_error=False)
        if env:
            draw_obstacles(ax, env)

    # Plot decomposition
    bounds_min, bounds_max = plot_decomposition(
        ax, decomp_data,
        color='blue',
        alpha=0.6,
        show_neighbors=show_neighbors,
        show_ids=show_ids
    )

    # Set axis limits
    margin = 0.5
    ax.set_xlim(bounds_min[0] - margin, bounds_max[0] + margin)
    ax.set_ylim(bounds_min[1] - margin, bounds_max[1] + margin)
    ax.set_aspect('equal')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

    # Title with decomposition info
    grid_length = decomp_data.get('grid_length', '?')
    num_regions = decomp_data.get('num_regions', '?')
    ax.set_title(f'Decomposition: {os.path.basename(decomp_file)}\n'
                 f'Grid: {grid_length}x{grid_length}, Regions: {num_regions}')

    ax.grid(True, alpha=0.3)

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved to {output_file}")
    else:
        plt.show()


def plot_decomposition_sequence(decomp_dir, env_file=None, output_file=None,
                                show_neighbors=False, show_ids=True):
    """
    Plot a sequence of decompositions with a slider to navigate through them.

    Args:
        decomp_dir: Directory containing decomposition YAML files
        env_file: Optional path to environment YAML file (for obstacles)
        output_file: Optional output file path (saves GIF instead of showing)
        show_neighbors: Whether to show neighbor connections
        show_ids: Whether to show region IDs
    """
    # Find all decomposition files
    pattern = os.path.join(decomp_dir, 'decomposition_*.yaml')
    files = sorted(glob.glob(pattern))

    if not files:
        print(f"No decomposition files found in {decomp_dir}")
        return

    print(f"Found {len(files)} decomposition files")

    # Load all decompositions
    decompositions = []
    for f in files:
        data = load_yaml(f, exit_on_error=False)
        if data:
            decompositions.append({
                'file': f,
                'data': data,
                'name': os.path.basename(f)
            })

    if not decompositions:
        print("No valid decomposition files loaded")
        return

    # Load environment if provided
    env = None
    if env_file:
        env = load_yaml(env_file, exit_on_error=False)

    # Compute global bounds from all decompositions
    global_min = [float('inf'), float('inf')]
    global_max = [float('-inf'), float('-inf')]

    for d in decompositions:
        bounds = d['data'].get('bounds', {})
        b_min = bounds.get('min', [0, 0])
        b_max = bounds.get('max', [10, 10])
        global_min[0] = min(global_min[0], b_min[0])
        global_min[1] = min(global_min[1], b_min[1])
        global_max[0] = max(global_max[0], b_max[0])
        global_max[1] = max(global_max[1], b_max[1])

    # Create figure with slider
    fig = plt.figure(figsize=(12, 10))
    ax = plt.axes([0.1, 0.15, 0.8, 0.75])

    # Current state
    state = {'current_idx': 0}

    def draw_current():
        """Draw the current decomposition"""
        ax.clear()

        idx = state['current_idx']
        decomp = decompositions[idx]

        # Draw obstacles
        if env:
            draw_obstacles(ax, env)

        # Plot decomposition
        plot_decomposition(
            ax, decomp['data'],
            color='blue',
            alpha=0.6,
            show_neighbors=show_neighbors,
            show_ids=show_ids
        )

        # Set axis limits
        margin = 0.5
        ax.set_xlim(global_min[0] - margin, global_max[0] + margin)
        ax.set_ylim(global_min[1] - margin, global_max[1] + margin)
        ax.set_aspect('equal')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')

        # Title
        grid_length = decomp['data'].get('grid_length', '?')
        num_regions = decomp['data'].get('num_regions', '?')
        ax.set_title(f'Decomposition {idx + 1}/{len(decompositions)}: {decomp["name"]}\n'
                     f'Grid: {grid_length}x{grid_length}, Regions: {num_regions}')

        ax.grid(True, alpha=0.3)
        fig.canvas.draw_idle()

    # Create slider
    ax_slider = plt.axes([0.15, 0.05, 0.7, 0.03])
    slider = Slider(
        ax=ax_slider,
        label='Decomposition',
        valmin=0,
        valmax=len(decompositions) - 1,
        valinit=0,
        valstep=1
    )

    def on_slider_change(val):
        state['current_idx'] = int(val)
        draw_current()

    slider.on_changed(on_slider_change)

    # Initial draw
    draw_current()

    plt.show()


def plot_decomposition_overlay(decomp_files, env_file=None, output_file=None,
                               show_neighbors=False, show_ids=False):
    """
    Overlay multiple decompositions on the same plot.

    Args:
        decomp_files: List of decomposition YAML files
        env_file: Optional path to environment YAML file (for obstacles)
        output_file: Optional output file path
        show_neighbors: Whether to show neighbor connections
        show_ids: Whether to show region IDs
    """
    # Color cycle for different decompositions
    colors = plt.cm.tab10(np.linspace(0, 1, len(decomp_files)))

    fig, ax = plt.subplots(figsize=(12, 10))

    # Load environment if provided
    if env_file:
        env = load_yaml(env_file, exit_on_error=False)
        if env:
            draw_obstacles(ax, env)

    # Track global bounds
    global_min = [float('inf'), float('inf')]
    global_max = [float('-inf'), float('-inf')]

    legend_handles = []

    for i, decomp_file in enumerate(decomp_files):
        data = load_yaml(decomp_file, exit_on_error=False)
        if data is None:
            print(f"Warning: Could not load {decomp_file}")
            continue

        color = colors[i]

        bounds_min, bounds_max = plot_decomposition(
            ax, data,
            color=color,
            alpha=0.5,
            show_neighbors=show_neighbors,
            show_ids=show_ids,
            linewidth=1.0 + i * 0.5
        )

        # Update global bounds
        global_min[0] = min(global_min[0], bounds_min[0])
        global_min[1] = min(global_min[1], bounds_min[1])
        global_max[0] = max(global_max[0], bounds_max[0])
        global_max[1] = max(global_max[1], bounds_max[1])

        # Add to legend
        grid_length = data.get('grid_length', '?')
        label = f'{os.path.basename(decomp_file)} ({grid_length}x{grid_length})'
        legend_handles.append(patches.Patch(
            facecolor=color, edgecolor=color,
            alpha=0.5, label=label
        ))

    # Set axis limits
    margin = 0.5
    ax.set_xlim(global_min[0] - margin, global_max[0] + margin)
    ax.set_ylim(global_min[1] - margin, global_max[1] + margin)
    ax.set_aspect('equal')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title(f'Decomposition Overlay ({len(decomp_files)} decompositions)')
    ax.legend(handles=legend_handles, loc='upper right')
    ax.grid(True, alpha=0.3)

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved to {output_file}")
    else:
        plt.show()


def save_decomposition_sequence_as_gif(decomp_dir, output_file, env_file=None,
                                       show_neighbors=False, show_ids=True,
                                       fps=2, dpi=100):
    """
    Save a sequence of decompositions as an animated GIF.

    Args:
        decomp_dir: Directory containing decomposition YAML files
        output_file: Output GIF file path
        env_file: Optional path to environment YAML file (for obstacles)
        show_neighbors: Whether to show neighbor connections
        show_ids: Whether to show region IDs
        fps: Frames per second (decompositions per second)
        dpi: DPI for output image
    """
    import matplotlib.animation as animation

    # Find all decomposition files
    pattern = os.path.join(decomp_dir, 'decomposition_*.yaml')
    files = sorted(glob.glob(pattern))

    if not files:
        print(f"No decomposition files found in {decomp_dir}")
        return

    print(f"Found {len(files)} decomposition files")

    # Load all decompositions
    decompositions = []
    for f in files:
        data = load_yaml(f, exit_on_error=False)
        if data:
            decompositions.append({
                'file': f,
                'data': data,
                'name': os.path.basename(f)
            })

    if not decompositions:
        print("No valid decomposition files loaded")
        return

    # Load environment if provided
    env = None
    if env_file:
        env = load_yaml(env_file, exit_on_error=False)

    # Compute global bounds from all decompositions
    global_min = [float('inf'), float('inf')]
    global_max = [float('-inf'), float('-inf')]

    for d in decompositions:
        bounds = d['data'].get('bounds', {})
        b_min = bounds.get('min', [0, 0])
        b_max = bounds.get('max', [10, 10])
        global_min[0] = min(global_min[0], b_min[0])
        global_min[1] = min(global_min[1], b_min[1])
        global_max[0] = max(global_max[0], b_max[0])
        global_max[1] = max(global_max[1], b_max[1])

    # Create figure
    fig, ax = plt.subplots(figsize=(10, 10))

    def draw_frame(idx):
        """Draw a single frame"""
        ax.clear()

        decomp = decompositions[idx]

        # Draw obstacles
        if env:
            draw_obstacles(ax, env)

        # Plot decomposition
        plot_decomposition(
            ax, decomp['data'],
            color='blue',
            alpha=0.6,
            show_neighbors=show_neighbors,
            show_ids=show_ids
        )

        # Set axis limits
        margin = 0.5
        ax.set_xlim(global_min[0] - margin, global_max[0] + margin)
        ax.set_ylim(global_min[1] - margin, global_max[1] + margin)
        ax.set_aspect('equal')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')

        # Title
        grid_length = decomp['data'].get('grid_length', '?')
        num_regions = decomp['data'].get('num_regions', '?')
        ax.set_title(f'Decomposition {idx + 1}/{len(decompositions)}: {decomp["name"]}\n'
                     f'Grid: {grid_length}x{grid_length}, Regions: {num_regions}')

        ax.grid(True, alpha=0.3)

    def animate(frame):
        """Animation function"""
        draw_frame(frame)
        return []

    print(f"Creating animation with {len(decompositions)} frames at {fps} FPS...")

    anim = animation.FuncAnimation(
        fig, animate,
        frames=len(decompositions),
        interval=1000/fps,
        blit=False
    )

    print(f"Saving animation to {output_file}...")
    anim.save(output_file, writer='pillow', fps=fps, dpi=dpi)
    print(f"Animation saved successfully!")

    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description='Visualize decomposition files from MR-SyCLoP',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Plot a single decomposition
  %(prog)s --decomp output/decompositions/decomposition_0_initial.yaml

  # Plot decomposition with environment obstacles
  %(prog)s --decomp decomposition.yaml --env environment.yaml

  # Browse through all decompositions in a directory
  %(prog)s --dir output/decompositions/

  # Overlay multiple decompositions
  %(prog)s --overlay decomp_0.yaml decomp_1.yaml decomp_2.yaml

  # Save output to file
  %(prog)s --decomp decomposition.yaml --output plot.png

  # Show neighbor connections
  %(prog)s --decomp decomposition.yaml --show-neighbors

  # Hide region IDs
  %(prog)s --decomp decomposition.yaml --no-ids

  # Save decomposition sequence as GIF
  %(prog)s --dir output/decompositions/ --output decomp_sequence.gif --fps 2

  # Visualize hierarchical decomposition from MR-SyCLoP output
  %(prog)s --hierarchy output.yaml --env input.yaml

  # Print hierarchy as text tree
  %(prog)s --hierarchy output.yaml --text
        '''
    )

    parser.add_argument('--decomp', required=False, default=None,
                        help='Single decomposition YAML file to plot')
    parser.add_argument('--dir', required=False, default=None,
                        help='Directory containing decomposition files (enables slider view)')
    parser.add_argument('--overlay', nargs='+', required=False, default=None,
                        help='Multiple decomposition files to overlay')
    parser.add_argument('--hierarchy', required=False, default=None,
                        help='MR-SyCLoP output YAML file with hierarchical decomposition')
    parser.add_argument('--text', action='store_true', default=False,
                        help='Print hierarchy as text tree (use with --hierarchy)')
    parser.add_argument('--env', required=False, default=None,
                        help='Environment YAML file (for obstacles)')
    parser.add_argument('--output', '-o', required=False, default=None,
                        help='Output file path (saves instead of showing)')
    parser.add_argument('--show-neighbors', action='store_true', default=False,
                        help='Show neighbor connections between regions')
    parser.add_argument('--show-ids', action='store_true', default=True,
                        help='Show region IDs (default: True)')
    parser.add_argument('--no-ids', action='store_true', default=False,
                        help='Hide region IDs')
    parser.add_argument('--fps', type=int, default=2,
                        help='Frames per second for GIF output (default: 2)')
    parser.add_argument('--dpi', type=int, default=100,
                        help='DPI for output image (default: 100)')

    args = parser.parse_args()

    # Handle negation flags
    show_ids = args.show_ids and not args.no_ids

    # Determine mode
    if args.hierarchy:
        if args.text:
            print_hierarchy_text(args.hierarchy)
        else:
            plot_decomposition_hierarchy(
                args.hierarchy,
                env_file=args.env,
                save_path=args.output,
                show_ids=show_ids
            )
    elif args.overlay:
        plot_decomposition_overlay(
            args.overlay,
            env_file=args.env,
            output_file=args.output,
            show_neighbors=args.show_neighbors,
            show_ids=show_ids
        )
    elif args.dir:
        # Check if output is a GIF file
        if args.output and args.output.lower().endswith('.gif'):
            save_decomposition_sequence_as_gif(
                args.dir,
                output_file=args.output,
                env_file=args.env,
                show_neighbors=args.show_neighbors,
                show_ids=show_ids,
                fps=args.fps,
                dpi=args.dpi
            )
        else:
            plot_decomposition_sequence(
                args.dir,
                env_file=args.env,
                output_file=args.output,
                show_neighbors=args.show_neighbors,
                show_ids=show_ids
            )
    elif args.decomp:
        plot_single_decomposition(
            args.decomp,
            env_file=args.env,
            output_file=args.output,
            show_neighbors=args.show_neighbors,
            show_ids=show_ids
        )
    else:
        parser.print_help()
        print("\nError: Must specify --decomp, --dir, --overlay, or --hierarchy")
        sys.exit(1)


if __name__ == '__main__':
    main()
