#!/usr/bin/env python3
"""
Multi-Stage Planning Visualizer for Multi-Robot SyCLoP
Shows the complete planning pipeline:
  1. High-level decomposition and MAPF paths
  2. Low-level guided paths
  3. Path segmentation
  4. Collision detection
  5. Final solution

Requires: matplotlib, pyyaml
Install: pip install matplotlib pyyaml
"""

import yaml
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Button
import numpy as np
import argparse
import sys

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
    """Load YAML file"""
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
    """Get robot radius from config"""
    if 'radius' in robot_config:
        return robot_config['radius']
    robot_type = robot_config.get('type', '')
    if robot_type in DEFAULT_GEOMETRIES:
        max_radius = 0.0
        for spec in DEFAULT_GEOMETRIES[robot_type]:
            if spec[0] == 'sphere':
                radius = spec[1]
            else:
                radius = 0.5 * float(np.hypot(spec[1], spec[2]))
            if radius > max_radius:
                max_radius = radius
        if max_radius > 0.0:
            return max_radius
    return 0.5

def draw_obstacles(ax, env):
    """Draw obstacles on the axis"""
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

def draw_grid(ax, decomp, env_min, env_max):
    """Draw decomposition grid"""
    if decomp.get('type') != 'grid':
        return None

    grid_size = decomp['grid_size']
    bounds_min = decomp['bounds']['min']
    bounds_max = decomp['bounds']['max']

    cell_width = (bounds_max[0] - bounds_min[0]) / grid_size[0]
    cell_height = (bounds_max[1] - bounds_min[1]) / grid_size[1]

    # Draw grid lines
    for i in range(grid_size[0] + 1):
        x = bounds_min[0] + i * cell_width
        ax.axvline(x, color='darkgray', linewidth=1.5, alpha=0.6, linestyle='-', zorder=0)

    for j in range(grid_size[1] + 1):
        y = bounds_min[1] + j * cell_height
        ax.axhline(y, color='darkgray', linewidth=1.5, alpha=0.6, linestyle='-', zorder=0)

    return {
        'grid_size': grid_size,
        'bounds_min': bounds_min,
        'bounds_max': bounds_max,
        'cell_width': cell_width,
        'cell_height': cell_height
    }

def get_region_center(region_id, grid_info):
    """Get center of a region"""
    grid_size = grid_info['grid_size']
    bounds_min = grid_info['bounds_min']
    cell_width = grid_info['cell_width']
    cell_height = grid_info['cell_height']

    row = region_id // grid_size[0]
    col = region_id % grid_size[0]
    x = bounds_min[0] + (col + 0.5) * cell_width
    y = bounds_min[1] + (row + 0.5) * cell_height
    return x, y

def get_region_bounds(region_id, grid_info):
    """Get bounds of a region"""
    grid_size = grid_info['grid_size']
    bounds_min = grid_info['bounds_min']
    cell_width = grid_info['cell_width']
    cell_height = grid_info['cell_height']

    row = region_id // grid_size[0]
    col = region_id % grid_size[0]
    x = bounds_min[0] + col * cell_width
    y = bounds_min[1] + row * cell_height
    return x, y, cell_width, cell_height

class PlanningStagesVisualizer:
    """Interactive visualizer showing all planning stages"""

    def __init__(self, env_file, solution_file):
        self.env = load_yaml(env_file)
        self.solution = load_yaml(solution_file)

        if not self.solution or 'result' not in self.solution:
            print("Warning: No solution found in file")
            return

        self.env_min = self.env['environment']['min']
        self.env_max = self.env['environment']['max']
        self.colors = plt.cm.tab10(np.linspace(0, 1, len(self.env['robots'])))

        # Current stage
        self.current_stage = 0
        self.stages = [
            'Decomposition & High-Level Paths',
            'Low-Level Guided Paths',
            'Path Segmentation',
            'Collision Detection',
            'Final Solution'
        ]

        # Create figure with multiple subplots
        self.fig = plt.figure(figsize=(16, 10))
        self.setup_ui()
        self.draw_current_stage()

    def setup_ui(self):
        """Setup UI elements"""
        # Main plot
        self.ax = plt.axes([0.05, 0.15, 0.9, 0.75])

        # Navigation buttons
        ax_prev = plt.axes([0.3, 0.05, 0.1, 0.05])
        ax_next = plt.axes([0.6, 0.05, 0.1, 0.05])

        self.btn_prev = Button(ax_prev, 'Previous')
        self.btn_next = Button(ax_next, 'Next')

        self.btn_prev.on_clicked(self.prev_stage)
        self.btn_next.on_clicked(self.next_stage)

        # Stage indicator text
        self.stage_text = self.fig.text(0.5, 0.02, '', ha='center', fontsize=14, weight='bold')

    def prev_stage(self, event):
        """Go to previous stage"""
        if self.current_stage > 0:
            self.current_stage -= 1
            self.draw_current_stage()

    def next_stage(self, event):
        """Go to next stage"""
        if self.current_stage < len(self.stages) - 1:
            self.current_stage += 1
            self.draw_current_stage()

    def draw_current_stage(self):
        """Draw the current planning stage"""
        self.ax.clear()
        self.ax.set_xlim(self.env_min[0], self.env_max[0])
        self.ax.set_ylim(self.env_min[1], self.env_max[1])
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')

        # Update stage text
        self.stage_text.set_text(f'Stage {self.current_stage + 1}/{len(self.stages)}: {self.stages[self.current_stage]}')

        # Draw obstacles (always visible)
        draw_obstacles(self.ax, self.env)

        # Draw start/goal positions
        for robot_idx, robot_config in enumerate(self.env['robots']):
            color = self.colors[robot_idx]
            start = robot_config['start']
            goal = robot_config['goal']
            self.ax.plot(start[0], start[1], 'o', color=color, markersize=10,
                        markeredgecolor='black', markeredgewidth=2, alpha=0.3, zorder=2)
            self.ax.plot(goal[0], goal[1], '*', color=color, markersize=13,
                        markeredgecolor='black', markeredgewidth=1.5, alpha=0.3, zorder=2)

        # Draw stage-specific content
        if self.current_stage == 0:
            self.draw_decomposition_stage()
        elif self.current_stage == 1:
            self.draw_guided_paths_stage()
        elif self.current_stage == 2:
            self.draw_segmentation_stage()
        elif self.current_stage == 3:
            self.draw_collision_stage()
        elif self.current_stage == 4:
            self.draw_final_solution_stage()

        self.fig.canvas.draw_idle()

    def draw_decomposition_stage(self):
        """Stage 1: Decomposition and high-level MAPF paths"""
        self.ax.set_title('High-Level Decomposition & MAPF Planning', fontsize=14, weight='bold')

        if 'decomposition' not in self.solution:
            self.ax.text(0.5, 0.5, 'No decomposition data available',
                        transform=self.ax.transAxes, ha='center', va='center', fontsize=14)
            return

        decomp = self.solution['decomposition']
        grid_info = draw_grid(self.ax, decomp, self.env_min, self.env_max)

        if grid_info and 'leads' in decomp and decomp['leads']:
            # Highlight regions in high-level paths
            for robot_idx, lead in enumerate(decomp['leads']):
                if lead:
                    color = self.colors[robot_idx]
                    for region_id in lead:
                        x, y, w, h = get_region_bounds(region_id, grid_info)
                        rect = patches.Rectangle((x, y), w, h,
                                                linewidth=0,
                                                facecolor=color,
                                                alpha=0.15,
                                                zorder=0)
                        self.ax.add_patch(rect)

            # Draw arrows showing high-level path
            for robot_idx, lead in enumerate(decomp['leads']):
                if len(lead) > 1:
                    color = self.colors[robot_idx]
                    robot_name = self.env['robots'][robot_idx].get('name', f'Robot {robot_idx}')

                    for i in range(len(lead) - 1):
                        x1, y1 = get_region_center(lead[i], grid_info)
                        x2, y2 = get_region_center(lead[i + 1], grid_info)

                        # Draw arrow
                        self.ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                                       arrowprops=dict(arrowstyle='->', color=color, lw=2.5, alpha=0.7),
                                       zorder=3)

                    # Add label for first segment
                    if len(lead) > 1:
                        x1, y1 = get_region_center(lead[0], grid_info)
                        self.ax.text(x1, y1, robot_name, ha='center', va='center',
                                   fontsize=9, weight='bold', color=color,
                                   bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7),
                                   zorder=4)

        # Add legend with region path
        legend_text = "High-Level Paths (Region IDs):\n"
        if 'leads' in decomp:
            for robot_idx, lead in enumerate(decomp['leads']):
                robot_name = self.env['robots'][robot_idx].get('name', f'Robot {robot_idx}')
                path_str = ' → '.join(str(r) for r in lead) if lead else 'No path'
                legend_text += f"{robot_name}: {path_str}\n"

        self.ax.text(0.02, 0.98, legend_text, transform=self.ax.transAxes,
                    verticalalignment='top', fontsize=10,
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                    family='monospace')

    def draw_guided_paths_stage(self):
        """Stage 2: Low-level guided paths"""
        self.ax.set_title('Low-Level Guided Paths (SyclopRRT)', fontsize=14, weight='bold')

        if 'guided_paths' not in self.solution:
            # Fall back to result if guided_paths not available
            if 'result' in self.solution:
                for robot_idx, path_data in enumerate(self.solution['result']):
                    if 'states' in path_data and path_data['states']:
                        states = path_data['states']
                        xs = [s[0] for s in states]
                        ys = [s[1] for s in states]
                        color = self.colors[robot_idx]
                        robot_name = self.env['robots'][robot_idx].get('name', f'Robot {robot_idx}')
                        self.ax.plot(xs, ys, '-', color=color, linewidth=2, alpha=0.7, label=robot_name, zorder=3)
                self.ax.legend(loc='upper right')
            return

        # Draw guided paths
        for robot_idx, guided_path in enumerate(self.solution['guided_paths']):
            if guided_path.get('success') and 'states' in guided_path:
                states = guided_path['states']
                xs = [s[0] for s in states]
                ys = [s[1] for s in states]
                color = self.colors[robot_idx]
                robot_name = self.env['robots'][robot_idx].get('name', f'Robot {robot_idx}')

                # Draw path
                self.ax.plot(xs, ys, '-', color=color, linewidth=2.5, alpha=0.8, label=robot_name, zorder=3)

                # Mark waypoints
                self.ax.plot(xs, ys, 'o', color=color, markersize=3, alpha=0.5, zorder=4)

        self.ax.legend(loc='upper right')

        # Add info box
        info_text = "Guided Paths Info:\n"
        for robot_idx, guided_path in enumerate(self.solution['guided_paths']):
            robot_name = self.env['robots'][robot_idx].get('name', f'Robot {robot_idx}')
            if guided_path.get('success'):
                num_states = len(guided_path.get('states', []))
                num_controls = len(guided_path.get('controls', []))
                time = guided_path.get('planning_time', 0)
                info_text += f"{robot_name}: {num_states} states, {num_controls} controls ({time:.2f}s)\n"
            else:
                info_text += f"{robot_name}: Planning failed\n"

        self.ax.text(0.02, 0.02, info_text, transform=self.ax.transAxes,
                    verticalalignment='bottom', fontsize=9,
                    bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8),
                    family='monospace')

    def draw_segmentation_stage(self):
        """Stage 3: Path segmentation"""
        self.ax.set_title('Path Segmentation (Temporal Discretization)', fontsize=14, weight='bold')

        # Draw paths first
        if 'guided_paths' in self.solution:
            for robot_idx, guided_path in enumerate(self.solution['guided_paths']):
                if guided_path.get('success') and 'states' in guided_path:
                    states = guided_path['states']
                    xs = [s[0] for s in states]
                    ys = [s[1] for s in states]
                    color = self.colors[robot_idx]
                    self.ax.plot(xs, ys, '-', color=color, linewidth=1.5, alpha=0.3, zorder=2)

        # Draw segments
        if 'segments' not in self.solution:
            return

        for robot_idx, robot_segments in enumerate(self.solution['segments']):
            color = self.colors[robot_idx]
            robot_name = self.env['robots'][robot_idx].get('name', f'Robot {robot_idx}')

            for seg_idx, segment in enumerate(robot_segments):
                if 'start_state' in segment and 'end_state' in segment:
                    start = segment['start_state']
                    end = segment['end_state']

                    # Draw segment as thick line
                    self.ax.plot([start[0], end[0]], [start[1], end[1]], '-',
                               color=color, linewidth=3, alpha=0.7, zorder=3)

                    # Mark segment boundaries
                    self.ax.plot(start[0], start[1], 'o', color=color, markersize=6,
                               markeredgecolor='black', markeredgewidth=1, alpha=0.8, zorder=4)

                    # Label segment index at midpoint
                    mid_x = (start[0] + end[0]) / 2
                    mid_y = (start[1] + end[1]) / 2
                    self.ax.text(mid_x, mid_y, str(seg_idx), fontsize=7, ha='center', va='center',
                               color='white', weight='bold',
                               bbox=dict(boxstyle='circle,pad=0.2', facecolor=color, alpha=0.9),
                               zorder=5)

        # Add segment info
        info_text = "Segment Counts:\n"
        for robot_idx, robot_segments in enumerate(self.solution['segments']):
            robot_name = self.env['robots'][robot_idx].get('name', f'Robot {robot_idx}')
            info_text += f"{robot_name}: {len(robot_segments)} segments\n"

        self.ax.text(0.02, 0.98, info_text, transform=self.ax.transAxes,
                    verticalalignment='top', fontsize=10,
                    bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8),
                    family='monospace')

    def draw_collision_stage(self):
        """Stage 4: Collision detection"""
        self.ax.set_title('Collision Detection', fontsize=14, weight='bold')

        # Draw paths
        if 'guided_paths' in self.solution:
            for robot_idx, guided_path in enumerate(self.solution['guided_paths']):
                if guided_path.get('success') and 'states' in guided_path:
                    states = guided_path['states']
                    xs = [s[0] for s in states]
                    ys = [s[1] for s in states]
                    color = self.colors[robot_idx]
                    robot_name = self.env['robots'][robot_idx].get('name', f'Robot {robot_idx}')
                    self.ax.plot(xs, ys, '-', color=color, linewidth=2, alpha=0.4, label=robot_name, zorder=2)

        # Draw segments
        if 'segments' in self.solution:
            for robot_idx, robot_segments in enumerate(self.solution['segments']):
                color = self.colors[robot_idx]
                for segment in robot_segments:
                    if 'start_state' in segment and 'end_state' in segment:
                        start = segment['start_state']
                        end = segment['end_state']
                        self.ax.plot([start[0], end[0]], [start[1], end[1]], 'o',
                                   color=color, markersize=4, alpha=0.3, zorder=3)

        # Draw collisions
        if 'collisions' not in self.solution or not self.solution['collisions']:
            self.ax.text(0.5, 0.5, 'No collisions detected!',
                        transform=self.ax.transAxes, ha='center', va='center',
                        fontsize=16, weight='bold', color='green',
                        bbox=dict(boxstyle='round,pad=1', facecolor='lightgreen', alpha=0.8))
        else:
            collisions = self.solution['collisions']

            # Find collision locations and mark them
            for coll_idx, collision in enumerate(collisions):
                robot_1 = collision['robot_1']
                robot_2 = collision['robot_2']
                seg_1 = collision['segment_1']
                seg_2 = collision['segment_2']
                timestep = collision['timestep']

                # Try to get approximate collision location from segment data
                if 'segments' in self.solution:
                    if (robot_1 < len(self.solution['segments']) and
                        seg_1 < len(self.solution['segments'][robot_1])):
                        segment_1 = self.solution['segments'][robot_1][seg_1]
                        if 'start_state' in segment_1:
                            pos = segment_1['start_state']

                            # Draw collision marker
                            self.ax.plot(pos[0], pos[1], 'X', color='red', markersize=15,
                                       markeredgecolor='darkred', markeredgewidth=2, zorder=10)

                            # Add collision label
                            label = f"C{coll_idx}\nR{robot_1}-R{robot_2}\nT{timestep}"
                            self.ax.text(pos[0], pos[1] + 0.3, label, ha='center', va='bottom',
                                       fontsize=8, weight='bold', color='darkred',
                                       bbox=dict(boxstyle='round,pad=0.4', facecolor='yellow',
                                               edgecolor='red', linewidth=2, alpha=0.9),
                                       zorder=11)

            # Add collision summary
            info_text = f"Collisions Detected: {len(collisions)}\n\n"
            for coll_idx, collision in enumerate(collisions[:5]):  # Show first 5
                r1 = collision['robot_1']
                r2 = collision['robot_2']
                t = collision['timestep']
                info_text += f"C{coll_idx}: R{r1}-R{r2} at t={t}\n"
            if len(collisions) > 5:
                info_text += f"... and {len(collisions) - 5} more"

            self.ax.text(0.02, 0.98, info_text, transform=self.ax.transAxes,
                        verticalalignment='top', fontsize=9,
                        bbox=dict(boxstyle='round', facecolor='lightyellow',
                                edgecolor='red', linewidth=2, alpha=0.9),
                        family='monospace')

        self.ax.legend(loc='upper right')

    def draw_final_solution_stage(self):
        """Stage 5: Final solution"""
        self.ax.set_title('Final Solution', fontsize=14, weight='bold')

        # Draw final paths
        for robot_idx, path_data in enumerate(self.solution['result']):
            if 'states' in path_data and path_data['states']:
                states = path_data['states']
                xs = [s[0] for s in states]
                ys = [s[1] for s in states]
                color = self.colors[robot_idx]
                robot_name = self.env['robots'][robot_idx].get('name', f'Robot {robot_idx}')
                robot_radius = get_robot_radius(self.env['robots'][robot_idx])

                # Draw path
                self.ax.plot(xs, ys, '-', color=color, linewidth=2.5, alpha=0.7, label=robot_name, zorder=3)

                # Draw robot at final position
                if len(states) > 0:
                    final_state = states[-1]
                    circle = plt.Circle((final_state[0], final_state[1]), robot_radius,
                                      color=color, alpha=0.6, edgecolor='black', linewidth=2, zorder=5)
                    self.ax.add_patch(circle)

        self.ax.legend(loc='upper right')

        # Add summary
        success = self.solution.get('success', False)
        planning_time = self.solution.get('planning_time', 0)

        info_text = f"Planning: {'SUCCESS' if success else 'FAILED'}\n"
        info_text += f"Time: {planning_time:.2f}s\n\n"

        if 'collisions' in self.solution:
            num_collisions = len(self.solution['collisions'])
            info_text += f"Collisions resolved: {num_collisions}\n"

        self.ax.text(0.02, 0.98, info_text, transform=self.ax.transAxes,
                    verticalalignment='top', fontsize=11,
                    bbox=dict(boxstyle='round', facecolor='lightgreen' if success else 'lightcoral',
                            alpha=0.8),
                    family='monospace', weight='bold')

    def show(self):
        """Show the visualization"""
        plt.show()

def main():
    parser = argparse.ArgumentParser(
        description='Multi-stage visualization for Multi-Robot SyCLoP planning pipeline'
    )
    parser.add_argument('--env', required=True, help='Environment YAML file')
    parser.add_argument('--solution', required=True, help='Solution YAML file (with debug data)')

    args = parser.parse_args()

    visualizer = PlanningStagesVisualizer(args.env, args.solution)
    visualizer.show()

if __name__ == '__main__':
    main()
