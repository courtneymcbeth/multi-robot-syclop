#!/usr/bin/env python3
"""
Skeleton generator for PPL/WoDaSH multi-robot planning.

Computes the medial axis (skeleton) of 2D environments using Voronoi diagrams
and outputs the graph format expected by PPL's WoDaSH planner.

Usage:
    python generate_skeleton.py --input scenarios/random5_20x20.yaml --output skeleton.graph
    python generate_skeleton.py --input scenarios/random5_20x20.yaml --output skeleton.graph --visualize
"""

import argparse
import math
import yaml
import numpy as np
from pathlib import Path
from collections import defaultdict

try:
    import pyvoronoi
    HAS_PYVORONOI = True
except ImportError:
    HAS_PYVORONOI = False
    print("Warning: pyvoronoi not installed, using fallback method")

from shapely.geometry import Polygon, LineString, Point, MultiPolygon, box
from shapely.ops import unary_union
import matplotlib.pyplot as plt


def load_environment(yaml_path):
    """
    Load environment from YAML file.

    Returns:
        dict: Environment with 'boundary' and 'obstacles'
    """
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    env = data['environment']

    return {
        'min': env['min'],
        'max': env['max'],
        'obstacles': env.get('obstacles', [])
    }


def create_boundary_polygon(env):
    """Create boundary polygon from environment bounds."""
    min_x, min_y = env['min']
    max_x, max_y = env['max']
    return box(min_x, min_y, max_x, max_y)


def create_obstacle_polygon(obstacle):
    """Create polygon from obstacle definition."""
    if obstacle['type'] != 'box':
        raise ValueError(f"Unsupported obstacle type: {obstacle['type']}")

    cx, cy = obstacle['center']
    w, h = obstacle['size']

    return box(cx - w/2, cy - h/2, cx + w/2, cy + h/2)


def get_polygon_segments(polygon):
    """Extract line segments from a polygon."""
    coords = list(polygon.exterior.coords)
    segments = []
    for i in range(len(coords) - 1):
        segments.append((coords[i], coords[i+1]))
    return segments


def compute_free_space(env):
    """
    Compute free space polygon (boundary minus obstacles).

    Returns:
        Polygon or MultiPolygon: The free space
    """
    boundary = create_boundary_polygon(env)

    if not env['obstacles']:
        return boundary

    obstacle_polygons = [create_obstacle_polygon(obs) for obs in env['obstacles']]
    obstacles_union = unary_union(obstacle_polygons)

    free_space = boundary.difference(obstacles_union)
    return free_space


def point_in_free_space(point, free_space, tolerance=1e-6):
    """Check if a point is inside the free space."""
    p = Point(point[0], point[1])
    return free_space.contains(p) or free_space.boundary.distance(p) < tolerance


def compute_voronoi_skeleton(env, clearance=0.0):
    """
    Compute skeleton using Voronoi diagram of line segments.

    Args:
        env: Environment dictionary
        clearance: Minimum clearance from obstacles (shrink free space)

    Returns:
        tuple: (vertices, edges) where edges are (src, tgt, waypoints)
    """
    if not HAS_PYVORONOI:
        return compute_skeleton_fallback(env, clearance)

    # Get all segments (boundary + obstacles)
    boundary = create_boundary_polygon(env)
    all_segments = get_polygon_segments(boundary)

    for obs in env['obstacles']:
        obs_poly = create_obstacle_polygon(obs)
        all_segments.extend(get_polygon_segments(obs_poly))

    # Compute free space for filtering
    free_space = compute_free_space(env)
    if clearance > 0:
        free_space = free_space.buffer(-clearance)

    # Scale factor for pyvoronoi (it uses integers internally)
    scale = 1000

    # Create Voronoi diagram
    pv = pyvoronoi.Pyvoronoi(scale)

    for seg in all_segments:
        pv.AddSegment([
            [seg[0][0], seg[0][1]],
            [seg[1][0], seg[1][1]]
        ])

    pv.Construct()

    # Extract edges and vertices
    edges = pv.GetEdges()
    vertices_raw = pv.GetVertices()
    cells = pv.GetCells()

    # Build vertex list (only those inside free space)
    vertex_coords = {}  # vertex_index -> (x, y)
    valid_vertices = set()

    for i, v in enumerate(vertices_raw):
        x, y = v.X, v.Y
        if point_in_free_space((x, y), free_space):
            vertex_coords[i] = (x, y)
            valid_vertices.add(i)

    # Build edge list
    skeleton_edges = []

    for edge in edges:
        if edge.is_primary:
            start_idx = edge.start
            end_idx = edge.end

            # Skip if either endpoint is at infinity (-1)
            if start_idx == -1 or end_idx == -1:
                continue

            # Skip if either endpoint is outside free space
            if start_idx not in valid_vertices or end_idx not in valid_vertices:
                continue

            start_pt = vertex_coords[start_idx]
            end_pt = vertex_coords[end_idx]

            # Check if edge is inside free space
            edge_line = LineString([start_pt, end_pt])
            if not free_space.contains(edge_line):
                # Check if at least the midpoint is inside
                mid = edge_line.interpolate(0.5, normalized=True)
                if not free_space.contains(mid):
                    continue

            # Handle curved edges (discretize) - is_linear=False means curved
            if not edge.is_linear:
                waypoints = discretize_curved_edge(edge, vertices_raw, pv, num_points=10)
                # Filter waypoints to those inside free space
                waypoints = [(x, y) for x, y in waypoints if point_in_free_space((x, y), free_space)]
                if len(waypoints) < 2:
                    waypoints = [start_pt, end_pt]
            else:
                waypoints = [start_pt, end_pt]

            skeleton_edges.append((start_idx, end_idx, waypoints))

    # Renumber vertices to be contiguous
    used_vertices = set()
    for src, tgt, _ in skeleton_edges:
        used_vertices.add(src)
        used_vertices.add(tgt)

    old_to_new = {old: new for new, old in enumerate(sorted(used_vertices))}

    final_vertices = [(vertex_coords[old][0], vertex_coords[old][1])
                      for old in sorted(used_vertices)]

    final_edges = [(old_to_new[src], old_to_new[tgt], waypoints)
                   for src, tgt, waypoints in skeleton_edges]

    return final_vertices, final_edges


def discretize_curved_edge(edge, vertices, pv, num_points=10):
    """Discretize a curved Voronoi edge into line segments."""
    start = vertices[edge.start]
    end = vertices[edge.end]

    # For parabolic arcs, we approximate with linear interpolation
    # A more accurate method would compute the actual parabola
    waypoints = []
    for i in range(num_points + 1):
        t = i / num_points
        x = start.X + t * (end.X - start.X)
        y = start.Y + t * (end.Y - start.Y)
        waypoints.append((x, y))

    return waypoints


def compute_skeleton_fallback(env, clearance=0.0):
    """
    Fallback skeleton computation using scipy Voronoi on discretized boundary.
    Less accurate but works without pyvoronoi.
    """
    from scipy.spatial import Voronoi

    # Get all segments
    boundary = create_boundary_polygon(env)
    all_segments = get_polygon_segments(boundary)

    for obs in env['obstacles']:
        obs_poly = create_obstacle_polygon(obs)
        all_segments.extend(get_polygon_segments(obs_poly))

    # Discretize segments into points
    points = []
    spacing = 0.5  # Point spacing along segments

    for seg in all_segments:
        p1, p2 = seg
        length = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
        num_points = max(2, int(length / spacing))

        for i in range(num_points):
            t = i / (num_points - 1) if num_points > 1 else 0
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            points.append([x, y])

    points = np.array(points)

    # Compute Voronoi
    vor = Voronoi(points)

    # Compute free space
    free_space = compute_free_space(env)
    if clearance > 0:
        free_space = free_space.buffer(-clearance)

    # Extract vertices inside free space
    vertex_coords = {}
    valid_vertices = set()

    for i, v in enumerate(vor.vertices):
        if point_in_free_space(v, free_space):
            vertex_coords[i] = (v[0], v[1])
            valid_vertices.add(i)

    # Extract edges
    skeleton_edges = []
    seen_edges = set()

    for ridge_vertices in vor.ridge_vertices:
        if -1 in ridge_vertices:
            continue

        v1, v2 = ridge_vertices
        if v1 not in valid_vertices or v2 not in valid_vertices:
            continue

        edge_key = (min(v1, v2), max(v1, v2))
        if edge_key in seen_edges:
            continue
        seen_edges.add(edge_key)

        p1 = vertex_coords[v1]
        p2 = vertex_coords[v2]

        # Check if edge is inside free space
        edge_line = LineString([p1, p2])
        if free_space.contains(edge_line):
            skeleton_edges.append((v1, v2, [p1, p2]))

    # Renumber vertices
    used_vertices = set()
    for src, tgt, _ in skeleton_edges:
        used_vertices.add(src)
        used_vertices.add(tgt)

    old_to_new = {old: new for new, old in enumerate(sorted(used_vertices))}

    final_vertices = [(vertex_coords[old][0], vertex_coords[old][1])
                      for old in sorted(used_vertices)]

    final_edges = [(old_to_new[src], old_to_new[tgt], waypoints)
                   for src, tgt, waypoints in skeleton_edges]

    return final_vertices, final_edges


def prune_skeleton(vertices, edges, min_branch_length=1.0):
    """
    Remove short dead-end branches from the skeleton.

    Args:
        vertices: List of (x, y) coordinates
        edges: List of (src, tgt, waypoints)
        min_branch_length: Minimum length of branches to keep

    Returns:
        tuple: Pruned (vertices, edges)
    """
    if not edges:
        return vertices, edges

    # Build adjacency list
    adj = defaultdict(list)
    for i, (src, tgt, waypoints) in enumerate(edges):
        length = sum(math.sqrt((waypoints[j+1][0]-waypoints[j][0])**2 +
                               (waypoints[j+1][1]-waypoints[j][1])**2)
                    for j in range(len(waypoints)-1))
        adj[src].append((tgt, i, length))
        adj[tgt].append((src, i, length))

    # Find dead ends (degree 1 vertices)
    edges_to_remove = set()
    changed = True

    while changed:
        changed = False
        for v in list(adj.keys()):
            neighbors = [(n, idx, l) for n, idx, l in adj[v] if idx not in edges_to_remove]
            if len(neighbors) == 1:
                neighbor, edge_idx, length = neighbors[0]
                if length < min_branch_length:
                    edges_to_remove.add(edge_idx)
                    changed = True

    # Filter edges
    remaining_edges = [(src, tgt, wp) for i, (src, tgt, wp) in enumerate(edges)
                       if i not in edges_to_remove]

    # Renumber vertices
    used_vertices = set()
    for src, tgt, _ in remaining_edges:
        used_vertices.add(src)
        used_vertices.add(tgt)

    if not used_vertices:
        return vertices, edges  # Return original if pruning removed everything

    old_to_new = {old: new for new, old in enumerate(sorted(used_vertices))}

    final_vertices = [vertices[old] for old in sorted(used_vertices)]
    final_edges = [(old_to_new[src], old_to_new[tgt], waypoints)
                   for src, tgt, waypoints in remaining_edges]

    return final_vertices, final_edges


def simplify_skeleton(vertices, edges, tolerance=0.5):
    """
    Simplify skeleton by merging close vertices and removing redundant waypoints.
    """
    if not vertices or not edges:
        return vertices, edges

    # Merge close vertices
    merged = {}  # old_idx -> new_idx
    new_vertices = []

    for i, (x, y) in enumerate(vertices):
        found = False
        for j, (nx, ny) in enumerate(new_vertices):
            if math.sqrt((x-nx)**2 + (y-ny)**2) < tolerance:
                merged[i] = j
                found = True
                break
        if not found:
            merged[i] = len(new_vertices)
            new_vertices.append((x, y))

    # Update edges
    new_edges = []
    seen = set()

    for src, tgt, waypoints in edges:
        new_src = merged[src]
        new_tgt = merged[tgt]

        if new_src == new_tgt:
            continue

        edge_key = (min(new_src, new_tgt), max(new_src, new_tgt))
        if edge_key in seen:
            continue
        seen.add(edge_key)

        # Simplify waypoints
        if len(waypoints) > 2:
            simplified = [waypoints[0]]
            for wp in waypoints[1:-1]:
                if math.sqrt((wp[0]-simplified[-1][0])**2 +
                            (wp[1]-simplified[-1][1])**2) > tolerance:
                    simplified.append(wp)
            simplified.append(waypoints[-1])
            waypoints = simplified

        new_edges.append((new_src, new_tgt, waypoints))

    return new_vertices, new_edges


def write_skeleton_graph(vertices, edges, output_path):
    """
    Write skeleton graph in PPL format.

    Format:
        num_vertices num_edges
        vertex_id x y z
        ...
        src_id tgt_id num_waypoints wp1_x wp1_y wp1_z ...
    """
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, 'w') as f:
        f.write(f"{len(vertices)} {len(edges)}\n")

        # Write vertices
        for i, (x, y) in enumerate(vertices):
            f.write(f"{i} {x} {y} 0\n")

        # Write edges with corrected waypoints
        for src, tgt, waypoints in edges:
            # Ensure waypoints start at src vertex and end at tgt vertex
            src_pos = vertices[src]
            tgt_pos = vertices[tgt]

            corrected_wp = list(waypoints)
            if corrected_wp:
                corrected_wp[0] = src_pos
                corrected_wp[-1] = tgt_pos
            else:
                corrected_wp = [src_pos, tgt_pos]

            wp_str = " ".join(f"{x} {y} 0" for x, y in corrected_wp)
            f.write(f"{src} {tgt} {len(corrected_wp)} {wp_str}\n")

    print(f"Written skeleton graph: {output_path}")
    print(f"  Vertices: {len(vertices)}")
    print(f"  Edges: {len(edges)}")


def visualize_skeleton(env, vertices, edges, output_path=None, show=True):
    """
    Visualize the skeleton overlaid on the environment.
    """
    fig, ax = plt.subplots(figsize=(12, 10))

    # Draw boundary
    min_x, min_y = env['min']
    max_x, max_y = env['max']
    boundary = plt.Rectangle((min_x, min_y), max_x - min_x, max_y - min_y,
                              fill=False, edgecolor='black', linewidth=2)
    ax.add_patch(boundary)

    # Draw obstacles
    for obs in env['obstacles']:
        if obs['type'] == 'box':
            cx, cy = obs['center']
            w, h = obs['size']
            rect = plt.Rectangle((cx - w/2, cy - h/2), w, h,
                                  fill=True, facecolor='gray', edgecolor='black')
            ax.add_patch(rect)

    # Draw skeleton edges
    for src, tgt, waypoints in edges:
        xs = [wp[0] for wp in waypoints]
        ys = [wp[1] for wp in waypoints]
        ax.plot(xs, ys, color='green', linewidth=2, alpha=0.7)

    # Draw skeleton vertices
    if vertices:
        vx = [v[0] for v in vertices]
        vy = [v[1] for v in vertices]
        ax.scatter(vx, vy, c='green', s=50, zorder=5)

    ax.set_xlim(min_x - 1, max_x + 1)
    ax.set_ylim(min_y - 1, max_y + 1)
    ax.set_aspect('equal')
    # ax.set_title('Environment Skeleton (Medial Axis)')
    ax.grid(False)
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.tick_params(length=0)

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Saved visualization: {output_path}")

    if show:
        plt.show()

    plt.close()


def generate_skeleton(yaml_path, output_path, clearance=0.0, prune_length=1.0,
                      simplify_tolerance=0.5, visualize=False, vis_output=None):
    """
    Main function to generate skeleton from environment YAML.

    Args:
        yaml_path: Path to environment YAML file
        output_path: Output path for skeleton graph
        clearance: Minimum clearance from obstacles
        prune_length: Minimum branch length (shorter branches are removed)
        simplify_tolerance: Tolerance for merging close vertices
        visualize: Whether to show visualization
        vis_output: Path to save visualization image
    """
    print(f"Loading environment: {yaml_path}")
    env = load_environment(yaml_path)

    print(f"Environment bounds: {env['min']} to {env['max']}")
    print(f"Number of obstacles: {len(env['obstacles'])}")

    print("Computing Voronoi skeleton...")
    vertices, edges = compute_voronoi_skeleton(env, clearance)
    print(f"  Raw: {len(vertices)} vertices, {len(edges)} edges")

    if prune_length > 0:
        print(f"Pruning branches shorter than {prune_length}...")
        vertices, edges = prune_skeleton(vertices, edges, prune_length)
        print(f"  After pruning: {len(vertices)} vertices, {len(edges)} edges")

    if simplify_tolerance > 0:
        print(f"Simplifying with tolerance {simplify_tolerance}...")
        vertices, edges = simplify_skeleton(vertices, edges, simplify_tolerance)
        print(f"  After simplifying: {len(vertices)} vertices, {len(edges)} edges")

    write_skeleton_graph(vertices, edges, output_path)

    if visualize or vis_output:
        visualize_skeleton(env, vertices, edges, vis_output, show=visualize)

    return vertices, edges


def main():
    parser = argparse.ArgumentParser(
        description='Generate skeleton graph for PPL/WoDaSH',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate skeleton from scenario file
  python generate_skeleton.py --input scenarios/random5_20x20.yaml --output skeleton.graph

  # With visualization
  python generate_skeleton.py --input scenarios/random5_20x20.yaml --output skeleton.graph --visualize

  # With custom parameters
  python generate_skeleton.py --input scenarios/random5_20x20.yaml --output skeleton.graph \\
    --clearance 0.5 --prune-length 2.0 --simplify 0.3
        """
    )

    parser.add_argument('--input', '-i', required=True,
                        help='Input YAML scenario file')
    parser.add_argument('--output', '-o', required=True,
                        help='Output skeleton graph file')
    parser.add_argument('--clearance', '-c', type=float, default=0.0,
                        help='Minimum clearance from obstacles (default: 0.0)')
    parser.add_argument('--prune-length', '-p', type=float, default=1.0,
                        help='Minimum branch length to keep (default: 1.0)')
    parser.add_argument('--simplify', '-s', type=float, default=0.5,
                        help='Simplification tolerance (default: 0.5)')
    parser.add_argument('--visualize', '-v', action='store_true',
                        help='Show visualization window')
    parser.add_argument('--vis-output', '-V',
                        help='Save visualization to file')

    args = parser.parse_args()

    generate_skeleton(
        yaml_path=args.input,
        output_path=args.output,
        clearance=args.clearance,
        prune_length=args.prune_length,
        simplify_tolerance=args.simplify,
        visualize=args.visualize,
        vis_output=args.vis_output
    )


if __name__ == '__main__':
    main()
