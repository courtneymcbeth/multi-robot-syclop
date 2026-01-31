"""
Generate .g geometry files (BRLCAD format) for PPL obstacles.

This module creates custom-sized box geometry files based on obstacle dimensions
from the multi-robot-syclop YAML scenario files.
"""

def generate_box_geometry(width, height, depth=0.1):
    """
    Generate a box geometry in BRLCAD .g format.

    Args:
        width: Box width (x dimension)
        height: Box height (y dimension)
        depth: Box depth (z dimension), default 0.1

    Returns:
        str: Content of .g file in BRLCAD format
    """
    # Box is centered at origin, so half-dimensions
    hw = width / 2.0
    hh = height / 2.0
    hd = depth / 2.0

    # 8 vertices of the box (in specific order for BRLCAD)
    # Bottom face (z = 0), then top face (z = depth)
    vertices = [
        ( hw,  hh, depth),    # 0: front-right-bottom
        ( hw,  hh, 0.0),  # 1: front-right-top
        ( hw, -hh, depth),    # 2: back-right-bottom
        ( hw, -hh, 0.0),  # 3: back-right-top
        (-hw,  hh, depth),    # 4: front-left-bottom
        (-hw,  hh, 0.0),  # 5: front-left-top
        (-hw, -hh, depth),    # 6: back-left-bottom
        (-hw, -hh, 0.0),  # 7: back-left-top
    ]

    # Build the .g file content
    # Header: appears to be: num_objects num_vertices num_edges num_faces
    # For a box: 1 object, 8 vertices, 12 edges, 36 total indices
    lines = []
    lines.append("      1    8      12       36")
    lines.append("      1     36")
    lines.append("")

    # Write vertices
    for v in vertices:
        lines.append(f"{v[0]} {v[1]} {v[2]}")

    lines.append("")

    # Face definitions (from the reference block.g)
    # These define the 12 edges/faces of the box
    # Format appears to be: count vertex1 vertex2 ... (negative numbers?)
    face_defs = [
        "1 5 -7",
        "1 7 -3",
        "1 2 -6",
        "1 6 -5",
        "1 3 -4",
        "1 4 -2",
        "5 6 -8",
        "5 8 -7",
        "2 4 -8",
        "2 8 -6",
        "3 7 -8",
        "3 8 -4",
    ]

    for face_def in face_defs:
        lines.append(face_def)

    lines.append("")

    return "\n".join(lines)


def create_geometry_file(width, height, output_path, depth=0.1):
    """
    Create a .g geometry file for a box of given dimensions.

    Args:
        width: Box width
        height: Box height
        output_path: Path where .g file should be written
        depth: Box depth (default 0.1)
    """
    content = generate_box_geometry(width, height, depth)

    with open(output_path, 'w') as f:
        f.write(content)

    print(f"Created geometry file: {output_path} (size: {width}x{height}x{depth})")


def get_geometry_filename(width, height):
    """
    Generate a standardized filename for a box geometry.

    Args:
        width: Box width
        height: Box height

    Returns:
        str: Filename in format "block_WxH.g"
    """
    # Round to 1 decimal place for filename
    w_str = f"{width:.1f}".rstrip('0').rstrip('.')
    h_str = f"{height:.1f}".rstrip('0').rstrip('.')

    return f"block_{w_str}x{h_str}.g"


if __name__ == "__main__":
    # Test generation
    import sys
    import os

    if len(sys.argv) < 3:
        print("Usage: python ppl_geometry_generator.py <width> <height> [output_dir]")
        print("Example: python ppl_geometry_generator.py 1.0 1.0 /tmp")
        sys.exit(1)

    width = float(sys.argv[1])
    height = float(sys.argv[2])
    output_dir = sys.argv[3] if len(sys.argv) > 3 else "."

    filename = get_geometry_filename(width, height)
    output_path = os.path.join(output_dir, filename)

    create_geometry_file(width, height, output_path)
    print(f"Generated: {output_path}")
