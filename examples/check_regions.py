#!/usr/bin/env python3
"""Check which regions the start/goal positions are in"""

# Grid: 4x4 in 10x10 workspace
grid_size = [4, 4]
bounds_min = [0, 0]
bounds_max = [10, 10]

cell_width = (bounds_max[0] - bounds_min[0]) / grid_size[0]  # 2.5
cell_height = (bounds_max[1] - bounds_min[1]) / grid_size[1]  # 2.5

def locate_region(x, y):
    """Locate which region a point is in"""
    col = int((x - bounds_min[0]) / cell_width)
    row = int((y - bounds_min[1]) / cell_height)
    
    # Clamp to valid range
    col = min(max(col, 0), grid_size[0] - 1)
    row = min(max(row, 0), grid_size[1] - 1)
    
    region_id = row * grid_size[0] + col
    return region_id, row, col

# Robot positions
robot0_start = (2.5, 5.0)
robot0_goal = (7.5, 5.0)
robot1_start = (7.5, 5.0)
robot1_goal = (2.5, 5.0)

print("Region layout (4x4 grid):")
print("Row 3 (Y: 7.5-10): 12  13  14  15")
print("Row 2 (Y: 5.0-7.5):  8   9  10  11")
print("Row 1 (Y: 2.5-5.0):  4   5   6   7")
print("Row 0 (Y: 0.0-2.5):  0   1   2   3")
print("         Col:       0   1   2   3")
print()

positions = [
    ("Robot 0 start", robot0_start),
    ("Robot 0 goal", robot0_goal),
    ("Robot 1 start", robot1_start),
    ("Robot 1 goal", robot1_goal),
]

for name, (x, y) in positions:
    region_id, row, col = locate_region(x, y)
    x_range = f"{bounds_min[0] + col * cell_width:.1f}-{bounds_min[0] + (col+1) * cell_width:.1f}"
    y_range = f"{bounds_min[1] + row * cell_height:.1f}-{bounds_min[1] + (row+1) * cell_height:.1f}"
    print(f"{name:20s} ({x:4.1f}, {y:4.1f}) → Region {region_id:2d} (row={row}, col={col}, X: {x_range}, Y: {y_range})")

print()
print("Actual leads from solution.yaml:")
print("  Robot 0: [6, 9, 14]")
print("  Robot 1: [14, 9, 6]")
