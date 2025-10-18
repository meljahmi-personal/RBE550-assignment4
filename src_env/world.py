"""
world.py
Environment generation for RBE550 HW4 – Autonomous Valet Parking.

Creates a 12×12 grid (3 m per cell) populated with randomized tetromino
obstacles.  Ensures adequate clearances for the largest vehicle (truck
with trailer), clears the start region in the northwest corner, and
defines a parking bay in the southeast corner.
"""

import math
import random
from vehicles.base import State2D
from geom.polygons import oriented_box



# ========================
# Global configuration
# ========================

GRID_CELL_SIZE_METERS = 3.0          # Each grid cell is 3×3 meters
GRID_SIZE_CELLS = 12                 # 12×12 grid

# Vehicle–dependent clearance design
TRUCK_WIDTH_METERS = 2.0
SAFETY_MARGIN_METERS = 0.5           # Margin on each side of vehicle
MIN_CORRIDOR_WIDTH_METERS = TRUCK_WIDTH_METERS + 2 * SAFETY_MARGIN_METERS  # 3.0 m
MAX_STEERING_ANGLE_RAD = 0.60        # ~34 degrees
TRUCK_WHEELBASE_METERS = 3.4
MIN_TURN_RADIUS_METERS = TRUCK_WHEELBASE_METERS / math.tan(MAX_STEERING_ANGLE_RAD)  # ≈5.0 m

# Tetromino definitions in grid-cell coordinates
TETROMINO_SHAPES = {
    "I": [(0, 0), (1, 0), (2, 0), (3, 0)],
    "O": [(0, 0), (1, 0), (0, 1), (1, 1)],
    "L": [(0, 0), (0, 1), (0, 2), (1, 2)],
    "J": [(1, 0), (1, 1), (1, 2), (0, 2)],
    "T": [(0, 0), (1, 0), (2, 0), (1, 1)],
}

ROTATION_FUNCTIONS = [
    lambda x, y: (x, y),
    lambda x, y: (-y, x),
    lambda x, y: (-x, -y),
    lambda x, y: (y, -x),
]


# ========================
# Helper functions
# ========================

def convert_cells_to_polygons(cell_indices):
    """Convert grid-cell coordinates to rectangular obstacle polygons."""
    obstacle_polygons = []
    for (col, row) in cell_indices:
        center_x = (col + 0.5) * GRID_CELL_SIZE_METERS
        center_y = (row + 0.5) * GRID_CELL_SIZE_METERS
        obstacle_polygons.append(
             oriented_box(
                 (center_x, center_y),
                 GRID_CELL_SIZE_METERS,        # length
                 GRID_CELL_SIZE_METERS,        # width
                 0.0                           # theta (axis-aligned)
             )
         )                               

    return obstacle_polygons


def get_parking_bay_cells():
    """Return a 3×3 block of cells in the southeast corner for the parking bay."""
    return {(GRID_SIZE_CELLS - 3 + i, 0 + j) for i in range(3) for j in range(3)}


def has_valid_clearance(occupied_cells):
    """Check that obstacle layout leaves sufficient boundary and corridor clearances."""
    boundary_buffer_cells = math.ceil(MIN_CORRIDOR_WIDTH_METERS / GRID_CELL_SIZE_METERS)

    # Reject obstacles that touch outer boundary
    for (col, row) in occupied_cells:
        if col < boundary_buffer_cells or col >= GRID_SIZE_CELLS - boundary_buffer_cells:
            return False
        if row < boundary_buffer_cells or row >= GRID_SIZE_CELLS - boundary_buffer_cells:
            return False

    # Simple choke-point rejection: forbid 3-of-4 filled 2×2 blocks
    occupied_grid = [[False] * GRID_SIZE_CELLS for _ in range(GRID_SIZE_CELLS)]
    for (col, row) in occupied_cells:
        occupied_grid[row][col] = True
    for col in range(GRID_SIZE_CELLS - 1):
        for row in range(GRID_SIZE_CELLS - 1):
            block = [occupied_grid[row][col],
                     occupied_grid[row][col + 1],
                     occupied_grid[row + 1][col],
                     occupied_grid[row + 1][col + 1]]
            if sum(block) == 3:
                return False

    # Basic turning-radius check near the bay: require at least a 2×2 open block west of bay
    parking_bay_cells = get_parking_bay_cells()
    min_col = min(c for (c, _) in parking_bay_cells)
    max_row = max(r for (_, r) in parking_bay_cells)
    pad_cells = 2  # 2×3 m = 6 m ≥ 5 m radius proxy
    for col in range(min_col - pad_cells, min_col):
        for row in range(0, max_row + 1):
            if not (0 <= col < GRID_SIZE_CELLS and 0 <= row < GRID_SIZE_CELLS):
                return False
            if (col, row) in occupied_cells:
                return False

    return True


def generate_random_obstacle_cells(obstacle_density, random_seed):
    """Generate tetromino obstacles satisfying clearance constraints."""
    random_generator = random.Random(random_seed)
    target_cell_count = int(round(GRID_SIZE_CELLS ** 2 * obstacle_density))
    start_clear_cells = {(0, GRID_SIZE_CELLS - 1), (1, GRID_SIZE_CELLS - 1)}
    parking_bay_cells = get_parking_bay_cells()

    for _ in range(1000):
        occupied_cells = set()
        while len(occupied_cells) < target_cell_count:
            shape = random_generator.choice(list(TETROMINO_SHAPES.values()))
            rotation = random_generator.choice(ROTATION_FUNCTIONS)
            offset_col = random_generator.randrange(0, GRID_SIZE_CELLS)
            offset_row = random_generator.randrange(0, GRID_SIZE_CELLS)

            new_shape_cells = set()
            for (x, y) in shape:
                rot_x, rot_y = rotation(x, y)
                col = offset_col + rot_x
                row = offset_row + rot_y
                if not (0 <= col < GRID_SIZE_CELLS and 0 <= row < GRID_SIZE_CELLS):
                    new_shape_cells = None
                    break
                new_shape_cells.add((col, row))
            if not new_shape_cells:
                continue

            # Skip if overlaps reserved areas or existing obstacles
            if (new_shape_cells & start_clear_cells or
                    new_shape_cells & parking_bay_cells or
                    new_shape_cells & occupied_cells):
                continue

            occupied_cells |= new_shape_cells

        # Overwrite parking bay (ensure clear)
        occupied_cells -= parking_bay_cells

        if has_valid_clearance(occupied_cells):
            return occupied_cells, start_clear_cells, parking_bay_cells

    # Fallback: return whatever we have; planner will handle failure gracefully
    return occupied_cells, start_clear_cells, parking_bay_cells


# ========================
# World class
# ========================

class World:
    """Main world container holding grid, obstacles, start pose, and parking goal."""

    def __init__(self,
                 n: int = 12,
                 cell: float = 3.0,
                 density: float = 0.10,
                 seed: int = 7,
                 trailer: bool = False):
        """
        Args:
            n (int): Grid size in cells (ignored; always 12×12).
            cell (float): Cell size in meters (ignored; always 3.0 m).
            density (float): Obstacle density (0–1).
            seed (int): Random seed.
            trailer (bool): Ignored parameter for backward compatibility.
        """
        occupied_cells, start_clear_cells, parking_bay_cells = \
            generate_random_obstacle_cells(density, seed)

        self.cell_size_m = GRID_CELL_SIZE_METERS
        self.grid_size_cells = GRID_SIZE_CELLS
        self.obstacle_polygons = convert_cells_to_polygons(occupied_cells)
        self.parking_info = {
            "cells": parking_bay_cells,
            "goal": {
                "pose": ((GRID_SIZE_CELLS - 1.5) * GRID_CELL_SIZE_METERS,
                         (1.5) * GRID_CELL_SIZE_METERS,
                         0.0),
                "tol_xy": 1.0,
                "tol_yaw": math.radians(10)
            }
        }

        # Start pose at NW corner, facing east
        start_x = 0.5 * GRID_CELL_SIZE_METERS
        start_y = (GRID_SIZE_CELLS - 0.5) * GRID_CELL_SIZE_METERS
        self.start_pose = State2D(start_x, start_y, 0.0)



    def obstacles_as_polygons(self):
        """Return list of obstacle polygons (for planner)."""
        return self.obstacle_polygons

