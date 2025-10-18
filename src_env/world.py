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
    boundary_buffer_cells = max(1, math.ceil(MIN_CORRIDOR_WIDTH_METERS / GRID_CELL_SIZE_METERS))


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
    """
    Generate tetromino obstacles satisfying clearance constraints.

    - 1-cell (3 m) clear ring along outer boundary.
    - 1-cell moat between different tetromino pieces (not within a piece).
    - NW start (2 cells) and SE parking bay reserved.
    - Final validation via has_valid_clearance(..).
    """
    random_generator = random.Random(random_seed)
    target_cell_count = int(round(GRID_SIZE_CELLS ** 2 * obstacle_density))

    
    # Reserve a safe moat for the start (NW), robust even with footprint inflation
    start_clear_cells = {
    (0, GRID_SIZE_CELLS - 1), (1, GRID_SIZE_CELLS - 1),
    (0, GRID_SIZE_CELLS - 2), (1, GRID_SIZE_CELLS - 2),
    }
    parking_bay_cells = get_parking_bay_cells()
    
    
    # Rough capacity clamp so we don't ask for more than can fit with the ring
    BOUNDARY_CLEAR_RING_CELLS = 1
    inner_n = max(0, GRID_SIZE_CELLS - 2 * BOUNDARY_CLEAR_RING_CELLS)
    inner_cells = inner_n * inner_n
    reserved_cells = len(start_clear_cells | parking_bay_cells)
    max_placeable_cells = max(0, inner_cells - reserved_cells)
    target_cell_count = min(target_cell_count, max_placeable_cells)

    # Moat (gap) between different tetromino pieces, measured in cells
    MIN_GAP_BETWEEN_SHAPES = 1

    for _ in range(1000):  # resample attempts
        occupied_cells = set()

        MAX_PLACEMENT_TRIES = 5000
        failed_tries = 0

        while len(occupied_cells) < target_cell_count and failed_tries < MAX_PLACEMENT_TRIES:
            shape = random_generator.choice(list(TETROMINO_SHAPES.values()))
            rotation = random_generator.choice(ROTATION_FUNCTIONS)
            offset_col = random_generator.randrange(0, GRID_SIZE_CELLS)
            offset_row = random_generator.randrange(0, GRID_SIZE_CELLS)

            # Build candidate tetromino in grid coordinates
            new_shape_cells = set()
            for (x, y) in shape:
                rx, ry = rotation(x, y)
                col = offset_col + rx
                row = offset_row + ry
                if not (0 <= col < GRID_SIZE_CELLS and 0 <= row < GRID_SIZE_CELLS):
                    new_shape_cells = None
                    break
                new_shape_cells.add((col, row))
            if not new_shape_cells:
                failed_tries += 1
                continue

            # Skip if overlaps reserved areas or already-placed cells
            if (new_shape_cells & start_clear_cells or
                new_shape_cells & parking_bay_cells or
                new_shape_cells & occupied_cells):
                failed_tries += 1
                continue

            # Keep a clear ring from the outer boundary
            touches_boundary_ring = False
            for (c, r) in new_shape_cells:
                if not (BOUNDARY_CLEAR_RING_CELLS <= c < GRID_SIZE_CELLS - BOUNDARY_CLEAR_RING_CELLS and
                        BOUNDARY_CLEAR_RING_CELLS <= r < GRID_SIZE_CELLS - BOUNDARY_CLEAR_RING_CELLS):
                    touches_boundary_ring = True
                    break
            if touches_boundary_ring:
                failed_tries += 1
                continue

            # Enforce moat: no new cell may be Chebyshev-distance 1 from any existing cell
            too_close_to_existing = False
            for (c, r) in new_shape_cells:
                for dc in (-MIN_GAP_BETWEEN_SHAPES, 0, MIN_GAP_BETWEEN_SHAPES):
                    for dr in (-MIN_GAP_BETWEEN_SHAPES, 0, MIN_GAP_BETWEEN_SHAPES):
                        if (dc, dr) == (0, 0):
                            continue
                        if (c + dc, r + dr) in occupied_cells:
                            too_close_to_existing = True
                            break
                    if too_close_to_existing:
                        break
                if too_close_to_existing:
                    break
            if too_close_to_existing:
                failed_tries += 1
                continue

            # Accept this tetromino
            occupied_cells |= new_shape_cells

        # Ensure parking bay is clear
        occupied_cells -= parking_bay_cells

        # Final layout validation (boundary buffer, no 3/4 2×2 blocks, bay pad, etc.)
        if has_valid_clearance(occupied_cells):
            return occupied_cells, start_clear_cells, parking_bay_cells

    # Fallback: return the last attempt; planner will handle failure gracefully
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
        

        half_L = 2.6  # conservative 5.2m / 2, covers the largest vehicle
        half_W = 1.0  # conservative 2.0m / 2
        W = GRID_SIZE_CELLS * GRID_CELL_SIZE_METERS

        cx, cy, th = self.start_pose.x, self.start_pose.y, self.start_pose.theta
        req_x = abs(half_L * math.cos(th)) + abs(half_W * math.sin(th))
        req_y = abs(half_L * math.sin(th)) + abs(half_W * math.cos(th))

        cx = max(req_x + 1e-2, min(W - req_x - 1e-2, cx))
        cy = max(req_y + 1e-2, min(W - req_y - 1e-2, cy))
        self.start_pose.x, self.start_pose.y = cx, cy
        self.start_pose.theta = 0.0  # face east (into the map)

        def _pick_inward_heading(x, y, W):
            cands = (0.0, math.pi/2, math.pi, -math.pi/2)  # →, ↑, ←, ↓
            def clearance(th):
                dx, dy = math.cos(th), math.sin(th)
                ts = []
                if dx > 0:   ts.append((W - x)/dx)
                elif dx < 0: ts.append((0 - x)/dx)
                if dy > 0:   ts.append((W - y)/dy)
                elif dy < 0: ts.append((0 - y)/dy)
                return min(abs(t) for t in ts) if ts else 0.0
            return max(cands, key=clearance)

        Wm = self.grid_size_cells * self.cell_size_m
        self.start_pose.theta = _pick_inward_heading(self.start_pose.x, self.start_pose.y, Wm)


    def obstacles_as_polygons(self):
        """Return list of obstacle polygons (for planner)."""
        return self.obstacle_polygons

