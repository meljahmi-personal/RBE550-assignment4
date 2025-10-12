# src_env/world.py
from src_env.pose import Pose
from src_env.tetromino import place_tetrominoes
from src_env.parking import parking_bays_se

class World:
    def __init__(self, n=12, cell=3.0, density=0.10, seed=0, trailer=False):
        self.n = n
        self.cell = cell
        self.density = density
        self.seed = seed
        self.trailer = trailer

        # Occupancy in grid coords (i,j)
        self.occ = place_tetrominoes(n, target_occupancy=density, seed=seed)

        # keep NW start clear
        self._clear_cell(0, 0)
        self._clear_cell(0, 1)

        # Parking info (SE corner)
        self.parking = parking_bays_se(n, cell, trailer=trailer)

        # Clear the first bay rectangle of any obstacles
        bay0 = self.parking["bays"][0]             # 4-vertex polygon
        bx = [p[0] for p in bay0]
        by = [p[1] for p in bay0]
        self._clear_rect_cells(min(bx), min(by), max(bx), max(by))

        # Start pose at center of (0,0)
        self.start = Pose(x=cell * 0.5, y=cell * 0.5, theta=0.0)

    def _clear_rect_cells(self, x0, y0, x1, y1):
        """Remove occupied grid cells that intersect [x0,x1]×[y0,y1] in meters."""
        # subtract a tiny epsilon so exact boundary lines map correctly
        eps = 1e-9
        j0 = int(x0 // self.cell)
        j1 = int((x1 - eps) // self.cell)
        i0 = int(y0 // self.cell)
        i1 = int((y1 - eps) // self.cell)
        for i in range(i0, i1 + 1):
            for j in range(j0, j1 + 1):
                self.occ.discard((i, j))

    def _clear_cell(self, i, j):
        self.occ.discard((i, j))

    def obstacles_as_polygons(self):
        """Convert each occupied grid cell (i,j) to a 4-vertex polygon in meters."""
        polys = []
        for (i, j) in self.occ:
            x0 = j * self.cell
            y0 = i * self.cell
            x1 = (j + 1) * self.cell
            y1 = (i + 1) * self.cell
            polys.append([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
        return polys

