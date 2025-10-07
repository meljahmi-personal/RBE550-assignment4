# env/world.py
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


        # Occupancy in grid coords (i,j). Start with empty until tetromino is ready.
        self.occ = set()
        
        # create obstacles (as occupied cells)
        self.occ = place_tetrominoes(n, target_occupancy=density, seed=seed)
        

        # Clear NW start cells (we’ll keep them clear later too)
        self._clear_cell(0, 0)
        self._clear_cell(0, 1)
      

        # Parking info (SE corner)
        self.parking = parking_bays_se(n, cell, trailer=trailer)

        # Start pose at center of (0,0) cell
        self.start = Pose(x=cell*0.5, y=cell*0.5, theta=0.0)

    def _clear_cell(self, i, j):
        try:
            self.occ.discard((i, j))
        except AttributeError:
            self.occ = set()

    def obstacles_as_polygons(self):
        """
        Convert each occupied grid cell (i,j) to a 4-vertex polygon in meters.
        If self.occ is empty, this just returns [] (no obstacles).
        """
        polys = []
        for (i, j) in self.occ:
            x0 = j * self.cell
            y0 = i * self.cell
            x1 = (j + 1) * self.cell
            y1 = (i + 1) * self.cell
            polys.append([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
        return polys

