# vehicles/diffdrive.py
import math
from vehicles.base import State2D

class DiffDrive:
    """Differential-drive (unicycle) kinematics + rectangle footprint dims."""
    def __init__(self, length: float = 0.70, width: float = 0.57):
        self.length = length   # vehicle body length (m)
        self.width  = width    # vehicle body width  (m)

    def step(self, state: State2D, v: float, w: float, dt: float) -> State2D:
        """
        Integrate one time-step.
        state: current (x, y, theta)
        v: linear velocity  [m/s]
        w: angular velocity [rad/s]
        dt: time step       [s]
        """
        nx = state.x + v * math.cos(state.theta) * dt
        ny = state.y + v * math.sin(state.theta) * dt
        nth = state.theta + w * dt
        return State2D(nx, ny, nth)

