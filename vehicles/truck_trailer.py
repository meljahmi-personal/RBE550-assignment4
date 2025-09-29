# vehicles/truck_trailer.py

class TTState:
    """Simple container for truck + trailer state."""
    def __init__(self, x, y, th_truck, th_trailer):
        self.x = x
        self.y = y
        self.th_truck = th_truck    # truck heading (radians)
        self.th_trailer = th_trailer  # trailer heading (radians)

    def __repr__(self):
        return f"TTState(x={self.x}, y={self.y}, th_truck={self.th_truck}, th_trailer={self.th_trailer})"


class TruckTrailer:
    """Truck + single trailer vehicle (geometry + kinematics stub)."""
    def __init__(self, truck_len=5.40, truck_w=2.00, L=3.40,
                 trailer_len=4.50, trailer_w=2.00, d_hitch_to_axle=5.00):
        # store dimensions
        self.truck_len = truck_len
        self.truck_w   = truck_w
        self.L         = L
        self.trailer_len = trailer_len
        self.trailer_w   = trailer_w
        self.d1 = d_hitch_to_axle

    def step(self, state: TTState, v: float, steer: float, dt: float) -> TTState:
        """Very simple forward integration for truck + trailer.
        For now just a placeholder that returns the same state.
        """
        return state

