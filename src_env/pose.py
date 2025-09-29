class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __repr__(self):
        return f"Pose(x={self.x}, y={self.y}, theta={self.theta})"

    def __eq__(self, other):
        if not isinstance(other, Pose):
            return False
        return (
            self.x == other.x and
            self.y == other.y and
            self.theta == other.theta
        )

