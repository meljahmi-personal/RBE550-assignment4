# sim/simulate.py
import numpy as np
from vehicles.ackermann import Ackermann
from src_env.pose import Pose
from geom.polygons import oriented_box
from vehicles.base import State2D

def rollout_ackermann(start_pose, v=1.0, steer=0.0, T=2.0, dt=0.05, wheelbase=2.80):
    """
    Roll out a simple Ackermann motion from start_pose for duration T.
    Returns a list of (x, y, theta) tuples sampled every dt.
    """
    car = Ackermann(wheelbase=wheelbase)

    # Initialize from start pose
    s = State2D(start_pose.x, start_pose.y, start_pose.theta)
    poses = []
    t = 0.0
    while t <= T + 1e-9:
        poses.append((s.x, s.y, s.theta))
        s = car.step(s, v, steer, dt)   # <-- returns State2D
        t += dt
    return poses


def poses_to_polys(poses, length, width):
    """Convert list of (x, y, theta) poses into oriented box polygons."""
    return [oriented_box((x, y), length, width, th) for (x, y, th) in poses]
