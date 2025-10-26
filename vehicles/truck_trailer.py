"""
RBE-550: Motion Planning — HW4 Valet Parking
Author: Mohamed Eljahmi
Date: October 2025

Description:
Implements a kinematic valet-parking simulation environment for three vehicles:
1. Delivery Robot (diff-drive)
2. Car (Ackermann steering)
3. Truck with trailer (Ackermann + hitch articulation)

Main Features:
- Random tetromino obstacle generation (12x12 grid, 3 m cells)
- SE parking bay creation
- Hybrid A* path planning under nonholonomic constraints
- SAT-based collision detection
- Animated and static visualization (PNG + GIF)

Acknowledgments:
Minor code structuring and syntax reviews were assisted by ChatGPT (GPT-5, Oct 2025).
All kinematic models, planners, and environment design were implemented and verified by the author.
"""


# vehicles/truck_trailer.py
import math
from typing import List, Tuple

def _curvature_from_path_segment(p0, p1, p2):
    """Estimate curvature kappa at p1 from three poses (x,y,theta).
    kappa ≈ Δθ / s where s is arc length between p0->p2 / 2."""
    x0,y0,t0 = p0; x1,y1,t1 = p1; x2,y2,t2 = p2
    # signed heading change around p1
    dth = (t2 - t0) * 0.5
    # wrap to [-pi,pi]
    while dth > math.pi: dth -= 2*math.pi
    while dth < -math.pi: dth += 2*math.pi
    s = 0.5 * (math.hypot(x1-x0,y1-y0) + math.hypot(x2-x1,y2-y1))
    if s < 1e-6:
        return 0.0
    return dth / s


class TruckTrailerFollower:
    """
    Follow a truck path (3-DOF poses) and compute trailer orientation.
    Truck: Ackermann with wheelbase L.
    Trailer: single hitch at distance d1 from trailer axle (assignment spec).
    """
    def __init__(self, L=3.4, d1=5.0,
                 truck_len=5.4, truck_w=2.0,
                 trailer_len=4.5, trailer_w=2.0):
        self.L = L
        self.d1 = d1
        self.truck_len = truck_len
        self.truck_w = truck_w
        self.trailer_len = trailer_len
        self.trailer_w = trailer_w

    def trailer_poses(self, path: List[Tuple[float,float,float]],
                      dt_per_sample: float = 0.2):
        """
        Given truck path [(x,y,theta), ...], return a list of trailer poses
        (x_tr, y_tr, theta_tr) using a first-order kinematic hitch model:

          theta_dot_truck = v/L * tan(delta)  => kappa = tan(delta)/L
          phi_dot = -(v/d1) * sin(phi) - v/L * tan(delta)
                  = -v*( sin(phi)/d1 + kappa )

        We avoid explicit v,delta by using kappa ≈ dtheta/ds and v ≈ ds/dt.
        """
        if not path:
            return []

        # initialize trailer aligned with truck at start
        x0,y0,th0 = path[0]
        phi = 0.0
        trailer_poses = []

        # run once to set initial trailer center at hitch-back position
        # hitch is at truck rear axle; place trailer axle d1 behind hitch along trailer heading
        # For a simple visual, put trailer center at: (x_tr, y_tr) = (x0, y0) - d1*[cos(th0), sin(th0)]
        xtr = x0 - self.d1 * math.cos(th0)
        ytr = y0 - self.d1 * math.sin(th0)
        th_tr = th0 - phi
        trailer_poses.append((xtr, ytr, th_tr))

        for i in range(1, len(path)-1):
            p0 = path[i-1]; p1 = path[i]; p2 = path[i+1]
            x,y,th = p1
            # curvature and speed estimates
            kappa = _curvature_from_path_segment(p0,p1,p2)          # 1/m
            ds = math.hypot(p2[0]-p1[0], p2[1]-p1[1])               # m per sample
            v = ds / max(dt_per_sample, 1e-6)                       # m/s approx

            # integrate articulation phi_dot = -v*( sin(phi)/d1 + kappa )
            phi_dot = -v*( math.sin(phi)/self.d1 + kappa )
            phi += phi_dot * dt_per_sample

            # trailer heading and position (axle center is d1 behind hitch)
            th_tr = th - phi
            xtr = x - self.d1 * math.cos(th_tr)
            ytr = y - self.d1 * math.sin(th_tr)
            trailer_poses.append((xtr, ytr, th_tr))

        # add last pose aligned with last truck pose
        xL,yL,thL = path[-1]
        th_tr_last = thL - phi
        xtrL = xL - self.d1 * math.cos(th_tr_last)
        ytrL = yL - self.d1 * math.sin(th_tr_last)
        trailer_poses.append((xtrL, ytrL, th_tr_last))

        return trailer_poses

    def polys_at_pose(self, truck_pose, trailer_pose):
        """Return (truck_poly, trailer_poly) rectangles for drawing/collision."""
        from geom.polygons import oriented_box
        tx,ty,tht = truck_pose
        xtr,ytr,thtr = trailer_pose
        truck_poly = oriented_box((tx,ty), self.truck_len, self.truck_w, tht)
        trailer_poly = oriented_box((xtr,ytr), self.trailer_len, self.trailer_w, thtr)
        return truck_poly, trailer_poly

