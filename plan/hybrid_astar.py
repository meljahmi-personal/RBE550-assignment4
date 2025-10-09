# plan/hybrid_astar.py
import math, heapq
from vehicles.ackermann import Ackermann
from vehicles.base import State2D
from geom.polygons import oriented_box
from geom.collision import first_collision

def _rollout(model, s: State2D, v, steer, T=1.0, dt=0.05):
    """Integrate one control for duration T; return list[(x,y,th)] including start."""
    poses = []
    t = 0.0
    cur = s
    while t <= T + 1e-9:
        poses.append((cur.x, cur.y, cur.theta))
        cur = model.step(cur, v, steer, dt)
        t += dt
    return poses, cur  # samples + terminal state

def _heuristic(x, y, gx, gy):
    # Euclidean is fine; heading term optional
    return math.hypot(x - gx, y - gy)

def _inside_goal(pose, goal):
    (gx, gy, gth) = goal["pose"]
    tol_xy = goal["tol_xy"]; tol_yaw = goal["tol_yaw"]
    if abs(pose.x - gx) <= tol_xy and abs(pose.y - gy) <= tol_xy:
        # normalize angle diff to [-pi, pi]
        d = (pose.theta - gth + math.pi) % (2*math.pi) - math.pi
        return abs(d) <= tol_yaw
    return False

def _discretize(s: State2D, cell=0.5, dth=math.radians(15)):
    """Quantize for visited set."""
    return (int(round(s.x / cell)), int(round(s.y / cell)), int(round(s.theta / dth)))

class HybridAStar:
    """
    Minimal Hybrid A* for the Ackermann car.
    - State: continuous (x,y,theta)
    - Controls: small set of (v, steer)
    - Success: within goal tolerances
    """
    def __init__(self, world, car: Ackermann,
                 dt=0.05, step_T=1.0,
                 steer_set=( -0.35, 0.0, 0.35 ),
                 speed_set=( -1.0, 1.0 ),
                 grid_cell=0.5,
                 theta_bin=math.radians(15)):
        self.world = world
        self.car = car
        self.dt = dt
        self.step_T = step_T
        self.steer_set = steer_set
        self.speed_set = speed_set
        self.grid_cell = grid_cell
        self.theta_bin = theta_bin

        self.obstacles = world.obstacles_as_polygons()

    def _edge_collision(self, samples):
        """Return True if any sample collides."""
        veh_polys = [oriented_box((x,y), self.car.length, self.car.width, th) for (x,y,th) in samples]
        k, j = first_collision(veh_polys, self.obstacles)
        return k is not None

    def plan(self, start_pose, goal, iters_limit=20000):
        start = State2D(start_pose.x, start_pose.y, start_pose.theta)

        # early exit if start already in goal
        if _inside_goal(start, goal):
            return [(start.x, start.y, start.theta)]

        gx, gy, _ = goal["pose"]
        openpq = []
        g_cost = {}
        parent = {}

        key0 = _discretize(start, self.grid_cell, self.theta_bin)
        g_cost[key0] = 0.0
        heapq.heappush(openpq, ( _heuristic(start.x, start.y, gx, gy), 0.0, start, key0 ))

        visited = set()

        iters = 0
        while openpq and iters < iters_limit:
            iters += 1
            f, g, s, skey = heapq.heappop(openpq)
            if skey in visited:
                continue
            visited.add(skey)

            # goal check on continuous state
            if _inside_goal(s, goal):
                # reconstruct
                path = []
                curk = skey
                cur  = s
                while curk in parent:
                    path.append( (cur.x, cur.y, cur.theta) )
                    curk, cur = parent[curk]
                path.append( (start.x, start.y, start.theta) )
                path.reverse()
                return path

            # expand
            for v in self.speed_set:
                for steer in self.steer_set:
                    samples, s2 = _rollout(self.car, s, v, steer, self.step_T, self.dt)
                    # skip if collision along edge
                    if self._edge_collision(samples):
                        continue
                    k2 = _discretize(s2, self.grid_cell, self.theta_bin)
                    newg = g + self.step_T * abs(v)  # path length cost
                    if k2 not in g_cost or newg < g_cost[k2]:
                        g_cost[k2] = newg
                        parent[k2] = (skey, s)
                        h = _heuristic(s2.x, s2.y, gx, gy)
                        heapq.heappush(openpq, (newg + h, newg, s2, k2))

        return None  # no path (or iteration limit)

