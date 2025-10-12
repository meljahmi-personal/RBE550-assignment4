# plan/hybrid_astar.py
import math, heapq
from vehicles.ackermann import Ackermann
from vehicles.base import State2D
from geom.polygons import oriented_box
from geom.collision import first_collision


IGNORE_COLLISIONS_FOR_BRINGUP = False


def _wrap(a):  # -> [-pi, pi]
    return (a + math.pi) % (2*math.pi) - math.pi

def _rollout(model, s: State2D, v, steer, T=0.3, dt=0.05):
    samples = []
    cur = State2D(s.x, s.y, s.theta)
    t = 0.0
    while t <= T + 1e-9:
        samples.append((cur.x, cur.y, cur.theta))
        cur = model.step(cur, v, steer, dt)
        t += dt
    return samples, cur

def _inside_goal(p: State2D, goal):
    gx, gy, gth = goal["pose"]
    tol_xy, tol_yaw = goal["tol_xy"], goal["tol_yaw"]
    if math.hypot(p.x - gx, p.y - gy) <= tol_xy:
        return abs(_wrap(p.theta - gth)) <= tol_yaw
    return False

def _disc(s: State2D, cell=1.0, dth=math.radians(30)):
    th = _wrap(s.theta)
    return (int(round(s.x / cell)), int(round(s.y / cell)), int(round(th / dth)))


def _heur(x, y, gx, gy, th=None, gth=None):
    h = math.hypot(x - gx, y - gy)
    return h if th is None else h + 0.5 * abs(_wrap(th - gth))  # was 0.25


class HybridAStar:
    def __init__(self, world, car: Ackermann,
                 dt=0.05, step_T=0.3,
                 steer_set=(-0.60, -0.30, 0.0, 0.30, 0.60),
                 speed_set=(-1.0, 1.0),
                 grid_cell=1.0,
                 theta_bin=math.radians(30)):
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
        if IGNORE_COLLISIONS_FOR_BRINGUP:
            return False
        if not self.obstacles:
            return False
        veh = [oriented_box((x,y), self.car.length, self.car.width, th) for (x,y,th) in samples]
        k, _ = first_collision(veh, self.obstacles)
        return k is not None


    def plan(self, start_pose, goal, iters_limit=200000):
        start = State2D(start_pose.x, start_pose.y, start_pose.theta)
        if _inside_goal(start, goal):
            return [(start.x, start.y, start.theta)]

        gx, gy, gth = goal["pose"]
        key0 = _disc(start, self.grid_cell, self.theta_bin)
        g_cost = {key0: 0.0}
        openpq = [( _heur(start.x, start.y, gx, gy, start.theta, gth), 0.0, start, key0 )]
        parent = {}
        visited = set()
        it = 0

        while openpq and it < iters_limit:
            it += 1
            f, g, s, skey = heapq.heappop(openpq)
            if it % 2000 == 0:
                print(f"expanded={it}, open={len(openpq)}")
            if skey in visited:
                continue
            visited.add(skey)

            if _inside_goal(s, goal):
                path = []
                k = skey; cur = s
                while k in parent:
                    path.append((cur.x, cur.y, cur.theta))
                    k, cur = parent[k]
                path.append((start.x, start.y, start.theta))
                path.reverse()
                return path
                

            for v in self.speed_set:
                for steer in self.steer_set:
                    samples, s2 = _rollout(self.car, s, v, steer, self.step_T, self.dt)
                    if self._edge_collision(samples):
                        continue
                    k2 = _disc(s2, self.grid_cell, self.theta_bin)
                    newg = g + self.step_T*abs(v)
                    if (k2 not in g_cost) or (newg < g_cost[k2]):
                        g_cost[k2] = newg
                        parent[k2] = (skey, s)
                        h = _heur(s2.x, s2.y, gx, gy, s2.theta, gth)
                        heapq.heappush(openpq, (newg + h, newg, s2, k2))

        return None

