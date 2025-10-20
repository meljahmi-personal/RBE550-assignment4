# run_valet.py
"""
RBE-550 HW4 — Valet Parking Runner

What this script does:
1) Builds a procedural parking-lot world (tetromino obstacles + SE bay).
2) Draws the static scene to a PNG (always produced).
3) Runs a short Ackermann rollout demo to illustrate motion (always produced).
4) Runs Hybrid A* planning with maneuverable params.
   - Tries several goal headings (0, ±90°, 180°) to match how cars park.
   - If the first seed fails, scans additional seeds and REBUILDS the planner per world.
5) On success, saves: planned_path.png and car_valet.gif.

Usage examples:
  python3 run_valet.py --vehicle car --seed 0 --density 0.06
  python3 run_valet.py --vehicle car --density 0.00    # no obstacles (sanity)
"""
import numpy as np
import math
import random
import argparse
from src_env.world import World
from src_env.pose import Pose
from vehicles.diffdrive import DiffDrive
from vehicles.ackermann import Ackermann
from vehicles.truck_trailer import TruckTrailerFollower
from geom.polygons import oriented_box
from geom.collision import poly_intersect_sat, first_collision
from sim.simulate import rollout_ackermann
from sim.animate import save_png, save_path_png, save_gif_frames
from plan.hybrid_astar import HybridAStar, SAFETY_MARGIN_M, MAX_EDGE_SAMPLE_SPACING
from geom.collision import first_collision
from sim.animate import save_png, save_path_png, save_gif_frames



# Parked orientations to try at the bay center (helps feasibility)
YAW_SET = (0.0, math.pi/2, math.pi, -math.pi/2)


def build_planner(world, model,
                  dt=0.05,
                  step_T=1.00,
                  steer_set=(-0.60, -0.45, -0.30, 0.0, 0.30, 0.45, 0.60),
                  speed_set=(-1.0, 1.0),
                  grid_cell=1.0,
                  theta_bin=math.radians(30)):
    """
    Configure a Hybrid A* planner.
    Default uses larger rollout (step_T=1.00) for faster progress.
    Callers can override step_T/steer_set for a finer retry.
    """
    return HybridAStar(
        world, model,
        dt=dt,
        step_T=step_T,
        steer_set=steer_set,
        speed_set=speed_set,
        grid_cell=grid_cell,
        theta_bin=theta_bin,
    )



def try_plan_for_world(world, model):
    for gth in YAW_SET:
        gx, gy, _ = world.parking_info["goal"]["pose"]
        world.parking_info["goal"]["pose"] = (gx, gy, gth)
        
        planner = HybridAStar(
            world, model,
            dt=0.05,
            step_T=1.00,  # was 0.30  ← makes each expansion actually go somewhere
            steer_set=(-0.60, -0.45, -0.30, 0.0, 0.30, 0.45, 0.60),  # a touch richer
            speed_set=(-1.0, 1.0),
            grid_cell=1.0,
            theta_bin=math.radians(30),
        )

        path = planner.plan(world.start_pose, world.parking_info["goal"], iters_limit=200000)
        if path:
            return path
    return None
    

import numpy as np, random

def interpolate_edge(a, b, k=20):
    """Linearly interpolate k poses between a=(x,y,th) and b=(x,y,th)."""
    ax, ay, ath = a
    bx, by, bth = b
    xs = np.linspace(ax, bx, k)
    ys = np.linspace(ay, by, k)
    ths = np.linspace(ath, bth, k)
    return list(zip(xs, ys, ths))


def shortcut(planner, path, trials=200, checks=20):
    """
    Attempt to shortcut the path by replacing subsegments that are collision-free,
    using the planner's own edge-collision checker.
    """

    pts = path[:]  # keep full (x,y,theta) poses
    edits = 0

    for _ in range(trials):
        if len(pts) < 3:
            break
        # Avoid picking the very first/last pose (stabilizes smoothing)
        i, j = sorted(random.sample(range(1, len(pts)-1), 2))
        if j - i <= 1:
            continue
        seg = interpolate_edge(pts[i], pts[j], k=checks)

        # planner._edge_collision may return bool or (bool, idx). Handle both.
        res = planner._edge_collision(seg)
        collides = res[0] if isinstance(res, tuple) else bool(res)

        if not collides:
            pts = pts[:i+1] + pts[j:]
            edits += 1

    # Ensure the smoothed path still has enough waypoints
    if len(pts) < 10 or edits == 0:
        return path  # revert to the original if oversimplified

    return pts


def choose_start_heading(world, model, obstacle_polygons,
                         safety_margin_m=0.20,
                         probe_distances_m=(1.5, 0.6),
                         sample_step_m=0.10):
    """
    Pick a start yaw that points into free space from the NW start.
    Tries headings in [east (0), south (-pi/2)] and returns the chosen theta.
    If neither probe is clear, returns the current theta unchanged.

    Uses the same inflated-vehicle footprint + discrete sampling you use in planning.
    """
    import math
    from geom.polygons import oriented_box
    from geom.collision import first_collision

    # Effective vehicle dimensions (inflated)
    veh_len = getattr(model, "length", getattr(model, "truck_len", 4.5))
    veh_wid = getattr(model, "width",  getattr(model, "truck_w",  2.0))
    L_eff = veh_len + 2.0 * safety_margin_m
    W_eff = veh_wid + 2.0 * safety_margin_m

    map_size = world.grid_size_cells * world.cell_size_m

    def within_bounds(poly, eps=1e-3):
        for (px, py) in poly:
            if px < -eps or py < -eps or px > map_size + eps or py > map_size + eps:
                return False
        return True

    def straight_feasible(x, y, th, dist_m):
        steps = max(1, int(abs(dist_m) / sample_step_m))
        dx = math.cos(th) * dist_m / steps
        dy = math.sin(th) * dist_m / steps
        cx, cy = x, y
        for _ in range(steps + 1):
            poly = oriented_box((cx, cy), L_eff, W_eff, th)
            if not within_bounds(poly):
                return False
            hit, _ = first_collision([poly], obstacle_polygons)
            if hit is not None:
                return False
            cx += dx; cy += dy
        return True

    x0, y0 = world.start_pose.x, world.start_pose.y
    candidates = (0.0, -math.pi/2)  # east, south from NW

    for d in probe_distances_m:                 # try longer, then shorter probe
        for th in candidates:                   # try east, then south
            if straight_feasible(x0, y0, th, d):
                return th

    return world.start_pose.theta               # unchanged if both fail


def main():
    # =========================================================
    # CLI
    # =========================================================
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0, help="world random seed")
    ap.add_argument("--density", type=float, default=0.10, help="obstacle occupancy (0..~0.15)")
    ap.add_argument("--vehicle", choices=["robot", "car", "truck"], default="car")
    args = ap.parse_args()

    # =========================================================
    # WORLD + VEHICLE
    # =========================================================
    trailer = (args.vehicle == "truck")
    world = World(n=12, cell=3.0, density=args.density, seed=args.seed, trailer=trailer)
    obstacle_polygons = world.obstacles_as_polygons()

    # Build parking bay polygon (SE) from this world's cells
    cell_size = world.cell_size_m
    bay_cells = world.parking_info["cells"]  # set of (col, row)
    min_col = min(c for (c, _) in bay_cells)
    max_col = max(c for (c, _) in bay_cells)
    min_row = min(r for (_, r) in bay_cells)
    max_row = max(r for (_, r) in bay_cells)
    minx =  min_col * cell_size
    maxx = (max_col + 1) * cell_size
    miny =  min_row * cell_size
    maxy = (max_row + 1) * cell_size
    bay_poly = [(minx, miny), (maxx, miny), (maxx, maxy), (minx, maxy)]
    parking_bays = [bay_poly]

    # Select vehicle model + output names
    if args.vehicle == "robot":
        model = DiffDrive()
        vehicle_length_meters = model.length
        vehicle_width_meters  = model.width
        scene_image_path = "scene_robot.png"
        output_gif_path  = "robot_valet.gif"
        planned_path_image_path = "planned_path_robot.png"
    elif args.vehicle == "car":
        model = Ackermann()
        vehicle_length_meters = model.length
        vehicle_width_meters  = model.width
        scene_image_path = "scene_car.png"
        output_gif_path  = "car_valet.gif"
        planned_path_image_path = "planned_path_car.png"
    else:  # truck (truck + trailer follower)
        model = TruckTrailerFollower()
        vehicle_length_meters = model.truck_len
        vehicle_width_meters  = model.truck_w
        scene_image_path = "scene_truck.png"
        output_gif_path  = "truck_valet.gif"
        planned_path_image_path = "planned_path_truck.png"

    # Simple, deterministic start yaw: point toward goal center
    gx, gy, _ = world.parking_info["goal"]["pose"]
    dx = gx - world.start_pose.x
    dy = gy - world.start_pose.y
    world.start_pose.theta = math.atan2(dy, dx)

    # =========================================================
    # STATIC SCENE — draw obstacles, bay, and footprint at start
    # =========================================================
    start_x, start_y, start_theta = world.start_pose.x, world.start_pose.y, world.start_pose.theta
    vehicle_polygon_start = oriented_box((start_x, start_y),
                                         vehicle_length_meters,
                                         vehicle_width_meters,
                                         start_theta)
    save_png(world, obstacle_polygons, parking_bays, vehicle_polygon_start, scene_image_path)
    print(f"wrote {scene_image_path}")

    # Start collision check (sanity)
    k0, _ = first_collision([vehicle_polygon_start], obstacle_polygons)
    print("start pose in collision?", k0 is not None)

    # ---------------------------------------------------------
    # STATIC SCENE + SHORT ROLLOUT OVERLAY (scene_path.png)
    # quick sanity check of rollout & collision (uses car-style rollout)
    # ---------------------------------------------------------
    trajectory_demo = rollout_ackermann(world.start_pose, v=1.0, steer=0.35, T=2.0, dt=0.05)
    vehicle_polygons_demo = [oriented_box((x, y), vehicle_length_meters, vehicle_width_meters, th)
                             for (x, y, th) in trajectory_demo]
    k_demo, _ = first_collision(vehicle_polygons_demo, obstacle_polygons)
    print("edge collision on demo rollout:", (k_demo is not None), "at sample", k_demo)

    path_xy_demo = [(x, y) for (x, y, _) in trajectory_demo]
    save_path_png(world, obstacle_polygons, parking_bays, path_xy_demo, "scene_path.png")
    print("wrote scene_path.png")

    # =========================================================
    # DYNAMIC SCENE — plan, (optionally) smooth, render planned PNG + GIF
    # =========================================================
    # Plan for requested seed
    planned_path = try_plan_for_world(world, model)

    # If not found, scan other seeds (rebuild world each time)
    if planned_path is None:
        print(f"planner: no path for seed {args.seed} - scanning other seeds...")
        found = False
        for test_seed in range(0, 50):
            tmp_world = World(n=12, cell=3.0, density=args.density, seed=test_seed, trailer=trailer)
            tmp_path = try_plan_for_world(tmp_world, model)
            if tmp_path:
                world = tmp_world
                obstacle_polygons = world.obstacles_as_polygons()

                # rebuild bay from this world's parking cells
                cell_size = world.cell_size_m
                bay_cells = world.parking_info["cells"]
                min_col = min(c for (c, _) in bay_cells)
                max_col = max(c for (c, _) in bay_cells)
                min_row = min(r for (_, r) in bay_cells)
                max_row = max(r for (_, r) in bay_cells)
                minx =  min_col * cell_size
                maxx = (max_col + 1) * cell_size
                miny =  min_row * cell_size
                maxy = (max_row + 1) * cell_size
                bay_poly = [(minx, miny), (maxx, miny), (maxx, maxy), (minx, maxy)]
                parking_bays = [bay_poly]

                planned_path = tmp_path
                print(f"planner: path found with seed {test_seed}")
                found = True
                break
        if not found:
            print("planner: no path found in first 50 seeds")
            return

    print(f"planner: path with {len(planned_path)} poses")

    # Smoothing (shortcut); keep only if still collision-free
    planner_for_smooth = build_planner(world, model)
    path_smoothed = shortcut(planner_for_smooth, planned_path, trials=300, checks=30)
    print(f"smooth path with {len(path_smoothed)} poses")

    # Validate smoothed edges against inflated footprint
    L = getattr(model, "length", getattr(model, "truck_len", 4.5))
    W = getattr(model, "width",  getattr(model, "truck_w",  2.0))
    L_eff = L + 2*SAFETY_MARGIN_M
    W_eff = W + 2*SAFETY_MARGIN_M

    def edge_hits(p0, p1):
        x0,y0,th0 = p0; x1,y1,th1 = p1
        seg = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(seg / MAX_EDGE_SAMPLE_SPACING))
        for k in range(n + 1):
            t = k / n
            x = x0 + t*(x1 - x0); y = y0 + t*(y1 - y0); th = th0 + t*(th1 - th0)
            poly = oriented_box((x, y), L_eff, W_eff, th)
            if first_collision([poly], obstacle_polygons)[0] is not None:
                return True
        return False

    smooth_ok = True
    for i in range(len(path_smoothed) - 1):
        if edge_hits(path_smoothed[i], path_smoothed[i+1]):
            smooth_ok = False
            break

    path_for_gif = path_smoothed if (len(path_smoothed) >= 10 and smooth_ok) else planned_path

    # --------------------
    # Planned path PNG (polyline + goal footprint[s])
    # --------------------
    path_xy = [(x, y) for (x, y, _) in path_for_gif]

    # Main vehicle goal footprint
    goal_x, goal_y, goal_th = path_for_gif[-1]
    vehicle_polygon_goal = oriented_box(
        (goal_x, goal_y),
        vehicle_length_meters,
        vehicle_width_meters,
        goal_th,
    )
    vehicle_polys = [vehicle_polygon_goal]

    # If truck, also overlay trailer at the goal
    trailer_path = None
    if args.vehicle == "truck":
        if len(path_for_gif) >= 2:
            trailer_goal_pose = model.trailer_poses([path_for_gif[-2], path_for_gif[-1]])[-1]
        else:
            trailer_goal_pose = model.trailer_poses([path_for_gif[-1]])[-1]
        xt, yt, tht = trailer_goal_pose
        trailer_polygon_goal = oriented_box((xt, yt), model.trailer_len, model.trailer_w, tht)
        vehicle_polys.append(trailer_polygon_goal)

    save_path_png(world, obstacle_polygons, parking_bays, path_xy, planned_path_image_path,
                  vehicle_polygons=vehicle_polys)
    print(f"wrote {planned_path_image_path}")

    # --------------------
    # Animated GIF
    # --------------------
    # Per-vehicle animation pacing (bigger = slower playback)
    if args.vehicle == "car":
        gif_delay = 0.16
    elif args.vehicle == "truck":
        gif_delay = 0.12
    else:  # robot
        gif_delay = 0.10

    if args.vehicle == "truck":
        trailer_path = model.trailer_poses(path_for_gif, dt_per_sample=0.2)

    save_gif_frames(
            world,
            obstacle_polygons,
            parking_bays,
            path_for_gif,
            output_gif_path,                 # filename (5th positional)
            stride=1,                        # ensure int
            vehicle_length_meters=vehicle_length_meters,
            vehicle_width_meters=vehicle_width_meters,
            frame_delay=gif_delay,
            trailer_path=(model.trailer_poses(path_for_gif, dt_per_sample=0.2)
                          if args.vehicle == "truck" else None),
            trailer_length_meters=(model.trailer_len if args.vehicle == "truck" else None),
            trailer_width_meters=(model.trailer_w   if args.vehicle == "truck" else None),
        )



    print(f"wrote {planned_path_image_path} and {output_gif_path}")




if __name__ == "__main__":
    main()

