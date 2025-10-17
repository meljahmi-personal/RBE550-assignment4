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
import random
import numpy as np
import math
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
from plan.hybrid_astar import HybridAStar
from geom.collision import first_collision



# Parked orientations to try at the bay center (helps feasibility)
YAW_SET = (0.0, math.pi/2, math.pi, -math.pi/2)


def build_planner(world, model):
    """
    Configure a maneuverable Hybrid A* planner.
    These settings are intentionally permissive for bring-up.
    Tighten later for nicer paths.
    """
    return HybridAStar(
            world, model,
            dt=0.05,
            step_T=1.00,  # was 0.30  ← makes each expansion actually go somewhere
            steer_set=(-0.60, -0.45, -0.30, 0.0, 0.30, 0.45, 0.60),  # a touch richer
            speed_set=(-1.0, 1.0),
            grid_cell=1.0,
            theta_bin=math.radians(30),
        )

def try_plan_for_world(world, model):
    
    """Try Hybrid A* for several possible goal headings (same bay center).
    Returns:
        path: list[(x, y, theta)] or None"""
    
    for gth in YAW_SET:
        gx, gy, _ = world.parking["goal"]["pose"]
        world.parking["goal"]["pose"] = (gx, gy, gth)

        planner = build_planner(world, model)
        path = planner.plan(world.start, world.parking["goal"], iters_limit=200000)
        if path:
            return path
    return None



def try_plan_for_world(world, model):
    for gth in YAW_SET:
        gx, gy, _ = world.parking["goal"]["pose"]
        world.parking["goal"]["pose"] = (gx, gy, gth)
        
        planner = HybridAStar(
            world, model,
            dt=0.05,
            step_T=1.00,  # was 0.30  ← makes each expansion actually go somewhere
            steer_set=(-0.60, -0.45, -0.30, 0.0, 0.30, 0.45, 0.60),  # a touch richer
            speed_set=(-1.0, 1.0),
            grid_cell=1.0,
            theta_bin=math.radians(30),
        )

        path = planner.plan(world.start, world.parking["goal"], iters_limit=200000)
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




def main():
    # --------------------
    # CLI
    # --------------------
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0, help="world random seed")
    ap.add_argument("--density", type=float, default=0.10, help="obstacle occupancy (0..~0.15)")
    ap.add_argument("--vehicle", choices=["robot", "car", "truck"], default="car")
    args = ap.parse_args()

    # --------------------
    # World + vehicle
    # --------------------
    trailer = (args.vehicle == "truck")
    world = World(n=12, cell=3.0, density=args.density, seed=args.seed, trailer=trailer)
    obstacles = world.obstacles_as_polygons()
    bays = world.parking["bays"]
    
    
    if args.vehicle == "robot":
        model = DiffDrive()
        veh_length, veh_width = model.length, model.width
        out = "scene_robot.png"
        gif_out = "robot_valet.gif"
        planned_png = "planned_path_robot.png"
    elif args.vehicle == "car":
        model = Ackermann()
        veh_length, veh_width = model.length, model.width
        out = "scene_car.png"
        gif_out = "car_valet.gif"
        planned_png = "planned_path_car.png"
    else:
        model = TruckTrailerFollower()
        veh_length, veh_width = model.truck_len, model.truck_w
        out = "scene_truck.png"
        gif_out = "truck_valet.gif"
        planned_png = "planned_path_truck.png"


    # Vehicle polygon at the start pose (for the static scene)
    start_pose = (world.start.x, world.start.y, world.start.theta)
    vpoly = oriented_box((start_pose[0], start_pose[1]), veh_length, veh_width, start_pose[2])

    # --------------------
    # Static scene PNG
    # --------------------
    save_png(world, obstacles, bays, vpoly, out)
    print(f"wrote {out}")

    # Start collision check (sanity)
    hit = any(poly_intersect_sat(vpoly, op) for op in obstacles)
    print("start pose in collision?", hit)

    # --------------------
    # Rollout demo (always produces a path PNG of a short curve)
    # --------------------
    traj = rollout_ackermann(world.start, v=1.0, steer=0.35, T=2.0, dt=0.05)
    veh_polys = [oriented_box((x, y), veh_length, veh_width, th) for (x, y, th) in traj]
    k, _ = first_collision(veh_polys, obstacles)
    print("edge collision:", (k is not None), "at sample", k)

    save_path_png(world, obstacles, bays, [(x, y) for (x, y, _) in traj], "scene_path.png")
    print("wrote scene_path.png")

    # --------------------
    # Hybrid A* — first try the requested seed
    # --------------------
    path = try_plan_for_world(world, model)

    # If not found, scan other seeds (REBUILD world + planner each time)
    if path is None:
        print(f"planner: no path for seed {args.seed} - scanning other seeds...")
        for test_seed in range(0, 50):
            tmp_world = World(n=12, cell=3.0, density=args.density, seed=test_seed, trailer=trailer)
            tmp_path = try_plan_for_world(tmp_world, model)
            if tmp_path:
                world = tmp_world
                obstacles = world.obstacles_as_polygons()
                bays = world.parking["bays"]
                path = tmp_path
                print(f"planner: path found with seed {test_seed}")
                break

    if path is None:
        print("planner: no path found in first 50 seeds")
        return


    # --------------------
    # Success → save outputs
    # --------------------
    print(f"planner: path with {len(path)} poses")

    # Build a planner instance in this scope for the smoother
    planner = build_planner(world, model)

    # Optional smoothing (comment out if not needed)
    path_smooth = shortcut(planner, path, trials=300, checks=30)
    print(f"smooth path with {len(path_smooth)} poses")

    # if smoothing left too few frames, animate the original
    path_for_gif = path_smooth if len(path_smooth) >= 10 else path

    #path_xy = [(x, y) for (x, y, th) in path_smooth]  # keep smoothed for static PNG
    #save_path_png(world, obstacles, bays, path_xy, "planned_path.png")
    #save_gif_frames(world, obstacles, bays, path_for_gif, out="car_valet.gif")  # stride=1 in function
    #print("wrote planned_path.png and car_valet.gif")
    
    path_xy = [(x, y) for (x, y, th) in path_smooth]
    save_path_png(world, obstacles, bays, path_xy, planned_png)
    save_gif_frames(world, obstacles, bays, path_for_gif, out=gif_out)
    print(f"wrote {planned_png} and {gif_out}")


    

if __name__ == "__main__":
    main()

