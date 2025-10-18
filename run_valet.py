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
        gx, gy, _ = world.parking_info["goal"]["pose"]
        world.parking_info["goal"]["pose"] = (gx, gy, gth)

        planner = build_planner(world, model)
        path = planner.plan(world.start_pose, world.parking_info["goal"], iters_limit=200000)      
        if path:
            return path
    return None



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
    obstacle_polygons = world.obstacles_as_polygons()
     
    # Build a single bay polygon from the SE bay cell set
    cell_size = world.cell_size_m
    bay_cells = world.parking_info["cells"]  # set of (col,row)

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
    
    if args.vehicle == "robot":
        # Differential drive delivery robot
        model = DiffDrive()
        vehicle_length_meters = model.length
        vehicle_width_meters  = model.width
        scene_image_path = "scene_robot.png"
        output_gif_path  = "robot_valet.gif"
        planned_path_image_path = "planned_path_robot.png"

    elif args.vehicle == "car":
        # Ackermann-steered police car
        model = Ackermann()
        vehicle_length_meters = model.length
        vehicle_width_meters  = model.width
        scene_image_path = "scene_car.png"
        output_gif_path  = "car_valet.gif"
        planned_path_image_path = "planned_path_car.png"

    else:
        # Truck with single trailer
        model = TruckTrailerFollower()
        vehicle_length_meters = model.truck_len
        vehicle_width_meters  = model.truck_w
        scene_image_path = "scene_truck.png"
        output_gif_path  = "truck_valet.gif"
        planned_path_image_path = "planned_path_truck.png"
      
    # --------------------
    # Static scene PNG with vehicle footprint at start
    # --------------------
    # Vehicle polygon at the start pose
    start_x, start_y, start_theta = world.start_pose.x, world.start_pose.y, world.start_pose.theta
    vehicle_polygon_start = oriented_box(
        (start_x, start_y),
        vehicle_length_meters,
        vehicle_width_meters,
        start_theta,
    )

    # Save static scene
    save_png(
        world,
        obstacle_polygons,
        parking_bays,
        vehicle_polygon_start,
        scene_image_path,
    )
    print(f"wrote {scene_image_path}")

    # Start collision check (sanity)
    hit = any(poly_intersect_sat(vehicle_polygon_start, op) for op in obstacle_polygons)
    print("start pose in collision?", hit)

    # --------------------
    # Short rollout demo and path overlay ("scene_path.png")
    # Purpose: quick visual sanity check that rollouts and collision checks behave as expected.
    # --------------------
    trajectory_demo = rollout_ackermann(
        world.start_pose, v=1.0, steer=0.35, T=2.0, dt=0.05
    )  # list of (x, y, theta)

    # Construct vehicle polygons along the demo trajectory for collision testing
    vehicle_polygons_demo = [
        oriented_box((x, y), vehicle_length_meters, vehicle_width_meters, theta)
        for (x, y, theta) in trajectory_demo
    ]

    # Edge-collision check against obstacles (reports first colliding sample index or None)
    k_demo, _ = first_collision(vehicle_polygons_demo, obstacle_polygons)
    print("edge collision on demo rollout:", (k_demo is not None), "at sample", k_demo)

    # Save a PNG with the demo trajectory overlaid on the scene (no goal overlay here)
    path_xy_demo = [(x, y) for (x, y, _) in trajectory_demo]
    save_path_png(
        world,
        obstacle_polygons,
        parking_bays,
        path_xy_demo,
        "scene_path.png",
    )
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
                bays = world.parking_info["bays"]
                path = tmp_path
                print(f"planner: path found with seed {test_seed}")
                break

    if path is None:
        print("planner: no path found in first 50 seeds")
        return

    # --------------------
    # Success → save outputs
    # Planner results (path, smoothing, final PNG and GIF)
    # Assumes `path` comes from planner.plan(...).
    # --------------------
    print(f"planner: path with {len(path)} poses")

    # Build a planner instance in this scope for the shortcut smoother
    planner = build_planner(world, model)

    # Optional: shortcut smoothing
    path_smoothed = shortcut(planner, path, trials=300, checks=30)
    print(f"smooth path with {len(path_smoothed)} poses")

    # Use smoothed path for still image; ensure enough frames for GIF
    planned_path = path_smoothed
    path_for_gif = planned_path if len(planned_path) >= 10 else path

    # Save planned path PNG (xy only) with the vehicle footprint drawn at the goal pose
    path_xy = [(x, y) for (x, y, _) in planned_path]
    goal_x, goal_y, goal_theta = planned_path[-1]
    vehicle_polygon_goal = oriented_box(
        (goal_x, goal_y),
        vehicle_length_meters,
        vehicle_width_meters,
        goal_theta,
    )
    save_path_png(
        world,
        obstacle_polygons,
        parking_bays,
        path_xy,
        planned_path_image_path,
        vehicle_polygon=vehicle_polygon_goal,
    )

    # Save animated GIF with moving vehicle footprint
    save_gif_frames(
        world,
        obstacle_polygons,
        parking_bays,
        path_for_gif,
        output_gif_path=output_gif_path,
        vehicle_length_meters=vehicle_length_meters,
        vehicle_width_meters=vehicle_width_meters,
    )

    print(f"wrote {planned_path_image_path} and {output_gif_path}")



    

if __name__ == "__main__":
    main()

