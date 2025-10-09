import argparse
from src_env.world import World
from src_env.pose import Pose
from vehicles.diffdrive import DiffDrive
from vehicles.ackermann import Ackermann
from vehicles.truck_trailer import TruckTrailer, TTState
from geom.polygons import oriented_box
from sim.animate import save_png
from geom.collision import poly_intersect_sat
from sim.simulate import rollout_ackermann
from geom.collision import first_collision
from geom.polygons import oriented_box
from sim.animate import save_png, save_path_png
from plan.hybrid_astar import HybridAStar

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--density", type=float, default=0.10)
    ap.add_argument("--vehicle", choices=["robot","car","truck"], default="car")
    args = ap.parse_args()

    trailer = (args.vehicle=="truck")
    world = World(n=12, cell=3.0, density=args.density, seed=args.seed, trailer=trailer)
    obstacles = world.obstacles_as_polygons()
    bays = world.parking["bays"]

    if args.vehicle == "robot":
        model = DiffDrive()
        pose = (world.start.x, world.start.y, 0.0)
        vpoly = oriented_box((pose[0],pose[1]), model.length, model.width, pose[2])
        out = "scene_robot.png"
    elif args.vehicle == "car":
        model = Ackermann()
        pose = (world.start.x, world.start.y, 0.0)
        vpoly = oriented_box((pose[0],pose[1]), model.length, model.width, pose[2])
        out = "scene_car.png"
    else:
        model = TruckTrailer()
        pose = (world.start.x, world.start.y, 0.0)
        vpoly = oriented_box((pose[0],pose[1]), model.truck_len, model.truck_w, pose[2])
        out = "scene_truck.png"

    save_png(world, obstacles, bays, vpoly, out)
    print(f"wrote {out}")

    hit = any(poly_intersect_sat(vpoly, op) for op in obstacles)
    print("start pose in collision?", hit)
    
    
    traj = rollout_ackermann(world.start, v=1.0, steer=0.35, T=2.0, dt=0.05)
    veh_polys = [oriented_box((x,y), model.length, model.width, th) for (x,y,th) in traj]
    k, j = first_collision(veh_polys, obstacles)
    print("edge collision:", (k is not None), "at sample", k)
    
    
    path_xy = [(x, y) for (x, y, th) in traj]
    save_path_png(world, obstacles, bays, path_xy, "scene_path.png")
    print("wrote scene_path.png")
    
    
    planner = HybridAStar(world, model,
                          dt=0.05, step_T=1.0,
                          steer_set=(-0.35, 0.0, 0.35),
                          speed_set=(-1.0, 1.0),
                          grid_cell=0.5,
                          theta_bin=3.14159/12)

    path = planner.plan(world.start, world.parking["goal"])
    if path is None:
        print("planner: no path found")
    else:
        print(f"planner: path with {len(path)} poses")
        # quick plot of the centerline
        path_xy = [(x,y) for (x,y,th) in path]
        save_path_png(world, obstacles, bays, path_xy, "planned_path.png")
        print("wrote planned_path.png")




if __name__ == "__main__":
    main()

