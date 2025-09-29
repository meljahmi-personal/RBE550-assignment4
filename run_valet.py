import argparse
from env.world import World
from env.pose import Pose
from vehicles.diffdrive import DiffDrive
from vehicles.ackermann import Ackermann
from vehicles.truck_trailer import TruckTrailer, TTState
from geom.polygons import oriented_box
from sim.animate import save_png

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

if __name__ == "__main__":
    main()

