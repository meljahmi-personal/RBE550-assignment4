# sim/animate.py
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")  # off-screen backend for image/GIF writing
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPoly
import imageio.v2 as imageio
from geom.polygons import oriented_box



def draw_scene(ax, world, obstacles, bays, veh_poly=None):
    """Draw obstacles, parking bays, and (optionally) one vehicle polygon."""
    ax.clear()
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(0, world.grid_size_cells * world.cell_size_m)
    ax.set_ylim(0, world.grid_size_cells * world.cell_size_m)
    
    # Draw outer world boundary (visual reference)
    L = world.grid_size_cells * world.cell_size_m
    ax.plot([0, L, L, 0, 0],
            [0, 0, L, L, 0],
            color="black", linewidth=1.5)

    # 🔹 Add gridlines and ticks for visual clarity (each cell = 3 m)
    ax.set_xticks([i * world.cell_size_m for i in range(world.grid_size_cells + 1)])
    ax.set_yticks([i * world.cell_size_m for i in range(world.grid_size_cells + 1)])
    ax.grid(True, linewidth=0.5, color="#dddddd")

    # obstacles (filled)
    for poly in obstacles:
        ax.add_patch(MplPoly(poly, closed=True, fill=True, alpha=0.25))

    # parking bays (outlines)
    for bay in bays:
        ax.add_patch(MplPoly(bay, closed=True, fill=False, linewidth=2))

    # vehicle polygon (outline)
    if veh_poly is not None:
        ax.add_patch(MplPoly(veh_poly, closed=True, fill=False, linewidth=2))

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Valet — static scene")
    

def save_png(world, obstacle_polygons, bays, vehicle_polygon, out_path):
    """Render one frame and save as a PNG."""
    fig, ax = plt.subplots(figsize=(6, 6))
    draw_scene(ax, world, obstacle_polygons, bays, vehicle_polygon)
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close(fig)


def save_path_png(world, obstacle_polygons, parking_bays, path_xy, output_path, vehicle_polygon=None):
    """
    path_xy: list of (x, y) points (e.g., from your rollout)
    veh_poly: optional last vehicle polygon to overlay
    """
    fig, ax = plt.subplots(figsize=(6, 6))
    draw_scene(ax, world, obstacle_polygons, parking_bays, vehicle_polygon)
    if path_xy:
        xs, ys = zip(*path_xy)
        ax.plot(xs, ys, linewidth=2)
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    
    

def save_gif_frames(world, obstacle_polygons, parking_bays, path_for_gif, output_gif_path="valet.gif", stride=1,
                    vehicle_length_meters=None, vehicle_width_meters=None):
    """
    Save an animated GIF of the vehicle moving along `path`.

    Args:
        vehicle_length_meters (float): vehicle length (m) – required to draw footprint.
        vehicle_width_meters  (float): vehicle width  (m) – required to draw footprint.
    """

    if vehicle_length_meters is None or vehicle_width_meters is None:
        raise ValueError("vehicle_length_m and vehicle_width_m are required to draw the vehicle footprint.")

    imgs = []
    xs = [p[0] for p in path_for_gif]
    ys = [p[1] for p in path_for_gif]

    for k in range(0, len(path_for_gif), stride):
        fig, ax = plt.subplots(figsize=(6,6))
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlim(0, world.grid_size_cells * world.cell_size_m)
        ax.set_ylim(0, world.grid_size_cells * world.cell_size_m)

        # optional grid
        ax.set_xticks([i * world.cell_size_m for i in range(world.grid_size_cells + 1)])
        ax.set_yticks([i * world.cell_size_m for i in range(world.grid_size_cells + 1)])
        ax.grid(True, linewidth=0.5, color="#dddddd")

        # draw static scene
        for poly in obstacle_polygons:
            ax.add_patch(MplPoly(poly, closed=True, fill=True, alpha=0.25))
        for bay in parking_bays:
            ax.add_patch(MplPoly(bay, closed=True, fill=False, linewidth=2))

        # path so far
        ax.plot(xs[:k+1], ys[:k+1], linewidth=2)

        # vehicle footprint at current pose
        x, y, theta = path_for_gif[k]
        vehicle_poly = oriented_box((x, y), vehicle_length_meters, vehicle_width_meters, theta)
        ax.add_patch(MplPoly(vehicle_poly, closed=True, fill=False, linewidth=2))

        # render to numpy array
        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
        rgba = buf.reshape(h, w, 4)
        rgb = rgba[..., :3].copy()
        imgs.append(rgb)
        plt.close(fig)

    imageio.mimsave(output_gif_path, imgs, duration=0.05)



