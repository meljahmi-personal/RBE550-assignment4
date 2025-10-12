# sim/animate.py
import matplotlib
matplotlib.use("Agg")  # off-screen backend for image/GIF writing
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPoly
import imageio.v2 as imageio
import numpy as np



def draw_scene(ax, world, obstacles, bays, veh_poly=None):
    """Draw obstacles, parking bays, and (optionally) one vehicle polygon."""
    ax.clear()
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(0, world.n * world.cell)
    ax.set_ylim(0, world.n * world.cell)

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
    

def save_png(world, obstacles, bays, veh_poly, out_path):
    """Render one frame and save as a PNG."""
    fig, ax = plt.subplots(figsize=(6, 6))
    draw_scene(ax, world, obstacles, bays, veh_poly)
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close(fig)


def save_path_png(world, obstacles, bays, path_xy, out_path, veh_poly=None):
    """
    path_xy: list of (x, y) points (e.g., from your rollout)
    veh_poly: optional last vehicle polygon to overlay
    """
    fig, ax = plt.subplots(figsize=(6, 6))
    draw_scene(ax, world, obstacles, bays, veh_poly)
    if path_xy:
        xs, ys = zip(*path_xy)
        ax.plot(xs, ys, linewidth=2)
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    
    

def save_gif_frames(world, obstacles, bays, path, out="valet.gif", stride=1):
    """
    Save an animated GIF of the car moving along `path`.
    Backend-agnostic: uses buffer_rgba() instead of tostring_rgb().
    """
    import math
    imgs = []
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]

    for k in range(0, len(path), stride):
        fig, ax = plt.subplots(figsize=(6,6))
        # draw scene
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlim(0, world.n*world.cell); ax.set_ylim(0, world.n*world.cell)
        for poly in obstacles:
            ax.add_patch(MplPoly(poly, closed=True, fill=True, alpha=0.25))
        for bay in bays:
            ax.add_patch(MplPoly(bay, closed=True, fill=False, linewidth=2))

        # path so far
        ax.plot(xs[:k+1], ys[:k+1], linewidth=2)

        # draw vehicle pose as a moving dot + heading (no world.car dependency)
        x, y, th = path[k]
        ax.plot(x, y, marker='o', markersize=6)
        hx, hy = x + 0.8*math.cos(th), y + 0.8*math.sin(th)  # short heading ray
        ax.plot([x, hx], [y, hy], linewidth=2)

        # render to numpy array (backend-agnostic)
        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
        rgba = buf.reshape(h, w, 4)
        rgb = rgba[..., :3].copy()
        imgs.append(rgb)
        plt.close(fig)

    imageio.mimsave(out, imgs, duration=0.05)




