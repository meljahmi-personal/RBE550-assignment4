# sim/animate.py
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPoly


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

