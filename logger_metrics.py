"""
logger_metrics.py
Simple experiment logger for RBE-550 HW4 Valet Parking.
Records planning metrics (vehicle, density, seed, runtime, nodes expanded, path length, etc.)
and appends them to a CSV file for analysis.
"""

import csv
import math
import os
from datetime import datetime

CSV_FILE = "planner_metrics.csv"


def compute_path_length(path):
    """Compute total XY length of a path = sum of Euclidean segment distances."""
    if not path or len(path) < 2:
        return 0.0
    length = 0.0
    for (x0, y0, _), (x1, y1, _) in zip(path[:-1], path[1:]):
        length += math.hypot(x1 - x0, y1 - y0)
    return length


def log_metrics(vehicle, density, seed, runtime_s, expanded_nodes, path):
    """
    Append one line of metrics to planner_metrics.csv.
    Automatically creates header if the file does not exist.
    """
    path_length = compute_path_length(path)
    num_poses = len(path) if path else 0
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    row = {
        "timestamp": now,
        "vehicle": vehicle,
        "density": f"{density:.2f}",
        "seed": seed,
        "runtime_s": f"{runtime_s:.2f}",
        "expanded_nodes": expanded_nodes,
        "path_length_m": f"{path_length:.2f}",
        "poses": num_poses,
    }

    header = list(row.keys())
    file_exists = os.path.isfile(CSV_FILE)

    with open(CSV_FILE, mode="a", newline="") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=header)
        if not file_exists:
            writer.writeheader()
        writer.writerow(row)

