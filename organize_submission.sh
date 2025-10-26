#!/usr/bin/env bash
# Reorganize HW4 submission from flat folder into the final repo layout.
# Run this script from the folder that currently contains all files (your `submit/` dir).
# Assumes Linux.

set -euo pipefail

echo "==> Creating directories..."
mkdir -p geom plan report sim src_env vehicles

echo "==> Moving core code files into place..."
# Geometry
[[ -f collision.py ]]   && mv -f collision.py   geom/collision.py
[[ -f polygons.py ]]    && mv -f polygons.py    geom/polygons.py

# Planner
[[ -f hybrid_astar.py ]] && mv -f hybrid_astar.py plan/hybrid_astar.py

# Simulation / viz
[[ -f animate.py ]]     && mv -f animate.py     sim/animate.py
[[ -f simulate.py ]]    && mv -f simulate.py    sim/simulate.py

# Environment
[[ -f parking.py ]]     && mv -f parking.py     src_env/parking.py
[[ -f pose.py ]]        && mv -f pose.py        src_env/pose.py
[[ -f tetromino.py ]]   && mv -f tetromino.py   src_env/tetromino.py
[[ -f world.py ]]       && mv -f world.py       src_env/world.py

# Vehicles
[[ -f ackermann.py ]]      && mv -f ackermann.py      vehicles/ackermann.py
[[ -f base.py ]]           && mv -f base.py           vehicles/base.py
[[ -f diffdrive.py ]]      && mv -f diffdrive.py      vehicles/diffdrive.py
[[ -f truck_trailer.py ]]  && mv -f truck_trailer.py  vehicles/truck_trailer.py

echo "==> Moving reports..."
# Reports (PDF + DOCX)
for f in \
  "meljahmi_RBE550_HW4.docx" \
  "meljahmi_RBE550_HW4.pdf" \
  "RBE550_HW4_Section6_2_Planner_Artifacts.docx" \
  "RBE550_HW4_Section6_2_Planner_Artifacts.pdf" \
  "RBE550_HW4_Section6_3_collision_detection.docx" \
  "RBE550_HW4_Section6_3_collision_detection.pdf" \
  "RBE550_HW4_Section6_5_Path_Animation.docx" \
  "RBE550_HW4_Section6_5_Path_Animation.pdf"
do
  [[ -f "$f" ]] && mv -f "$f" "report/$f"
done

echo "==> Leaving top-level items in place (README, runners, reqs, metrics, system info)..."
# These should remain at repo root; ensure they exist, otherwise skip.
#   README.md
#   run_valet.py
#   run_all.sh
#   requirements.txt
#   logger_metrics.py
#   system.info

echo "==> Setting file permissions..."
# Executables: only the launchers need +x
[[ -f run_all.sh     ]] && chmod 755 run_all.sh
[[ -f run_valet.py   ]] && chmod 755 run_valet.py

# Everything else as readable (code modules do not need exec bit)
# Keep logger_metrics.py as a module (no exec)
find geom plan sim src_env vehicles -type f -name "*.py" -print0 | xargs -0 chmod 644
[[ -f logger_metrics.py ]] && chmod 644 logger_metrics.py

# Common text artifacts
for f in README.md requirements.txt system.info; do
  [[ -f "$f" ]] && chmod 644 "$f"
done

echo "==> Done. Final layout:"
command -v tree >/dev/null 2>&1 && tree -L 3 || ls -R

