#!/bin/bash
# RBE550 HW4 - Run all vehicle simulations
# Assumes: Linux, python3, and venv already activated

set -e  # stop if any command fails

python3 run_valet.py --vehicle robot --density 0.10 --seed 5
python3 run_valet.py --vehicle car   --density 0.10 --seed 7
python3 run_valet.py --vehicle truck --density 0.08 --seed 11


