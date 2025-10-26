#!/bin/bash
set -e
cd "$(dirname "$0")/../src"

python3 run_valet.py --vehicle robot --density 0.10 --seed 5
python3 run_valet.py --vehicle car   --density 0.10 --seed 7
python3 run_valet.py --vehicle truck --density 0.08 --seed 11

