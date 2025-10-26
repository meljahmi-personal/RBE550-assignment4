# RBE550 — HW4 Valet Parking (Hybrid A*)

**Student:** Mohamed Eljahmi  
**Course:** RBE-550 Motion Planning — Fall 2025  
**Platform:** Linux (Ubuntu 22.04 / 24.04 recommended)

---

## 1. Clone and Build Instructions

This project assumes use of **Git over SSH** and a Linux environment.

```bash
# Clone using SSH
git clone git@github.com:meljahmi-personal/RBE550-assignment4.git
cd RBE550-assignment4/src
```

## Rebuild permissions for executable scripts:
# Ensure run scripts are executable
```bash
chmod +x run_all.sh
chmod +x run_valet.py
```

# Build environment
```bash
python3 -m venv hw4env
source hw4env/bin/activate
```

# Install dependencies
```bash
pip install -r requirements.txt
```
## Repository Layout
geom/          # Collision and geometry utilities (SAT-based)
plan/          # Hybrid A* planner implementation
vehicles/      # Vehicle models: diffdrive, car, truck+trailer
src_env/       # World generation, tetromino obstacles, parking bay
sim/           # Animation and rendering scripts
annotation/    # Annotated collision examples
results/       # Scene images, planned paths, and GIF animations
report/        # Main report and Section 6.x standalone PDFs
run_valet.py   # Main Hybrid A* runner
run_all.sh     # Batch runner for all vehicle types


## Running the Planner individually (these are the default values)
# Differential-drive robot
```bash
python3 run_valet.py --vehicle robot --density 0.10 --seed 5
```

# Ackermann car
```bash
python3 run_valet.py --vehicle car   --density 0.10 --seed 7
```
# Truck + trailer
```bash
python3 run_valet.py --vehicle truck --density 0.08 --seed 11
```

## Alternatively, run all simulations in sequence:
```bash
./run_all.sh
```

