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
cd RBE550-assignment4
```

## Repository Layout
.
├── geom                                # Collision and geometry utilities (SAT-based)
│   ├── collision.py
│   └── polygons.py
├── logger_metrics.py                   # Creation of metric for the runs
├── plan                                # Hybrid A* planner implementation
│   └── hybrid_astar.py
├── README.md                           # Mark Down file.
├── report                              # Main report and Sections 6.x standalone PDFs and word documents
│   ├── meljahmi_RBE550_HW4.docx
│   ├── meljahmi_RBE550_HW4.pdf
│   ├── RBE550_HW4_Section6_2_Planner_Artifacts.docx
│   ├── RBE550_HW4_Section6_2_Planner_Artifacts.pdf
│   ├── RBE550_HW4_Section6_3_collision_detection.docx
│   ├── RBE550_HW4_Section6_3_collision_detection.pdf
│   ├── RBE550_HW4_Section6_5_Path_Animation.docx
│   └── RBE550_HW4_Section6_5_Path_Animation.pdf
├── requirements.txt                    # Python modules names and versions. To be used by pip
├── run_all.sh                          # Batch runner for all vehicle types
├── run_valet.py                        # Main Hybrid A* runner
├── sim                                 # Animation and rendering scripts
│   ├── animate.py
│   └── simulate.py
├── src_env                             # World generation, tetromino obstacles, parking bay
│   ├── parking.py
│   ├── pose.py
│   ├── tetromino.py
│   └── world.py
├── system.info                        # System info Linux version and Python modules names and versions
└── vehicles                           # Vehicle models: diffdrive, car, truck+trailer
    ├── ackermann.py
    ├── base.py
    ├── diffdrive.py
    └── truck_trailer.py


6 directories, 28 files


# 2. Build environment
```bash
python3 -m venv hw4env
source hw4env/bin/activate
```

# 3. Install dependencies
```bash
pip install -r requirements.txt
```

## 4. Rebuild permissions for executable scripts:
# Ensure run scripts are executable
```bash
chmod +x run_all.sh
chmod +x run_valet.py
```

## 5. Running the Planner individually (these are the default values)
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

