# RBE550 — HW4 Valet Parking (Hybrid A*)

**Student:** Mohamed Eljahmi  
**Course:** RBE-550 Motion Planning — Fall 2025  
**Platform:** Linux (Ubuntu 22.04 / 24.04 recommended)

---

This project implements a Hybrid A* motion planner for three vehicle types (robot, car, and truck–trailer)
in a randomized valet parking environment. Each vehicle follows its respective kinematic model while
ensuring nonholonomic motion and collision-free paths using the Separating Axis Theorem (SAT).

---

## 1  Using Git

This project assumes **Git over SSH** and a **Linux environment**.

```bash
# Clone using SSH
git clone git@github.com:meljahmi-personal/RBE550-assignment4.git
cd RBE550-assignment4
```

---

## 2  Repository Layout

```
.
├── geom                                # Collision and geometry utilities (SAT-based)
│   ├── collision.py
│   └── polygons.py
├── logger_metrics.py                   # Creation of metrics for runs
├── plan                                # Hybrid A* planner implementation
│   └── hybrid_astar.py
├── README.md                           # Project documentation
├── report                              # Main report and Section 6.x PDFs and Word docs
│   ├── meljahmi_RBE550_HW4.docx
│   ├── meljahmi_RBE550_HW4.pdf
│   ├── RBE550_HW4_Section6_2_Planner_Artifacts.docx
│   ├── RBE550_HW4_Section6_2_Planner_Artifacts.pdf
│   ├── RBE550_HW4_Section6_3_collision_detection.docx
│   ├── RBE550_HW4_Section6_3_collision_detection.pdf
│   ├── RBE550_HW4_Section6_5_Path_Animation.docx
│   └── RBE550_HW4_Section6_5_Path_Animation.pdf
├── requirements.txt                    # Python module list and versions
├── run_all.sh                          # Batch runner for all vehicle types
├── run_valet.py                        # Main Hybrid A* runner
├── sim                                 # Animation and rendering scripts
│   ├── animate.py
│   └── simulate.py
├── src_env                             # World generation, tetromino obstacles, parking bay
│   ├── parking.py
│   ├── pose.py
│   ├── tetromino.py
│   └── world.py
├── system.info                         # OS + Python + library versions
└── vehicles                            # Vehicle models: diff-drive, car, truck + trailer
    ├── ackermann.py
    ├── base.py
    ├── diffdrive.py
    └── truck_trailer.py
```

*(6 directories, 28 files)*

---

## 3  Create the Python Virtual Environment

```bash
python3 -m venv hw4env
source hw4env/bin/activate
```

---

## 4  Install Dependencies

```bash
pip install -r requirements.txt
```

---

## 5  Make Scripts Executable

```bash
chmod +x run_all.sh
chmod +x run_valet.py
```

---

## 6  Running the Planner (Individual Runs)

```bash
# Differential-drive robot
python3 run_valet.py --vehicle robot --density 0.10 --seed 5

# Ackermann car
python3 run_valet.py --vehicle car   --density 0.10 --seed 7

# Truck + trailer
python3 run_valet.py --vehicle truck --density 0.08 --seed 11
```

**Alternatively**, run all simulations in sequence:

```bash
./run_all.sh
```

---

## 7  Rebuilding the Repository from Individual Files

If only the individual files are provided (as per submission rules):

1. Create a new directory.  
2. Copy all submitted files into it.  
3. Run the following commands:

```bash
chmod +x run_all.sh
chmod +x run_valet.py
chmod +x organize_submission.sh
```

Then rebuild the environment and run:

```bash
python3 -m venv hw4env
source hw4env/bin/activate
pip install -r requirements.txt
```

Finally, execute:

```bash
python3 run_valet.py --vehicle robot --density 0.10 --seed 5
python3 run_valet.py --vehicle car   --density 0.10 --seed 7
python3 run_valet.py --vehicle truck --density 0.08 --seed 11
```

or simply:

```bash
./run_all.sh
```
