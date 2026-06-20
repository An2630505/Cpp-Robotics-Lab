<div align="center">

# Cpp-Robotics-Lab 🚗

**Autonomous Driving PNC Algorithm Learning & Experimentation Platform**

Python Pipeline · C++ Algorithms · pybind11 Bindings · Real-Track Simulation

[![Project: CRL](https://img.shields.io/badge/Project-CRL-blueviolet)](https://github.com/An2630505/Cpp-Robotics-Lab)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Status: Active](https://img.shields.io/badge/Status-Active-brightgreen)](https://github.com/An2630505/Cpp-Robotics-Lab)
[![GitHub Stars](https://img.shields.io/github/stars/An2630505/Cpp-Robotics-Lab?style=social)](https://github.com/An2630505/Cpp-Robotics-Lab)

</div>

---

## Overview

Cpp-Robotics-Lab is a learning and experimentation platform for autonomous driving **PNC (Planning, Navigation, Control)** algorithms.

**Pipeline:** Map → Centerline Extraction → Trajectory Planning → Control → Visualization

### Key Features

- 🗺️ **Real-Track Simulation** — Race track from rendered images (PNG/JPG), with outer boundary, holes/islands, and starting line
- 🛣️ **Centerline Extraction** — Skeleton-based topology graph from track boundaries, handling junctions, roundabouts, and U-shaped loops
- 🔄 **Circuit Assembly** — Auto-wiring centerline edges into a continuous loop, supporting 3-way forks (roundabout entry), 4+ way crossroads, and bidirectional traversal
- 🎯 **MPC Lane Keeping** — Model Predictive Control + Bicycle Model + Kalman Filter on real centerline
- 🚀 **Trajectory Optimization** — Hybrid A* gate-to-gate planning + SafeCorridor + B-Spline smoothing + MPC tracking
- 🎬 **Animation** — Frame-by-frame playback with zoomed-in view + track overview inset + live error charts
- 🧩 **Modular Design** — Each module (`map_parser`, `centerline`) is independently importable and testable

### Architecture

```
Python pipeline (scene scripts / visualization / orchestration)
    ├── Python-native algorithms (map parser, centerline, circuit assembly)
    └── C++ pnc library (pybind11 bindings for core control algorithms)

Real-Track Pipeline (lane keeping):
  path2.png → map_parser → bounds JSON → centerline → graph JSON
      → assemble_go_straight_circuit → Trajectory → MPC + BicycleModel + KF
      → simulation log → animate / visualize

Trajectory Optimization Pipeline:
  path2.png → bounds → centerline → circuit → occupancy grid
      → Gate generation → Hybrid A* gate-to-gate planning
      → SafeCorridor → B-Spline fitting → equal-arc resample
      → MPC simulation
```

---

## Quick Start

### Real-Track Simulation (recommended)

```bash
# 1. Run MPC lane keeping on real track
python pipeline/sim_lane_keeping_real.py

# 2. Play animation
python pipeline/sim_lane_keeping_real_animate.py

# 3. Save animation as GIF
python pipeline/sim_lane_keeping_real_animate.py --save output/animation.gif --speed 0.5
```

### C++ Module Simulation (requires build)

```bash
# 1. Build C++ library
./build_pnc.sh

# 2. Trajectory optimization (HA* + B-Spline + MPC) — full pipeline on real track
python pipeline/sim_trajectory_optimization.py
python pipeline/sim_trajectory_optimization_animate.py

# 3. Run basic MPC simulation
python pipeline/sim_mpc_basic.py

# 4. Lane keeping on synthetic path (straight + arc + S-curve)
python pipeline/sim_lane_keeping.py
python pipeline/sim_lane_keeping_animate.py

# 5. Path planning + navigation
python pipeline/sim_path_planning.py
python pipeline/sim_navigation.py
```

See [docs/dev-guide.md](docs/dev-guide.md) for details.

---

## Project Structure

```
Cpp-Robotics-Lab/
├── pipeline/                          # Python simulation scripts
│   ├── map_parser/                    #   Track boundary extraction from images
│   │   ├── _core.py                   #     Otsu + contour extraction + spline smoothing
│   │   ├── _smooth.py                 #     Cubic periodic spline resampling
│   │   └── cli.py                     #     CLI entry: image → JSON
│   ├── centerline/                    #   Centerline topology graph extraction
│   │   ├── _core.py                   #     Skeletonization + junction detection + edge tracing
│   │   ├── _skeleton.py               #     Grid skeleton from boundary mask
│   │   ├── _smooth_open.py            #     Open-curve spline smoothing
│   │   └── cli.py                     #     CLI entry: bounds JSON → graph JSON
│   ├── sim_lane_keeping_real.py       #   ★ Real-track MPC lane keeping
│   ├── sim_lane_keeping_real_animate.py  # ★ Real-track animation
│   ├── sim_trajectory_optimization.py    # ★ Trajectory optimization (HA* + B-Spline + MPC)
│   ├── sim_trajectory_optimization_animate.py  # ★ Trajectory optimization animation
│   ├── sim_lane_keeping.py            #   Synthetic-path MPC lane keeping (uses C++ pnc)
│   ├── sim_lane_keeping_animate.py    #   Synthetic-path animation
│   ├── sim_lane_keeping_visualize.py  #   Static visualization
│   ├── sim_mpc_basic.py               #   Minimal MPC validation
│   ├── sim_path_planning.py           #   A* / Hybrid A* path planning
│   ├── sim_path_planning_visualize.py
│   ├── sim_navigation.py              #   End-to-end navigation
│   ├── sim_navigation_visualize.py
│   ├── test_map_parser.py             #   Unit tests
│   └── test_centerline.py             #   Unit tests
├── pnc/                               # C++ algorithm library (pybind11)
│   ├── common/types.h                 #   Shared data structures
│   ├── control/
│   │   ├── mpc/                       #   MPC controller
│   │   ├── kf/                        #   Kalman Filter
│   │   ├── pid/                       #   PID controller
│   │   └── lqr/                       #   LQR controller
│   ├── motion/
│   │   ├── astar/                     #   A* path planning
│   │   ├── hybrid_astar/              #   Hybrid A* (with Gate planning)
│   │   ├── safe_corridor/             #   Safe corridor construction
│   │   ├── bspline/                   #   B-Spline fitting & resampling
│   │   ├── mpc_planner/               #   Pure Pursuit trajectory planner
│   │   ├── map_parser/                #   PGM map parser
│   │   ├── bicycle_model/             #   Vehicle dynamics model
│   │   └── path/                      #   Geometric path builder (straight/arc/slalom)
│   ├── bindings.cpp                   #   pybind11 glue
│   └── CMakeLists.txt
├── docs/                              # Documentation
│   ├── map_parser.md                  #   Track boundary extraction design
│   ├── centerline.md                  #   Centerline graph extraction design
│   ├── dev-guide.md                   #   Developer guide
│   ├── GIT_COMMIT_GUIDE.md            #   Commit conventions
│   └── plan.md / plan-kdl.md          #   Design plans
├── map/                               # Input data (gitignored)
├── output/                            # Simulation output (gitignored)
└── build_pnc.sh                       # C++ build script
```

---

## Modules

### Map Parser — Track Boundary Extraction

Extracts outer boundary + hole/island contours from rendered track images.

```
path2.png → grayscale → Otsu binarize → RETR_CCOMP contours
    → world coordinates (pixels_per_meter) → cubic periodic spline → JSON
```

| Feature | Detail |
|---------|--------|
| Input | PNG/JPG rendered track image |
| Method | Otsu adaptive threshold + `cv2.RETR_CCOMP` |
| Output | Outer boundary + N hole contours in world coordinates (meters) |
| Smoothing | Cubic periodic spline (`splprep per=1 k=3`), C2 continuous |
| Starting Line | Optional detection via `has_starting_line=True` |

📖 [docs/map_parser.md](docs/map_parser.md)

### Centerline — Track Centerline Topology Graph

Extracts the road centerline as a node-edge graph from track boundaries.

```
boundary mask → skeletonize → junction detection (3×3 conv)
    → spur pruning → KDTree clustering → edge tracing
    → world coordinates → spline smoothing → JSON
```

| Feature | Detail |
|---------|--------|
| Input | `map_parser` output (outer + holes) |
| Method | Grid skeletonization (`skimage.skeletonize`) |
| Output | `{nodes: [{id,x,y}], edges: [{id,from,to,points,length_m}]}` |
| Junctions | 3+ degree nodes → KDTree clustering → Union-Find merge |
| Special Handling | U-shaped loops around islands, spur removal |

📖 [docs/centerline.md](docs/centerline.md)

### Circuit Assembly — Continuous Loop from Centerline Graph

Auto-wires centerline edges into a closed circuit for continuous driving.

```
graph → edge terminals (A/B pos + tangent angles)
    → physical traversals → junction-aware greedy walk
    → continuous point array → Trajectory
```

| Feature | Detail |
|---------|--------|
| 3-way Forks | Routes to curviest branch <90° (roundabout entry) |
| 4+ way Crossroads | Goes straight (minimum heading deviation) |
| Roundabout Edge | Identifies shortest 3↔3 edge for dual traversal |
| Starting Point | Respects `start_node_id` from centerline metadata |
| Direction | Supports forward / reverse via `[::-1]` |

### Real-Track MPC Lane Keeping

Closed-loop simulation on extracted centerline.

```
Trajectory → reference (x, y, psi, kappa) at each step
    → Kalman Filter state estimation
    → MPC feedback (unconstrained QP, Cholesky solve)
    → Feedforward (kinematic + dynamic)
    → Bicycle Model step (error dynamics + kinematic pose)
```

| Parameter | Value |
|-----------|-------|
| Vehicle Model | 4-state error dynamics (e_y, de_y, e_psi, de_psi) |
| MPC Horizon | N=40, closed-form unconstrained QP |
| Discretization | `scipy.linalg.expm` exact matrix exponential |
| Kalman Filter | 4-state, Q=0.01I, adaptive measurement covariance |
| Feedforward | Kinematic + dynamic curvature compensation |
| Max Steer | ±30° |
| Sim Speed | 10 m/s, DT=0.1s |

---

## Trajectory Optimization — Hybrid A* + B-Spline + MPC

Full PNC pipeline on real track. Gate-guided Hybrid A* planning followed by B-Spline smoothing and MPC tracking.

```bash
./build_pnc.sh
python pipeline/sim_trajectory_optimization.py            # run simulation
python pipeline/sim_trajectory_optimization_animate.py    # play animation
```

**Pipeline (7 steps):**

```
path2.png → parse_map → extract_centerline_graph → assemble_go_straight_circuit
    → build_occupancy_grid → generate_gates → plan_through_gates (Hybrid A* gate-to-gate)
    → SafeCorridor.build() → BSpline.fit() → BSpline.resample()
    → MPC simulation (BicycleModel + KF + feedforward)
```

| Step | Module | Detail |
|------|--------|--------|
| 1 | map_parser | Track boundary extraction + starting line detection |
| 2 | centerline | Skeleton-based topology graph |
| 3 | circuit assembly | Auto-wired closed loop from centerline graph |
| 4 | occupancy grid | Scanline fill + dilation (0.2m cell, 0.5m margin) |
| 5 | Hybrid A* Gate planning | 15m-spaced gates, kinematically-constrained segment-by-segment |
| 6 | SafeCorridor + B-Spline | Corridor-constrained cubic B-spline fitting, 0.5m resample |
| 7 | MPC simulation | Error dynamics + Kalman Filter + curvature feedforward, 10 m/s |

**B-Spline design:**

| Aspect | Detail |
|--------|--------|
| Type | Clamped (open) cubic B-spline — trajectory starts/ends at gates |
| Basis | Cox-de Boor recurrence with correct clamped right-endpoint interpolation |
| Corridor constraint | 2 soft-projection iterations: detect violation → project → refit |
| Endpoint smoothness | Path extension + clip-back strategy to avoid clamped endpoint artifacts |

**Key parameters:** cell=0.2m, safety_margin=0.5m, gate_spacing=15m, HA* arc_step=0.6m, BSpline n_ctrl=50 degree=3, resample_spacing=0.5m, MPC N=40 DT=0.1s Vx=10m/s

---

## Algorithms

### Motion

| Algorithm | Description | Location |
|-----------|-------------|----------|
| Map Parser (Python) | Track boundary extraction from images | `pipeline/map_parser/` |
| Centerline (Python) | Skeleton-based topology graph | `pipeline/centerline/` |
| Circuit Assembly (Python) | Graph-to-loop auto-wiring | `pipeline/sim_lane_keeping_real.py` |
| A* | 8-direction discrete path planning | `pnc/motion/astar/` |
| Hybrid A* | Kinematically-constrained continuous planning (with Gate planning) | `pnc/motion/hybrid_astar/` |
| Safe Corridor | Convex constraint tube along reference path | `pnc/motion/safe_corridor/` |
| B-Spline | Cubic B-Spline fitting with corridor constraints | `pnc/motion/bspline/` |
| Pure Pursuit | Trajectory tracking with bicycle model | `pnc/motion/mpc_planner/` |
| Bicycle Model | Vehicle lateral dynamics | `pnc/motion/bicycle_model/` |
| Path | Multi-segment (straight/arc/slalom) path | `pnc/motion/path/` |
| Map Parser (C++) | PGM/YAML occupancy grid extraction | `pnc/motion/map_parser/` |

### Control

| Algorithm | Description | Location |
|-----------|-------------|----------|
| MPC | Model Predictive Control | `pnc/control/mpc/` |
| LQR | Linear Quadratic Regulator | `pnc/control/lqr/` |
| PID | Positional & incremental PID | `pnc/control/pid/` |
| Kalman Filter | State estimation | `pnc/control/kf/` |

---

## Running Tests

```bash
# Python module tests
python pipeline/test_map_parser.py
python pipeline/test_centerline.py

# C++ unit tests
./build_pnc.sh test
```

---

## Adding a New Algorithm

1. Create `pnc/<module>/<algo>/xxx.h` + `xxx.cc` + `xxx_test.cc`
2. Register in `pnc/CMakeLists.txt`
3. Add pybind11 bindings in `pnc/bindings.cpp`
4. `./build_pnc.sh test` to build and verify

For Python-only modules, add to `pipeline/<module>/` with `__init__.py` and follow the existing pattern.

---

## Version History

| Version | Description |
|---------|-------------|
| **v1.2** | Trajectory optimization: Hybrid A* Gate planning, SafeCorridor, B-Spline fitting (clamped open curve), full pipeline with MPC tracking |
| **v1.1** | Real-track pipeline: map parser, centerline extraction, circuit assembly, MPC lane keeping on real track with roundabout support, bidirectional traversal |
| **v1.0** | Framework refactor: Python pipeline + C++ pnc library, 10 algorithms, pybind11 integration |

---

## License

MIT
