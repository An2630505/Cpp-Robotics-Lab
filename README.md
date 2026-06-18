<div align="center">

# Cpp-Robotics-Lab 🚗

**Autonomous Driving PNC Algorithm Learning & Experimentation Platform**

Python Orchestration · C++ Algorithms · pybind11 Bindings

[![Project: CRL](https://img.shields.io/badge/Project-CRL-blueviolet)](https://github.com/An2630505/Cpp-Robotics-Lab)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Status: Active](https://img.shields.io/badge/Status-Active-brightgreen)](https://github.com/An2630505/Cpp-Robotics-Lab)
[![GitHub Stars](https://img.shields.io/github/stars/An2630505/Cpp-Robotics-Lab?style=social)](https://github.com/An2630505/Cpp-Robotics-Lab)

</div>

---

## Overview

Cpp-Robotics-Lab is a learning and experimentation platform for autonomous driving **PNC (Planning, Navigation, Control)** algorithms.

Pipeline: **Map → Path Planning → Trajectory Planning → Control → Chassis**

### Architecture

```
Python pipeline (scene scripts / visualization / orchestration)
        ↕ pybind11
C++ pnc library (core algorithms, compiled as .so)
```

---

## Quick Start

```bash
# 1. Build C++ library
./build_pnc.sh

# 2. Run simulation
python pipeline/sim_lane_keeping.py

# 3. Visualize results
python pipeline/sim_lane_keeping_visualize.py
```

See [docs/dev-guide.md](docs/dev-guide.md) for details.

---

## Project Structure

```
Cpp-Robotics-Lab/
├── pipeline/                  # Python simulation scripts
│   ├── sim_lane_keeping.py
│   ├── sim_path_planning.py
│   ├── sim_navigation.py
│   └── *_visualize.py / *_animate.py
├── pnc/                       # C++ algorithm library
│   ├── common/types.h                #   Shared data structures
│   ├── control/                      #   Control algorithms
│   │   ├── mpc/   (MPC controller)
│   │   ├── kf/    (Kalman Filter)
│   │   ├── pid/   (PID controller)
│   │   └── lqr/   (LQR controller)
│   ├── motion/                       #   Motion planning algorithms
│   │   ├── astar/          (A* path planning)
│   │   ├── hybrid_astar/   (Hybrid A*)
│   │   ├── mpc_planner/    (Pure Pursuit trajectory planner)
│   │   ├── map_parser/     (PGM map parser)
│   │   ├── bicycle_model/  (Vehicle dynamics model)
│   │   └── path/           (Geometric path builder)
│   └── prediction/                  #   (future)
├── map/                       # Input data (gitignored)
├── output/                    # Simulation output (gitignored)
├── docs/                      # Documentation
└── build_pnc.sh               # Build script
```

---

## Algorithms

### Motion

| Algorithm | Description |
|-----------|-------------|
| A* | 8-direction discrete path planning |
| Hybrid A* | Kinematically-constrained continuous planning |
| Pure Pursuit | Trajectory tracking with bicycle model |
| Bicycle Model | Vehicle lateral dynamics |
| Path | Multi-segment (straight/arc/slalom) path |
| Map Parser | PGM/YAML occupancy grid extraction |

### Control

| Algorithm | Description |
|-----------|-------------|
| MPC | Model Predictive Control |
| LQR | Linear Quadratic Regulator |
| PID | Positional & incremental PID |
| Kalman Filter | State estimation |

---

## Adding a New Algorithm

1. Create `pnc/<module>/<algo>/xxx.h` + `xxx.cc` + `xxx_test.cc`
2. Register in `pnc/CMakeLists.txt`
3. Add pybind11 bindings in `pnc/bindings.cpp`
4. `./build_pnc.sh test` to build and verify

---

## v1.0 Highlights

> 🎉 First framework-level release

| Highlight | Description |
|-----------|-------------|
| **Pipeline/Algorithm Separation** | Python orchestrates scenes, C++ handles core computation |
| **pybind11 Integration** | `import pnc` — call C++ algorithms directly from Python |
| **Unified 3-Step Workflow** | `Build → Simulate → Visualize`, consistent across all scenes |
| **Modular Structure** | `pnc/<module>/<algo>/` with `.h` + `.cc` + `_test.cc` |
| **End-to-End Simulation** | Map → A* planning → Pure Pursuit → MPC lane keeping |
| **Unit Test Suite** | 6 standalone C++ tests, `./build_pnc.sh test` |
| **Built-in Visualization** | Static charts + animation, auto-read simulation output |

### Algorithms (10 total)

| Module | Algorithms |
|--------|-----------|
| Motion | A* · Hybrid A* · Pure Pursuit · Bicycle Model · Path · Map Parser |
| Control | MPC · LQR · PID · Kalman Filter |

---

## Version History

| Version | Description |
|---------|-------------|
| **v1.0** | Framework refactor: Python pipeline + C++ pnc library, 10 algorithms |
| Legacy | Makefile architecture, experimental prototype |

---

## License

MIT
