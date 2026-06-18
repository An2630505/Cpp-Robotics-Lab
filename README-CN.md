<div align="center">

# Cpp-Robotics-Lab 🚗

**自动驾驶 PNC 算法学习与实验平台**

Python 编排 · C++ 算法 · pybind11 绑定

[![Project: CRL](https://img.shields.io/badge/Project-CRL-blueviolet)](https://github.com/An2630505/Cpp-Robotics-Lab)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Status: Active](https://img.shields.io/badge/Status-Active-brightgreen)](https://github.com/An2630505/Cpp-Robotics-Lab)
[![GitHub Stars](https://img.shields.io/github/stars/An2630505/Cpp-Robotics-Lab?style=social)](https://github.com/An2630505/Cpp-Robotics-Lab)

</div>

---

## 项目概述

Cpp-Robotics-Lab 是一个自动驾驶 **PNC（Planning, Navigation, Control）** 算法学习与实验平台。

目标是从感知输入到控制输出，覆盖完整的 PNC pipeline：

```
地图 → 路径规划 → 轨迹规划 → 控制 → 底盘
```

项目采用 **Python + C++ 混合架构**：
- **Python** 负责 pipeline 编排、仿真脚本、可视化
- **C++** 负责核心算法实现（通过 pybind11 编译为 Python 可调用的 `.so` 动态库）

---

## 快速开始

### 环境

```bash
conda activate CRL
```

依赖：Eigen3、pybind11、numpy、matplotlib

### 三步工作流（示例）

```bash
# 1. 编译 C++ 算法库
./build_pnc.sh

# 2. 运行仿真
python pipeline/sim_lane_keeping.py

# 3. 可视化
python pipeline/sim_lane_keeping_visualize.py
```

> 详细操作见 [docs/dev-guide.md](docs/dev-guide.md)

---

## 项目结构

```
Cpp-Robotics-Lab/
├── pipeline/                  # Python 仿真场景
│   ├── sim_lane_keeping.py           #   车道保持仿真
│   ├── sim_path_planning.py          #   路径规划验证
│   ├── sim_navigation.py             #   端到端导航
│   └── ..._visualize.py / ..._animate.py
│
├── pnc/                       # C++ 算法库
│   ├── common/types.h                #   共享数据结构
│   ├── control/                      #   控制模块
│   │   ├── mpc/   {mpc.h, mpc.cc}           #   模型预测控制
│   │   ├── kf/    {kf.h, kf.cc, kf_test.cc} #   卡尔曼滤波
│   │   ├── pid/   {pid.h, pid.cc, pid_test.cc}
│   │   └── lqr/   {lqr.h, lqr.cc, lqr_test.cc}
│   ├── motion/                       #   运动规划模块
│   │   ├── astar/            {astar.h, astar.cc, astar_test.cc}
│   │   ├── hybrid_astar/     {hybrid_astar.h, hybrid_astar.cc}
│   │   ├── mpc_planner/      {mpc_planner.h, mpc_planner.cc}
│   │   ├── map_parser/       {map_parser.h, map_parser.cc}
│   │   ├── bicycle_model/    {bicycle_model.h, bicycle_model.cc, ...}
│   │   └── path/             {path.h, path.cc, path_test.cc}
│   └── prediction/                  #   预测模块（后期）
│
├── map/                       # 本地输入数据 (gitignore)
├── output/                    # 仿真输出 (gitignore)
├── docs/                      # 文档
├── CMakeLists.txt             # 顶层构建
└── build_pnc.sh               # 编译脚本
```

---

## 算法模块

### Motion（运动规划）

| 算法 | 说明 | 状态 |
|------|------|:--:|
| A* | 8 方向离散路径规划 | ✅ |
| Hybrid A* | 运动学约束连续路径规划 | ✅ |
| Pure Pursuit | 纯追踪轨迹规划 | ✅ |
| Bicycle Model | 自行车动力学模型 | ✅ |
| Path | 多段组合路径（直道/圆弧/S弯） | ✅ |
| Map Parser | PGM/YAML 地图解析 | ✅ |

### Control（控制）

| 算法 | 说明 | 状态 |
|------|------|:--:|
| MPC | 模型预测控制 | ✅ |
| LQR | 线性二次调节器 | ✅ |
| PID | 位置式 / 增量式 PID | ✅ |
| Kalman Filter | 卡尔曼滤波状态估计 | ✅ |

### Prediction（预测）

> 后期实现 — 静态地图预测 + 动态障碍物行为意图预测

---

## 添加新算法

1. 在 `pnc/<模块>/<算法>/` 下创建 `xxx.h` + `xxx.cc` + `xxx_test.cc`
2. 在 `pnc/CMakeLists.txt` 注册源文件和测试
3. 在 `pnc/bindings.cpp` 添加 pybind11 绑定
4. `./build_pnc.sh test` 编译并验证

---

## 运行单元测试

```bash
./build_pnc.sh test
```

---

## 文档

| 文档 | 说明 |
|------|------|
| [docs/dev-guide.md](docs/dev-guide.md) | 开发操作手册 |
| [docs/refactor-plan.md](docs/refactor-plan.md) | 系统框架重构方案 |
| [docs/retrospective.md](docs/retrospective.md) | 重构复盘与规划 |

---

## v1.0 亮点

> 🎉 首次系统框架重构发布

| 亮点 | 说明 |
|------|------|
| **Pipeline 与算法分离** | Python 编排场景 + C++ 核心算法，调参无需重新编译 |
| **pybind11 混合编程** | C++ 编译为 `.so` 动态库，Python 直接 `import pnc` 调用 |
| **统一三步工作流** | `编译 → 仿真 → 可视化`，所有场景操作一致 |
| **模块化目录** | `pnc/<模块>/<算法>/` 三层结构，`.h` + `.cc` + `_test.cc` 三件套 |
| **端到端仿真** | 地图 → A* 路径规划 → 纯追踪 → MPC 车道保持，完整 PNC 管线 |
| **单元测试体系** | 6 个独立 C++ 测试，`./build_pnc.sh test` 一键运行 |
| **配套可视化** | 静态图表 + 动画，自动读取仿真输出 |

### 包含算法 (10 个)

| 模块 | 算法 |
|------|------|
| Motion | A* · Hybrid A* · Pure Pursuit · Bicycle Model · Path · Map Parser |
| Control | MPC · LQR · PID · Kalman Filter |

---

## 版本历史

| 版本 | 说明 |
|------|------|
| **v1.0** | 系统框架重构：Python pipeline + C++ pnc 库（pybind11 绑定），10 个算法 |
| 旧版 | Makefile 架构（`include/` + `src/` + `main.cpp`），实验原型 |

---

## License

MIT
