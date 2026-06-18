# PNC 系统框架重构方案

> 本文档记录 Cpp-Robotics-Lab 项目的系统框架重构需求分析、方案设计及分阶段实施策略。

---

## 1. 项目背景与目标

### 1.1 项目定位

Cpp-Robotics-Lab 是一个**自动驾驶 PNC（Planning, Navigation, Control）算法学习与实验平台**。

长期目标：
- 实现一个**完整的 PNC 仿真系统**（从感知输入到底盘控制的端到端 pipeline）
- 沉淀为一个**可复用的 C++ PNC 算法库**，供仿真系统调用

开发理念：
- **敏捷迭代**：不过度设计，先满足当下需求
- **算法优先**：关注算法正确性和仿真验证，构建工具和框架服务于算法开发
- 每个阶段有明确的实现指标和预期目标

### 1.2 当前问题分析

当前项目虽已积累多个算法实现（MPC 控制、A* / Hybrid A* 规划、卡尔曼滤波、自行车模型等），但存在以下结构性问题：

| 问题 | 说明 |
|------|------|
| **无统一框架** | 每个 `.cpp` 文件是一个独立的小程序，自包含 `main()`、数据输入、算法执行和输出，本质上是一个"脚本"，而非可复用的模块 |
| **算法与 pipeline 耦合** | 算法实现、参数配置、仿真循环、文件 I/O 紧耦合在单个文件中，无法灵活组合 |
| **编译不统一** | Makefile 只编译 `src/*.cpp` 和根目录 `*.cpp`，`src/planner/` 下的文件未被纳入统一构建 |
| **数据结构重复定义** | 如 `Pose` 结构体在 `mpc_planner.h` 和 `hybrid_astar.cpp` 中各自定义，缺乏共享基础层 |
| **头文件组织不一致** | `include/` 放控制相关头文件，但 `src/planner/` 下的头文件放在 `src/` 内 |
| **旧代码堆积** | `main.cpp` 中大量已废弃的注释代码未清理 |
| **可视化脚本散落** | 根目录和子目录下 Python 脚本分布零散 |

### 1.3 设计原则

1. **算法与 pipeline 分离**：C++ 实现算法，Python 组织 pipeline
2. **onboard（仿真系统）与 offboard（实验测试）分离**
3. **单元测试与算法同目录，命名规则 `*_test.cc`**
4. **先搭骨架，再逐步迁移**：不求一步到位

---

## 2. 总体架构设计

### 2.1 核心思想

```
Python pipeline 层（框架/编排）──pybind11──> C++ pnc 算法库（核心计算）
```

- **Python 层**：仿真脚本、pipeline 编排、可视化、用 `pytest` 跑 C++ 模块的集成测试
- **C++ 层**：所有算法实现，编译为 `.so` 动态库，通过 pybind11 暴露给 Python
- **接口方案**：pybind11（同一进程内直接函数调用，numpy ↔ Eigen 零拷贝）

### 2.2 为什么这样设计

| 决策 | 选择 | 理由 |
|------|------|------|
| Python ↔ C++ 交互 | pybind11 | 同进程零开销调用；numpy ↔ Eigen 零拷贝；pytest 可直接测 C++；无需文件 I/O 胶水代码 |
| 仿真循环驱动 | Python 侧 | 修改参数/场景不需要重新编译；可视化和数据处理自然集成 |
| Pipeline 模式 | 简单顺序调用脚本 | 当前不需要 ROS 式的节点/话题机制，不做过度设计 |
| 构建系统 | CMake | pybind11 + Eigen 的 CMake 集成最成熟，三行配置搞定 |
| 模块组织 | 按 PNC 功能模块分 | 与自动驾驶系统设计的认知模型一致 |

---

## 3. 模块划分

### 3.1 顶层模块

PNC 算法库按以下三大模块组织：

| 模块 | 职责 | 包含算法 |
|------|------|---------|
| **Motion** | 运动规划：根据感知/预测信息生成可执行轨迹 | 路径规划（A*, Hybrid A*）、轨迹规划（MPC Trajectory）、车辆模型（Bicycle Model）、地图解析、碰撞检测 |
| **Control** | 底盘控制：轨迹跟踪控制 | MPC 控制器、LQR 控制器、PID 控制器、前馈控制、卡尔曼滤波（KF） |
| **Prediction** | 障碍物行为与意图预测 | （后期实现）静态地图预测、动态障碍物行为意图预测 |

### 3.2 模块边界

```
感知信息 + 定位信息  ──> [Prediction] ──> [Motion] ──> [Control] ──> 底盘执行
                              ↑                ↑              ↑
                         后期实现          路径→轨迹       轨迹跟踪
```

- 定位（Localization）和传感器仿真**不属于 PNC 模块范围**，假设为外部输入
- 前期聚焦受限场景（简单道路、静态障碍物），Prediction 模块后期添加

---

## 4. 目录结构

### 4.1 目标结构

```
Cpp-Robotics-Lab/
├── pipeline/                  # Python 框架 — 仿真场景编排
│   ├── sim_lane_keeping.py    #   车道保持仿真
│   ├── sim_parking.py         #   (后期) 泊车仿真
│   └── ...
│
├── pnc/                       # C++ 算法库（pybind11 绑定）
│   ├── CMakeLists.txt         #   pnc 子目录构建
│   ├── bindings.cpp           #   pybind11 绑定入口
│   │
│   ├── motion/                #   运动规划模块
│   │   ├── astar.cc           #     A* 路径规划
│   │   ├── astar_test.cc      #     A* 单元测试
│   │   ├── hybrid_astar.cc    #     Hybrid A* 路径规划
│   │   ├── hybrid_astar_test.cc
│   │   ├── bicycle_model.cc   #     自行车运动学/动力学模型
│   │   ├── bicycle_model_test.cc
│   │   ├── map_parser.cc      #     PGM/YAML 地图解析
│   │   ├── map_parser_test.cc
│   │   └── ...
│   │
│   ├── control/               #   控制模块
│   │   ├── mpc.cc             #     MPC 控制器
│   │   ├── mpc_test.cc
│   │   ├── lqr.cc             #     LQR 控制器
│   │   ├── lqr_test.cc
│   │   ├── pid.cc             #     PID 控制器
│   │   ├── pid_test.cc
│   │   ├── kf.cc              #     卡尔曼滤波器
│   │   ├── kf_test.cc
│   │   └── ...
│   │
│   └── prediction/            #   预测模块（后期）
│       └── ...
│
├── tools/                     # 可视化与辅助工具
│   ├── plot_trajectory.py
│   ├── animate_search.py
│   └── ...
│
├── CMakeLists.txt             # 顶层构建
├── README.md
└── docs/                      # 文档
    ├── refactor-plan.md       #   本文档
    ├── plan.md
    └── ...
```

### 4.2 命名约定

- C++ 源文件统一用 `.cc` 后缀
- 单元测试文件命名：`<模块名>_test.cc`，放在对应模块目录下
- 头文件与源文件同名或按模块合并，初期不强制 `.h`/`.cc` 分离

---

## 5. 技术选型

### 5.1 技术栈

| 项 | 选择 | 说明 |
|----|------|------|
| 语言 | C++17 + Python 3.8+ | C++ 做算法，Python 做编排 |
| 绑定 | [pybind11](https://github.com/pybind/pybind11) | header-only，直接调用 C++ 函数 |
| 构建 | CMake ≥ 3.14 | pybind11 官方推荐，配置简洁 |
| 线性代数 | Eigen3 | 已在使用，pybind11 原生支持 numpy ↔ Eigen 映射 |
| 测试 | Catch2 或 doctest（C++ 端）+ pytest（Python 端） | 轻量 header-only 测试框架 |

### 5.2 pybind11 绑定示例

C++ 侧（`pnc/bindings.cpp`）：
```cpp
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "motion/astar.cc"   // 或独立头文件

namespace py = pybind11;

PYBIND11_MODULE(pnc, m) {
    m.def("astar", &astar, "A* path planning",
          py::arg("grid"), py::arg("start"), py::arg("goal"));
}
```

Python 侧：
```python
import pnc
path = pnc.astar(grid, start, goal)  # 直接调用，grid 是 numpy array
```

---

## 6. 分阶段实施计划

### 阶段一：骨架搭建（搭建阶段）

**目标**：建立 CMake + pybind11 构建骨架，迁移最简单的一个模块并跑通端到端流程。

**范围**：
- 配置顶层 `CMakeLists.txt` + `pnc/CMakeLists.txt`
- 引入 pybind11（FetchContent 或 find_package）
- 将现有 MPC 控制器从 `include/MPC.h` + `src/MPC.cpp` 迁移到 `pnc/control/mpc.cc`
- 写 `pnc/bindings.cpp` 暴露 MPC 给 Python
- 创建 `pipeline/sim_mpc_basic.py`，跑一个最简仿真（直线车道保持）
- 验证 Python `import pnc` 成功，仿真循环跑通

**预期产出**：
```
pnc/
├── CMakeLists.txt
├── bindings.cpp
└── control/
    └── mpc.cc
pipeline/
└── sim_mpc_basic.py
CMakeLists.txt
```

**成功指标**：
- `python -c "import pnc; print('OK')"` 成功
- 仿真脚本跑通，输出轨迹数据

---

### 阶段二：控制模块迁移

**目标**：将现有控制器全量迁移到新架构，并补齐单元测试。

**范围**：
- 迁移 `LQR` → `pnc/control/lqr.cc`
- 迁移 `PID` → `pnc/control/pid.cc`
- 迁移 `KF` → `pnc/control/kf.cc`
- 迁移 `Plant_car`（自行车模型）→ `pnc/motion/bicycle_model.cc`
- 迁移 `Path`（几何路径）→ `pnc/motion/` 或合并到适当位置
- 为每个模块编写 `_test.cc`
- 写 `pipeline/sim_lane_keeping.py`（复现当前 `main.cpp` 的车道保持仿真）

**预期产出**：完整 `pnc/control/` + 部分 `pnc/motion/`，车道保持仿真脚本

**成功指标**：
- 所有模块通过单元测试
- 车道保持仿真输出与旧版 `main.cpp` 行为一致
- 可视化脚本正常显示结果

---

### 阶段三：规划模块迁移与整合

**目标**：将 planner 目录下的算法迁移到新架构，实现"从地图到控制"的完整 pipeline。

**范围**：
- 迁移 `astar.cpp` → `pnc/motion/astar.cc`
- 迁移 `hybrid_astar.cpp` → `pnc/motion/hybrid_astar.cc`
- 迁移 `mpc_planner.cpp` → `pnc/motion/mpc_planner.cc`
- 迁移 `map_parser.cpp` + `map_parser.h` → `pnc/motion/map_parser.cc`
- 统一共享数据结构（`Pose`、网格等），放入 `pnc/common/` 或适当位置
- 写 `pipeline/sim_navigation.py`：地图 → 规划 → 控制的完整仿真
- 清理旧版 `src/` 和 `include/` 目录

**预期产出**：完整 `pnc/motion/`，地图→规划→控制端到端 pipeline

**成功指标**：
- 从 PGM 地图出发，A* 规划路径，MPC 控制车辆沿路径行驶
- 碰撞检测生效

---

### 阶段四（远期待定）：Prediction 模块 + 场景扩展

**目标**：添加 Prediction 模块，扩展多场景仿真。

**范围**：
- 实现静态地图预测
- 实现简单动态障碍物行为预测
- 多场景仿真脚本（跟车、变道、交叉路口等）
- 性能优化与代码清理

---

## 7. 与旧代码的关系

- 阶段一~二期间，旧代码保留，新骨架在 `pnc/` + `pipeline/` 并行开发
- 阶段三完成迁移后，删除旧 `include/`、`src/`（非 planner）、根目录的 `main.cpp`、`main.h`
- 旧的 `Makefile` 不再维护，替换为 CMake
- 旧的 `output/` 目录视情况保留或整合到 pipeline 输出

---

## 8. 附录：方案对比记录

### A. Python-C++ 交互方案对比

| 方案 | 复杂度 | 性能 | 在线仿真 | 单元测试 | 结论 |
|------|:--:|:--:|:--:|:--:|------|
| 子进程 + 文件/管道 | 低 | 低 | 不适合 | 麻烦 | 不选 |
| pybind11 | 中 | 最优 | 很适合 | 很自然 | ✅ 选择 |
| Protobuf/IPC | 高 | 中 | 过度设计 | 可行但重 | 不选 |

### B. Pipeline 方案对比

| 方案 | 复杂度 | 适用场景 | 结论 |
|------|:--:|------|------|
| 简单顺序调用脚本 | 低 | 单人开发、顺序流程 | ✅ 选择（当前阶段） |
| ROS 节点/话题 | 高 | 多进程分布式 | 后期需要时再引入 |

### C. 构建系统对比

| 方案 | pybind11 支持 | 学习成本 | 结论 |
|------|:--:|:--:|------|
| Makefile | 需手写复杂编译标志 | 低（但调试成本高） | 不选 |
| CMake | 原生支持，三行配置 | 低（仅需基础） | ✅ 选择 |

---

> 📅 创建时间：2026-06-14
> 📝 基于讨论：Cpp-Robotics-Lab 框架重构需求分析
