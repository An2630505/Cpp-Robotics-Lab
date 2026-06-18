# 系统框架重构复盘与规划

> 重构周期：2026.06.14 — 2026.06.18
> 分支：`dev/refactor`

---

## 一、重构背景

重构前的 Cpp-Robotics-Lab 代码存在以下结构性问题：

- **算法与 pipeline 强耦合**：每个 `.cpp` 文件是独立可执行程序，自包含 `main()`、文件 I/O 和仿真循环，无法灵活组合
- **编译不统一**：Makefile 只编译部分文件，planner 目录未被纳入构建
- **数据结构重复定义**：`Pose` 在多处各自定义，缺乏共享层
- **头文件组织不一致**：`include/` 和 `src/` 目录划分混乱
- **旧代码堆积**：`main.cpp` 中大量注释的废弃代码

核心诉求：**将 pipeline 编排与算法实现分离**，Python 做编排层，C++ 做算法层。

---

## 二、重构方案

### 2.1 核心架构

```
Python pipeline（场景编排/仿真/可视化）
        ↕ pybind11
C++ pnc 算法库（核心计算, 编译为 .so）
```

### 2.2 关键技术决策

| 决策 | 选择 | 理由 |
|------|------|------|
| Python ↔ C++ 交互 | pybind11 | 同进程零开销调用；numpy ↔ Eigen 零拷贝；pytest 可直接测 C++ |
| 构建系统 | CMake | pybind11 官方推荐，三行配置 |
| Pipeline 模式 | 顺序调用脚本 | 当前阶段不需要 ROS 节点/话题机制 |
| 目录组织 | `pnc/<模块>/<算法>/` | 每个算法独立子目录，`.h` + `.cc` + `_test.cc` 三件套 |
| 命名约定 | 仿真：`sim_<场景>.py`；可视化：`sim_<场景>_visualize.py`；动画：`sim_<场景>_animate.py` |

### 2.3 目标目录结构

```
Cpp-Robotics-Lab/
├── pipeline/                  # Python 仿真场景
│   ├── sim_mpc_basic.py              #   MPC 基础验证
│   ├── sim_lane_keeping.py           #   完整车道保持
│   ├── sim_path_planning.py          #   路径规划
│   ├── sim_navigation.py             #   端到端导航
│   └── ..._visualize.py / ..._animate.py
├── pnc/                       # C++ 算法库
│   ├── common/types.h                #   共享数据结构 (Point, Pose, GridData)
│   ├── control/
│   │   ├── mpc/   {mpc.h, mpc.cc}
│   │   ├── kf/    {kf.h, kf.cc, kf_test.cc}
│   │   ├── pid/   {pid.h, pid.cc, pid_test.cc}
│   │   └── lqr/   {lqr.h, lqr.cc, lqr_test.cc}
│   ├── motion/
│   │   ├── astar/            {astar.h, astar.cc, astar_test.cc}
│   │   ├── hybrid_astar/     {hybrid_astar.h, hybrid_astar.cc}
│   │   ├── mpc_planner/      {mpc_planner.h, mpc_planner.cc}
│   │   ├── map_parser/       {map_parser.h, map_parser.cc}
│   │   ├── bicycle_model/    {bicycle_model.h, bicycle_model.cc, bicycle_model_test.cc}
│   │   └── path/             {path.h, path.cc, path_test.cc}
│   ├── prediction/                  #   (后期)
│   ├── bindings.cpp                 #   pybind11 绑定入口
│   └── CMakeLists.txt
├── map/                       # 本地输入数据 (gitignore)
├── output/                    # 仿真输出 (gitignore)
├── docs/                      # 文档
├── tools/                     # 通用工具
├── CMakeLists.txt             # 顶层构建
└── build_pnc.sh               # 编译脚本
```

---

## 三、实施过程 (四阶段)

### 阶段一：骨架搭建
- 配置 CMake + pybind11 + Eigen3
- 迁移 MPC 到 `pnc/control/mpc/`
- 写 `pipeline/sim_mpc_basic.py` 跑通最小验证
- **关键突破**：`import pnc` 成功，Python 能直接调用 C++

### 阶段二：控制模块迁移
- 迁移 KF、PID、LQR → `pnc/control/`
- 迁移 BicycleModel、Path → `pnc/motion/`
- 补齐 5 个 `_test.cc` 单元测试
- 写 `pipeline/sim_lane_keeping.py` 复现 `main.cpp`
- **踩坑**：pybind11 的 `def_readwrite` 引用语义导致 KF 状态更新失败，需传前一次控制量

### 阶段三：规划模块迁移
- 创建 `pnc/common/types.h` 统一 Point/Pose/GridData
- 迁移 A*、Hybrid A*、MPC Planner、Map Parser → `pnc/motion/`
- **踩坑**：A* 等模块存的是 `const std::vector<std::vector<int>>&` 引用，pybind11 转换出的临时对象被销毁后引用悬空，改为存值
- **踩坑**：pybind11 类型注册顺序影响，Point/Pose 需在 AStar 之前注册

### 阶段四：场景脚本 + 可视化 + 清理
- 写 `sim_path_planning.py`（A* / Hybrid A* 路径规划验证）
- 写 `sim_navigation.py`（端到端：地图 → 规划 → 控制）
- 配套 `_visualize.py` 和 `_animate.py`
- 清理 git 跟踪的数据/图片文件，更新 `.gitignore`

---

## 四、重构收益

### 4.1 开发体验提升

| 维度 | 重构前 | 重构后 |
|------|--------|--------|
| 调参 | 改 C++ 代码 → 重新编译 → 重新运行 | 改 Python 脚本参数 → 直接运行 |
| 添加算法 | 新建独立 `.cpp` + main + 文件 I/O + 编译 | `pnc/<模块>/xxx.h+.cc` + pybind11 绑定 + `./build_pnc.sh` |
| 可视化 | 写 C++ PPM 输出 / 独立 Python 脚本跨文件读取 | 统一 `_visualize.py`，同目录直接读仿真输出 |
| 测试 | 无 | `_test.cc` 独立编译，`./build_pnc.sh test` 一键跑 |

### 4.2 核心价值

- **Pipeline 与算法分离**：算法不关心数据从哪来、结果到哪去，只负责核心计算
- **C++ 做算法，Python 做编排**：各自发挥语言优势
- **统一三步工作流**：编译 → 仿真 → 可视化，操作一致

---

## 五、当前不足与改进方向

### 5.1 已识别的不足

1. **顺序调用架构** — 当前 pipeline 是顺序脚本，未来如果需要多模块并行运行（感知 + 规划 + 控制同时跑），可能需要引入 ROS 式的节点/话题机制。但当前阶段不需要。

2. **共享参数散落** — 车辆参数（质量、轴距、侧偏刚度等）在多个脚本中重复定义，改一处需同步 N 处。后续可抽取到 `pipeline/config.py` 统一管理。

3. **测试覆盖有限** — 目前只有 6 个单元测试，Hybrid A*、MPC Planner、Map Parser 缺少 `_test.cc`。

4. **pybind11 绑定有一定学习成本** — 引用语义、类型注册顺序、复杂 struct 的暴露等，新加模块需要写绑定代码。

5. **文档待完善** — 各模块缺少 API 文档和使用示例。

### 5.2 不适合当前阶段做的事

- ROS 节点/话题并行架构 — 需求未到，过度设计
- 完善 Prediction 模块 — 等 Motion 和 Control 的算法储备够丰富后再补齐

---

## 六、后续开发规划

### 近期：丰富算法储备

| 领域 | 可添加的算法 |
|------|-----------|
| 路径规划 | RRT*、DWA、PRM |
| 轨迹规划 | 多项式轨迹、贝塞尔曲线、B 样条 |
| 控制 | 滑模控制、自适应控制、H∞ |
| 状态估计 | EKF、UKF、粒子滤波 |
| 车辆模型 | 动力学模型（已部分实现）、轮胎模型 |

### 中期：工程完善

- 统一参数配置（`pipeline/config.py` 抽取共用参数）
- 补齐缺失的单元测试
- CI/CD 自动化测试
- 模块 API 文档

### 远期：Prediction + 复杂场景

- Prediction 模块（静态地图预测 + 动态障碍物行为意图预测）
- 多场景仿真（跟车、变道、交叉路口）
- 需要时再考虑 ROS 节点架构

---

## 七、开发规范备忘

### 三步工作流

```bash
./build_pnc.sh                              # 1. 编译
python pipeline/sim_<场景>.py               # 2. 仿真
python pipeline/sim_<场景>_visualize.py     # 3. 可视化
```

### 添加新算法

1. 在 `pnc/<模块>/<算法>/` 下创建 `xxx.h` + `xxx.cc` + `xxx_test.cc`
2. 在 `pnc/CMakeLists.txt` 注册源文件和测试
3. 在 `pnc/bindings.cpp` 添加 pybind11 绑定
4. `./build_pnc.sh test` 验证

### 命名约定

- C++ 文件：`.h` / `.cc` / `_test.cc`
- 场景脚本：`sim_<场景>.py` / `sim_<场景>_visualize.py` / `sim_<场景>_animate.py`
- 输出文件：`output/sim_<场景>.txt`

---

> 📅 创建：2026.06.18
> 📝 基于 6 轮 Q&A 讨论，由 Claude 和 tory 共同完成
