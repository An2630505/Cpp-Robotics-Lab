# 开发操作手册

Cpp-Robotics-Lab 统一开发流程。所有命令在项目根目录执行。

---

## 三步工作流

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  1. 编译 C++  │ ──> │  2. 运行仿真  │ ──> │  3. 可视化    │
│     算法库    │     │     场景脚本  │     │     查看结果  │
└──────────────┘     └──────────────┘     └──────────────┘
```

### 1. 编译 C++ 算法库

```bash
# 首次：配置 + 编译
./build_pnc.sh config

# 日常：改了 C++ 代码后只编译
./build_pnc.sh

# 编译 + 运行单元测试
./build_pnc.sh test
```

### 2. 运行仿真场景

```bash
python pipeline/<场景名>.py
```

| 场景 | 脚本 | 说明 |
|------|------|------|
| MPC 基础验证 | `pipeline/sim_mpc_basic.py` | 最小 MPC 车道保持，快速验证 |
| 完整车道保持 | `pipeline/sim_lane_keeping.py` | 复现 `main.cpp`，20段复合路径 |

仿真结果输出到 `output/<场景名>.txt`。

### 3. 可视化

```bash
# 静态图表
python pipeline/<场景名>_visualize.py

# 交互式动画
python pipeline/<场景名>_animate.py

# 保存为图片 / GIF
python pipeline/<场景名>_visualize.py --save output/result.png
python pipeline/<场景名>_animate.py --save output/result.gif
```

## 完整示例 — 车道保持

```bash
# 1. 编译
./build_pnc.sh

# 2. 运行仿真
python pipeline/sim_lane_keeping.py

# 3. 查看结果
python pipeline/sim_lane_keeping_visualize.py
```

## 添加新算法模块

1. 在 `pnc/<模块>/` 下写 `xxx.h` + `xxx.cc`
2. 添加 `xxx_test.cc` 单元测试
3. 在 `pnc/CMakeLists.txt` 中注册源文件和测试
4. 在 `pnc/bindings.cpp` 中添加 pybind11 绑定
5. `./build_pnc.sh test` 编译并跑测试
6. Python 侧 `import pnc; pnc.NewClass()` 即可调用

## 目录结构

```
Cpp-Robotics-Lab/
├── pipeline/          # Python 仿真场景与地图工具
│   ├── map_parser/           #   赛道边界提取
│   ├── centerline/           #   中心线拓扑图提取
│   ├── sim_*.py              #   仿真运行
│   ├── sim_*_visualize.py    #   静态可视化
│   └── sim_*_animate.py      #   动画可视化
├── pnc/               # C++ 算法库
│   ├── bindings.cpp          #   pybind11 绑定
│   ├── CMakeLists.txt        #   构建配置
│   ├── control/              #   控制模块
│   └── motion/               #   运动规划模块
├── output/            # 仿真结果
├── tools/             # 通用工具
├── docs/              # 设计文档
│   ├── map_parser.md         #   地图解析器设计
│   └── centerline.md         #   中心线提取器设计
├── CMakeLists.txt     # 顶层构建
└── build_pnc.sh       # 编译脚本
```

## 地图处理模块

| 模块 | 文档 | 说明 |
|------|------|------|
| `map_parser` | [docs/map_parser.md](map_parser.md) | JPG → 赛道边界（outer + holes） |
| `centerline` | [docs/centerline.md](centerline.md) | 边界 → 中心线拓扑图（节点 + 边） |

## 环境

```bash
conda activate CRL
```

依赖：Eigen3, pybind11, numpy, matplotlib, opencv-python, scipy, scikit-image（均已安装）。

---

> 📅 2026-06-14 | 更新 2026-06-19
