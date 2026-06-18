
# Pipeline — 仿真场景流水线

该目录包含自动驾驶仿真系统的 Python 场景脚本和地图处理工具。

## 目录结构

```
pipeline/
├── map_parser/              # 地图边界解析模块
│   ├── README.md            #   使用指南
│   ├── __init__.py           #   公开 API
│   ├── _core.py              #   图像处理管线
│   ├── _smooth.py            #   样条平滑
│   └── cli.py                #   CLI 封装
├── centerline/              # 中心线拓扑图提取模块
│   ├── README.md            #   使用指南
│   ├── __init__.py           #   公开 API
│   ├── _core.py              #   处理管线
│   ├── _skeleton.py          #   骨架化图构建
│   ├── _smooth_open.py       #   开曲线样条平滑
│   └── cli.py                #   CLI 封装
├── sim_*.py                 # 仿真场景脚本
├── sim_*_visualize.py       # 静态可视化
├── sim_*_animate.py         # 动画可视化
└── test_*.py                # 模块验证脚本
```

## 地图处理模块

### map_parser — 赛道边界提取

从赛道渲染图（JPG/PNG）中提取几何边界。

```python
from map_parser import parse_map
boundaries = parse_map("track.jpg")
# → {"outer_boundary": [...], "holes": [...], "metadata": {...}}
```

详见 [map_parser/README.md](map_parser/README.md)

### centerline — 中心线拓扑图提取

从 map_parser 输出中提取赛道中心线拓扑图（节点 + 边）。

```python
from map_parser import parse_map
from centerline import extract_centerline_graph

boundaries = parse_map("track.jpg")
graph = extract_centerline_graph(boundaries["outer_boundary"], boundaries["holes"])
# → {"nodes": [...], "edges": [...], "metadata": {...}}
```

详见 [centerline/README.md](centerline/README.md)

## 仿真场景

### 编译 C++ 算法库

```bash
# 首次：配置 + 编译
./build_pnc.sh config

# 日常：改了 C++ 代码后只编译
./build_pnc.sh

# 编译 + 运行单元测试
./build_pnc.sh test
```

### 运行仿真场景

```bash
python pipeline/<场景名>.py
```

| 场景 | 脚本 | 说明 |
|------|------|------|
| MPC 基础验证 | `sim_mpc_basic.py` | 最小 MPC 车道保持，快速验证 |
| 完整车道保持 | `sim_lane_keeping.py` | 复现 `main.cpp`，20段复合路径 |
| 路径规划 | `sim_path_planning.py` | A* / Hybrid A* 路径规划 |
| 导航仿真 | `sim_navigation.py` | 全局路径规划 + 纯追踪控制 |

仿真结果输出到 `output/<场景名>.txt`。

### 可视化

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

## 添加新算法模块 (C++)

1. 在 `pnc/<模块>/` 下写 `xxx.h` + `xxx.cc`
2. 添加 `xxx_test.cc` 单元测试
3. 在 `pnc/CMakeLists.txt` 中注册源文件和测试
4. 在 `pnc/bindings.cpp` 中添加 pybind11 绑定
5. `./build_pnc.sh test` 编译并跑测试
6. Python 侧 `import pnc; pnc.NewClass()` 即可调用