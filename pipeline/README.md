# Pipeline — 仿真场景流水线

Python 仿真脚本 + 地图处理工具 + 可视化。

## 目录结构

```
pipeline/
├── map_parser/                      # 赛道边界提取模块
│   ├── __init__.py                   #   公开 API: parse_map()
│   ├── _core.py                      #   图像处理管线 (Otsu + 轮廓提取)
│   ├── _smooth.py                    #   闭曲线样条平滑
│   └── cli.py                        #   CLI: image → JSON
├── centerline/                       # 中心线拓扑图提取模块
│   ├── __init__.py                   #   公开 API: extract_centerline_graph()
│   ├── _core.py                      #   骨架化 → 交汇点检测 → 边追踪
│   ├── _skeleton.py                  #   栅格骨架构建
│   ├── _smooth_open.py               #   开曲线样条平滑
│   └── cli.py                        #   CLI: bounds JSON → graph JSON
├── sim_mpc_basic.py                  # MPC 基础验证 (C++ pnc)
├── sim_lane_keeping.py               # 合成路径车道保持 (C++ pnc)
├── sim_lane_keeping_visualize.py     #   静态图表
├── sim_lane_keeping_animate.py       #   动画回放
├── sim_lane_keeping_real.py          # ★ 真实赛道 MPC 车道保持 (纯 Python)
├── sim_lane_keeping_real_animate.py  # ★ 真实赛道动画
├── sim_path_planning.py              # A* / Hybrid A* 路径规划 (C++ pnc)
├── sim_path_planning_visualize.py
├── sim_navigation.py                 # 全局导航仿真 (C++ pnc)
├── sim_navigation_visualize.py
├── test_map_parser.py                # map_parser 单元测试
└── test_centerline.py                # centerline 单元测试
```

---

## 地图处理模块

### map_parser — 赛道边界提取

从赛道渲染图（PNG/JPG）提取几何边界，输出世界坐标系下的闭合多边形。

```
path2.png → 灰度化 → Otsu 二值化 → RETR_CCOMP 轮廓提取
    → 世界坐标转换 → cubic periodic spline 平滑 → dict
```

```python
from map_parser import parse_map

bounds = parse_map("path2.png", pixels_per_meter=12.8, smoothing_factor=0.0,
                   num_control_points=200, resample_spacing_m=0.1,
                   has_starting_line=True)
# → {"outer_boundary": [[x,y],...], "holes": [[[x,y],...],...],
#    "starting_line": ((x1,y1),(x2,y2)), "metadata": {...}}
```

| 参数 | 说明 |
|------|------|
| `pixels_per_meter` | 像素到米的转换比例 |
| `smoothing_factor` | 样条平滑强度 (0 = 插值, >0 = 平滑) |
| `has_starting_line` | 是否检测起跑线（橙色横线） |
| `resample_spacing_m` | 重采样间距（米） |

详见 [map_parser/README.md](map_parser/README.md) · [docs/map_parser.md](../docs/map_parser.md)

### centerline — 中心线拓扑图提取

从 `map_parser` 输出中提取赛道中心线的节点-边拓扑图。

```
边界掩码 → skimage.skeletonize → 3×3 交汇点检测 → 毛刺剪除
    → KDTree 聚类 → Union-Find 合并 → 边追踪 → 样条平滑 → dict
```

```python
from map_parser import parse_map
from centerline import extract_centerline_graph

bounds = parse_map("path2.png")
graph = extract_centerline_graph(
    bounds["outer_boundary"], bounds["holes"],
    pixels_per_meter=12.8, smoothing_factor=0.02,
    starting_line=bounds.get("starting_line"))
# → {"nodes": [{"id":0,"x":...,"y":...},...],
#    "edges": [{"id":0,"from":0,"to":1,"points":[[x,y],...],"length_m":...},...],
#    "metadata": {"start_node_id": 3, ...}}
```

| 特性 | 说明 |
|------|------|
| 交汇点处理 | 度≥3 的像素 → KDTree 聚类 → Union-Find 合并重叠簇 |
| 毛刺剪除 | 自动去除长度 < threshold 的骨架末端分支 |
| 环岛处理 | 岛旁 U 形环回边的自动识别与保留 |
| 起跑线 | 自动定位起跑线中点对应的最近节点作为 `start_node_id` |

详见 [centerline/README.md](centerline/README.md) · [docs/centerline.md](../docs/centerline.md)

---

## 真实赛道仿真 (Real Track)

**零编译依赖，纯 Python 实现。** 推荐首选。

### sim_lane_keeping_real — MPC 车道保持

闭环仿真管线：`path2.png → map_parser → centerline → circuit assembly → MPC + BicycleModel + KF`

```bash
python pipeline/sim_lane_keeping_real.py
```

**仿真参数：**

| 参数 | 值 |
|------|-----|
| 车辆模型 | 4 状态误差动力学 (e_y, de_y, e_psi, de_psi) |
| MPC 预测时域 | N=40, 闭式无约束 QP (Cholesky 求解) |
| 离散化 | `scipy.linalg.expm` 精确矩阵指数 |
| 卡尔曼滤波 | 4 状态, Q=0.01I |
| 前馈控制 | 运动学 + 动力学曲率补偿 |
| 最大转向角 | ±30° |
| 仿真速度 | 10 m/s, DT=0.1s |

**输出：**
- `output/sim_lane_keeping_real.txt` — 仿真日志 (时间序列)
- `output/sim_lane_keeping_real.png` — 可视化图表

### sim_lane_keeping_real_animate — 动画回放

流畅回放仿真记录，包含放大的主视角 + 赛道总览小窗 + 实时误差曲线。

```bash
# 交互播放
python pipeline/sim_lane_keeping_real_animate.py

# 调节播放速度 (倍数)
python pipeline/sim_lane_keeping_real_animate.py --speed 0.5

# 保存为 GIF
python pipeline/sim_lane_keeping_real_animate.py --save output/animation.gif --speed 0.3

# 固定全局视角 (不放大小窗)
python pipeline/sim_lane_keeping_real_animate.py --full
```

| 参数 | 说明 |
|------|------|
| `--file` | 仿真日志路径 (默认 `output/sim_lane_keeping_real.txt`) |
| `--save` | 保存 GIF 路径 |
| `--speed` | 播放速度倍数 (默认 1.0) |
| `--interval` | 每帧毫秒数 (默认 25) |
| `--skip` | 每隔 N 步 = 1 帧 (默认 2) |
| `--full` | 固定全局视角 |

### 电路拼接与方向控制

`assemble_go_straight_circuit()` 自动将 centerline 图拼接为连续闭环。方向可在 `sim_lane_keeping_real.py` 中切换：

```python
loop_pts = assemble_go_straight_circuit(graph)
loop_pts = loop_pts[::-1]  # 反向跑
```

拼接规则：
- **3 岔路口** — 选最弯曲且 <90° 的分支入环岛
- **4+ 路口** — 选航向偏差最小的分支直行
- **环岛边** — 自动识别最短的 3↔3 度边，允许双向通行
- **起跑线** — 从 `start_node_id` 出发开始拼接

---

## C++ 模块仿真 (Synthetic Track)

需要先编译 `pnc` 库。使用合成路径（直道 + 圆弧 + S 弯）验证 C++ 算法。

### 编译

```bash
# 首次：配置 + 编译
./build_pnc.sh config

# 日常：只编译
./build_pnc.sh

# 编译 + 运行 C++ 单元测试
./build_pnc.sh test
```

### 仿真场景

| 场景 | 脚本 | 说明 |
|------|------|------|
| MPC 基础验证 | `sim_mpc_basic.py` | 直道 300 步，快速验证 C++ MPC + BicycleModel 正常 |
| 完整车道保持 | `sim_lane_keeping.py` | 直道 + 圆弧 + S 弯 20 段复合路径，800 步 |
| 路径规划 | `sim_path_planning.py` | A* 8 方向 + Hybrid A* 连续规划对比 |
| 导航仿真 | `sim_navigation.py` | A* 全局规划 + Pure Pursuit 轨迹跟踪 |

### 可视化

```bash
# 静态图表 —— 轨迹图 + 误差曲线 + 转向角
python pipeline/sim_lane_keeping_visualize.py

# 动画 —— 逐帧回放
python pipeline/sim_lane_keeping_animate.py

# 保存
python pipeline/sim_lane_keeping_visualize.py --save output/result.png
python pipeline/sim_lane_keeping_animate.py --save output/animation.gif
```

---

## 测试

```bash
# Python 模块测试
python pipeline/test_map_parser.py
python pipeline/test_centerline.py
```

---

## 添加新模块

### Python 模块

1. 在 `pipeline/<module>/` 下创建 `__init__.py` + 实现文件
2. 在 `__init__.py` 中暴露公开 API
3. 添加 `test_<module>.py`
4. 在本 README 中添加使用说明

### C++ 算法

1. 在 `pnc/<模块>/<algo>/` 下写 `xxx.h` + `xxx.cc` + `xxx_test.cc`
2. 在 `pnc/CMakeLists.txt` 中注册
3. 在 `pnc/bindings.cpp` 中添加 pybind11 绑定
4. `./build_pnc.sh test` 编译并跑测试
5. Python 侧 `import pnc; pnc.NewClass()` 即可调用
