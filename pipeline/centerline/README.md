# centerline — 赛道中心线拓扑图提取

从 `map_parser` 输出的赛道边界中提取中心线拓扑图（节点 + 边）。

## 快速开始

```python
import sys; sys.path.insert(0, 'pipeline')
from map_parser import parse_map
from centerline import extract_centerline_graph

# Step 1: 解析赛道边界
boundaries = parse_map("pipeline/map_parser/path1.jpg")

# Step 2: 提取中心线拓扑图
graph = extract_centerline_graph(
    boundaries["outer_boundary"],
    boundaries["holes"],
)

# 查看结果
print(f"节点: {len(graph['nodes'])}")
for n in graph["nodes"]:
    print(f"  Node {n['id']}: ({n['x']:.1f}, {n['y']:.1f})m")

print(f"边: {len(graph['edges'])}")
for e in graph["edges"]:
    print(f"  Edge {e['id']}: node{e['from']}->node{e['to']}, {e['length_m']:.1f}m, {len(e['points'])} pts")

# 保存为 JSON
import json
with open("output/centerline_graph.json", "w") as f:
    json.dump(graph, f, indent=2)
```

## CLI 用法

```bash
# 从 map_parser 输出的 JSON 提取中心线
python pipeline/centerline/cli.py output/boundaries.json

# 保存到文件
python pipeline/centerline/cli.py output/boundaries.json -o output/graph.json

# 调整参数
python pipeline/centerline/cli.py boundaries.json --smoothing-factor 0.05 --prune-spur-length 3.0
```

## API 参考

### `extract_centerline_graph()`

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `outer_boundary` | `list[list[float]]` | (必需) | 外边界，来自 map_parser |
| `holes` | `list[list[list[float]]]` | (必需) | 孔洞列表，来自 map_parser |
| `pixels_per_meter` | `float` | `12.8` | 渲染分辨率比例 |
| `smoothing_factor` | `float` | `0.02` | 样条平滑系数 |
| `resample_spacing_m` | `float \| None` | `None`→auto | 边弧长重采样间距(m) |
| `render_resolution` | `int` | `1024` | 渲染掩码最大边长(px) |
| `prune_spur_length_m` | `float` | `2.0` | 骨架毛刺剪除阈值(m) |
| `starting_line` | `list[list[list[float]]] \| None` | `None` | 起跑线坐标，来自 `map_parser` 输出；用于定位 `start_node_id` |

### 返回值

```python
{
    "nodes": [
        {"id": int, "x": float, "y": float},
    ],
    "edges": [
        {
            "id": int,
            "from": int,         # 起始节点 id
            "to": int,           # 终止节点 id
            "points": [[float, float], ...],  # 点序列 (x, y) 世界米
            "length_m": float,   # 弧长（米）
        },
    ],
    "metadata": {
        "outer_perimeter_m": float,
        "num_nodes": int,
        "num_edges": int,
        "pixels_per_meter": float,
        "render_resolution": int,
        "actual_scale": float,   # 实际 px/m
        "smoothing_factor": float,
        "resample_spacing_m": float,
        "prune_spur_length_m": float,
    }
}
```

## 闭环参考轨迹组装

中心线图是拓扑分支结构，车道保持等应用需要将其拼接为单条闭环轨迹。
使用 `pipeline/sim_lane_keeping_real.py` 中的 `assemble_go_straight_circuit()`：

```python
from pipeline.sim_lane_keeping_real import assemble_go_straight_circuit

graph = extract_centerline_graph(...)
loop_pts = assemble_go_straight_circuit(graph)      # 返回 (N, 2) np.ndarray
loop_pts = loop_pts[::-1]                            # 可选反向跑
```

### 拼接规则

| 场景 | 策略 |
|------|------|
| **3 岔路口** (分叉/汇合) | 选最弯曲且 <90° 的候选边（入环岛） |
| **4+ 岔路口** (十字路口) | 选航向偏差最小的候选边（直行） |
| **2 岔路口** (普通连接) | 选航向偏差最小的候选边 |
| **环岛边** | 自动识别最短的 3↔3 度节点间边，允许正反向各自独立通行 |
| **起跑线** | 从 `metadata.start_node_id` 对应的边终端出发 |
| **无候选** | 自动终止，输出已拼接的完整闭环 |

### 工作原理

1. 为每条边计算 A/B 两个终端的位置和切向角
2. 将终端聚类为物理节点（KDTree），计算每个终端的岔路度数
3. 从起跑线出发，每步在所有未访问且物理相邻的候选边中选择
4. 非环岛边访问一次即双向屏蔽，环岛边可独立访问正反向
5. 拼接为连续坐标数组，自动衔接相邻边（去重首点）

## 验证测试

```bash
# 基础测试（path2.png，默认检测起跑线）
python pipeline/test_centerline.py

# 无起跑线赛道
python pipeline/test_centerline.py --image pipeline/map_parser/path1.jpg --no-has-starting-line

# 含可视化
python pipeline/test_centerline.py --visualize

# 保存输出
python pipeline/test_centerline.py --save output/test_graph.json --save-plot output/test_centerline.png

# 完整参数
python pipeline/test_centerline.py --image pipeline/map_parser/path2.png \
    --pixels-per-meter 12.8 --smoothing-factor 0.02 --has-starting-line \
    --max-starting-line-area 200 --visualize
```

## 依赖

```bash
pip install scikit-image scipy numpy opencv-python
```

## 与 map_parser 组合使用

```python
from map_parser import parse_map
from centerline import extract_centerline_graph

# 普通赛道：图像 → 拓扑图
boundaries = parse_map("track.jpg")
graph = extract_centerline_graph(
    boundaries["outer_boundary"],
    boundaries["holes"],
)

# 带起跑线的赛道
boundaries = parse_map("track.png", has_starting_line=True)
graph = extract_centerline_graph(
    boundaries["outer_boundary"],
    boundaries["holes"],
)
```
