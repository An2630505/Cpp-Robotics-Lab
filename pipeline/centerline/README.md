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

## 验证测试

```bash
# 基础测试
python pipeline/test_centerline.py

# 含可视化
python pipeline/test_centerline.py --visualize

# 保存输出
python pipeline/test_centerline.py --save output/test_graph.json --save-plot output/test_centerline.png
```

## 依赖

```bash
pip install scikit-image scipy numpy opencv-python
```

## 与 map_parser 组合使用

```python
from map_parser import parse_map
from centerline import extract_centerline_graph

# 完整的"图像 → 拓扑图"流水线
boundaries = parse_map("track.jpg")
graph = extract_centerline_graph(
    boundaries["outer_boundary"],
    boundaries["holes"],
)
```
