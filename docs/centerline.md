# Centerline Graph — 赛道中心线拓扑图提取

基于 `map_parser` 输出的赛道边界，用栅格骨架法提取赛道中心线的拓扑图（节点 + 边）。

## 设计决策

| 维度 | 决策 | 理由 |
|------|------|------|
| 输入 | `map_parser` 的输出（outer + holes） | 直接消费上游边界数据 |
| 中心线语义 | 道路中心线（非多车道） | 一条边覆盖整条道路宽度 |
| 图结构 | 节点-边拓扑图 | 节点仅在分叉/汇合点 |
| 方向性 | 无向边，仅 `x,y` 坐标 | 纯拓扑描述，方向留给下游 |
| 算法路线 | 栅格骨架法 | 成熟稳健，`skimage.skeletonize` 现成可用 |
| 平滑方式 | cubic open spline (`splprep per=0 k=3`) | 边是开曲线，首尾连接不同节点 |
| 输出格式 | 单个 JSON 文件 | 与 map_parser 风格一致 |
| 代码组织 | 独立 `pipeline/centerline/` 包 | 职责分离，各自独立演化 |

## 处理管线

```
渲染掩码 → 骨架化(skeletonize) → 交汇点检测 → 毛刺剪除 → 节点聚类合并 → 边追踪 → 像素→世界 → 样条平滑 → JSON
```

### 为什么选栅格骨架法

像素骨架法（vs Voronoi 中轴法）：
- 实现复杂度低，`skimage.morphology.skeletonize` 现成可用
- 精度足够（~8cm @12.8px/m），经样条平滑后可进一步改善
- 骨架天然拥有图结构（交汇像素 = 节点，骨架线段 = 边）

### 交汇点检测与节点合并

用 3×3 卷积计算每个骨架像素的度（邻居数）：
- **度 = 2**：普通骨架点
- **度 = 1**：端点（会被毛刺剪除）
- **度 ≥ 3**：交汇点（分叉/汇合）

相邻的交汇像素先用 KDTree 聚类（半径 5px），再对过近的簇（<20px）做并查集合并，确保一个拓扑节点对应唯一坐标。

### 岛旁 U 形环回处理

当赛道中有岛屿时，骨架会形成一个 U 形——两端连接到同一节点的同一个连通分量。此时通过距离变换找到距离岛屿最近的"收窄点"（constriction），在该点将 U 形分割为两条边。

## 输出结构

```json
{
  "nodes": [
    {"id": 0, "x": 43.7, "y": 77.2}
  ],
  "edges": [
    {
      "id": 0,
      "from": 0,
      "to": 0,
      "points": [[x1, y1], [x2, y2], ...],
      "length_m": 172.1
    }
  ],
  "metadata": {
    "outer_perimeter_m": 464.0,
    "num_nodes": 1,
    "num_edges": 4,
    "pixels_per_meter": 12.8,
    "render_resolution": 1024,
    "smoothing_factor": 0.02
  }
}
```

## API

```python
from map_parser import parse_map
from centerline import extract_centerline_graph

boundaries = parse_map("track.jpg")
graph = extract_centerline_graph(
    outer_boundary=boundaries["outer_boundary"],
    holes=boundaries["holes"],
    pixels_per_meter=12.8,         # 渲染分辨率比例
    smoothing_factor=0.02,         # 样条平滑系数
    resample_spacing_m=None,       # 弧长重采样间距
    render_resolution=1024,        # 渲染画布最大边长
    prune_spur_length_m=2.0,       # 毛刺剪除阈值（米）
)
```

## 依赖

- `scikit-image` — 骨架化 (`skimage.morphology.skeletonize`)
- `scipy` — 样条平滑、KDTree、距离变换
- `numpy` — 数组运算
- `opencv-python` — 掩码渲染 (`fillPoly`)、连通分量分析

## 边界情况

| 情况 | 策略 |
|------|------|
| 无孔洞（纯闭合环） | 引入 1 个虚拟节点，在骨架最左端将环断开为 1 条边 |
| 多个独立赛道 | 分别提取各自的图 |
| 骨架毛刺 | 小于 `prune_spur_length_m` 的分支自动剪除 |
| 渲染分辨率不足致骨架断裂 | 提高 `render_resolution` 或报错 |
| 边点数过少 (< 5) | `_core.py` 中自动过滤 |

---

> 实现于 `pipeline/centerline/` ，作为独立 Python 包。与 `map_parser` 组合使用。
