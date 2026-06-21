# 静态障碍物与避障系统

## 概述

在赛道上放置静态障碍物（矩形、圆形、多边形），通过 Hybrid A\* + SafeCorridor + B-Spline + MPC 管线实现避障。障碍物通过 `config/obstacles.json` 配置文件定义，`pipeline/static_obstacles/` 模块加载并注入管线。

## 模块架构

```
config/obstacles.json          — 障碍物定义（位置、形状、尺寸）
       │
       ▼
pipeline/static_obstacles/     — 障碍物加载模块
  ├── _core.py                 — Obstacle, ObstacleLayer
  └── __init__.py              — 公开 API
       │
       ▼
pipeline/sim_static_obstacles.py  — 主管线脚本（8 步）
pipeline/sim_static_obstacles_animate.py — 动画回放
```

### ObstacleLayer API

```python
from pipeline.static_obstacles import ObstacleLayer

obs = ObstacleLayer()
obs.add_rectangle(center=(x, y), width=w, height=h, yaw=θ)
obs.add_circle(center=(x, y), radius=r)
obs.add_polygon(vertices=[(x1,y1), ...])
obs.add_from_json("config/obstacles.json")  # 从配置文件加载
obs.to_polygons()                            # → list of (M,2) numpy
obs.apply_to_grid(grid, grid_meta)           # 直接标记占用栅格
```

## 配置文件格式

```json
{
  "obstacles": [
    {
      "type": "rectangle",
      "center": [59.0, 47.0],
      "width": 1.5,
      "height": 1.0,
      "yaw": 0.4,
      "dilate_margin": 0.2
    },
    {
      "type": "circle",
      "center": [30.5, 57.0],
      "radius": 1.0,
      "dilate_margin": 0.2
    },
    {
      "type": "polygon",
      "vertices": [[10, 10], [12, 10], [12, 13], [11, 14], [10, 13]]
    }
  ]
}
```

每个障碍物可选 `dilate_margin` 字段做额外局部膨胀（默认 0）。

## 管线流程

```
[1] map_parser       — 赛道渲染图 → 外边界 + 孔洞（世界坐标多边形）
[2] centerline       — 边界多边形 → 中心线拓扑图（节点 + 边）
[3] circuit assembly — 拓扑图 → 闭环回路点列（等弧长采样）
[4] load_obstacles   — 从 JSON 加载障碍物 → ObstacleLayer
[5] occupancy grid   — 外边界 + 孔洞 + 障碍物多边形 → 扫描线填充 → 占用栅格
                        障碍物多边形合并到 grid_holes：
                        outer 边界填充 1 → holes 挖空为 0 → 翻转 0↔1
                        → dilate_grid 膨胀 SAFETY_MARGIN (1.2m)
[6] HA* 规划         — 沿中心线生成 Gate（15m 间距），逐 Gate 运动学规划
                        碰撞检测基于栅格：车辆 bounding box 与障碍物 cell 重叠即碰撞
                        Dijkstra 启发式：障碍物 cell 不可达
[7] SafeCorridor     — 基于栅格的矩形扩张法，沿 HA* 路径构建安全走廊
                        B-Spline 在走廊内最小二乘拟合 → 等弧长重采样
[8] MPC 仿真         — 误差动力学模型 + 卡尔曼滤波 + 曲率前馈控制
```

## 三层安全边距

传统做法是同一个 margin 值同时用于栅格膨胀和走廊减法，两者互相抵消。本设计拆成三个独立参数：

| 参数 | 值 | 用途 |
|------|-----|------|
| `SAFETY_MARGIN` | VEHICLE_HW + 0.2 = 1.2m | 栅格膨胀 — dilate_grid 半径 |
| `COLLISION_MARGIN` | VEHICLE_HW + 0.2 = 1.2m | HA\* 碰撞盒半宽, 也传给前/后延伸; 走廊矩形半宽 |
| `CORRIDOR_MARGIN` | COLLISION_MARGIN = 1.2m | 走廊 expandRect 后的减边值, 让间隙肉眼可见 |

```
真实障碍物
  ← SAFETY_MARGIN (栅格膨胀) →
        dilated cell（HA* 和走廊看到的"障碍物边界"）
  ← COLLISION_MARGIN →
        HA* 车盒边缘
  ← CORRIDOR_MARGIN →
        走廊边界（B-Spline 约束）
```

三个值各自独立：膨胀让 HA\* 绕远，碰撞盒匹配车辆真实尺寸，走廊缩进给平滑留余量。

## HA\* Grid Origin 修复

### 问题

`build_occupancy_grid` 为栅格添加 padding 后 `x_min`/`y_min` 为负值。HA\* 的 `collides()` 和 Dijkstra 启发式中直接用 `x / cell_size` 作为栅格索引，缺少 `x_min`/`y_min` 偏移。

```cpp
// HA* (错误) — 缺少原点偏移
int c = x / cell_size;

// 正确做法
int c = (x - x_min) / cell_size;
```

偏移量约 1.5m。对于宽路面该偏移影响不大，但对于小障碍物（0.5m~1.2m），HA\* 在错误位置读取栅格，障碍物被"移动"了 1.5m，导致路径穿过真实障碍物。

### 修复

在 `HybridAStar` 类中添加 `x_min_`/`y_min_` 成员和 `setGridOrigin()` 方法。`collides()`、`plan()`、`planToGate()` 中所有坐标转换统一加上原点偏移。

Python 侧调用：`ha.set_grid_origin(grid_meta["x_min"], grid_meta["y_min"])`

## SafeCorridor 方案演进

### v1 — 射线-多边形求交（原始方案）

每个采样点沿法向发射线，与 outer/holes 多边形求交点。
- **问题**：相邻采样点之间的扇形盲区可容纳整个小障碍物。hole 多边形传入后，走廊在障碍物处被大幅裁剪（曾在测试中从 6m 骤降至 0.36m），B 样条无法通过。

### v2 — 栅格射线

底层从多边形求交换成栅格查 cell，但仍是每条法向一条线。盲区未消除。

### v3 — 矩形扩张（当前方案）

每个采样点向法向扩张矩形，检查**矩形内全部 cell**：

1. 计算矩形 4 角点 → 栅格包围盒
2. 遍历包围盒内每个 cell
3. 点积判定是否在矩形内
4. 任一 cell 为 occupied → 停止扩张，返回上一层的距离
5. 减去 CORRIDOR_MARGIN → 记录 CorridorSection

```
 step 3:  ┌──────────────────┐
 step 2:  │  ┌──────────────┐│
 step 1:  │  │  ┌──────────┐││
          │  │  │  车辆    │││  ← 矩形宽 = 2 × COLLISION_MARGIN
          │  │  └──────────┘││
          │  └──────────────┘│
          └──────────────────┘
```

SafeCorridor 直接使用 HA\* 同一个栅格作为输入。`expandRect` 用 `COLLISION_MARGIN` 作为矩形半宽（与 HA\* 碰撞盒一致），扩张到 occupied cell 后停止，再减去 `CORRIDOR_MARGIN`（= COLLISION_MARGIN = 1.2m）作为走廊边界。走廊到真实路边的间隙 = 1.2m（膨胀）+ 1.2m（走廊减法）= 2.4m，肉眼可见。

## 讨论中尝试过但未采用的方案

### 延伸-截断（extension-truncation）

在 HA\* 路径两端延伸 5m 让 B 样条端点平滑，拟合后再截断回原始起止点。
- **问题**：闭合赛道中 gate0 ≈ gate29，路径回绕时 `best_start > best_end` 导致切片为空（崩溃）。障碍物改变路径形状后问题加剧。

### 障碍物多边形传入 SafeCorridor

将障碍物多边形和原始孔洞合并后传给 SafeCorridor，让走廊射线裁剪在障碍物处停止。
- **问题**：小障碍物多边形（0.3m~1m）在 SafeCorridor 的射线求交中造成走廊极度收窄（0.36m），B 样条无法通过，产生振荡（min seg 0.086m, max κ 0.3）。

### 障碍物多边形膨胀后传入 SafeCorridor

给障碍物多边形 centroid 膨胀 0.5m 后再传给 SafeCorridor。
- **问题**：走廊虽有所改善，但仍在障碍物附近收窄到 2.85m。小障碍物在宽赛道（6m）上不影响通行，但窄赛道上仍是瓶颈。

### 最终方案：栅格统一

障碍物多边形合并到 hole 列表 → 只传给 `build_occupancy_grid`（栅格膨胀统一处理）。SafeCorridor 直接使用栅格（v3 矩形扩张），不再接收多边形。HA\* 和 SafeCorridor 看到同一个栅格的同一份障碍物数据。B 样条控制点从 50 增至 100 以确保紧贴 HA\* 路径。

## 车辆参数

| 参数 | 值 | 说明 |
|------|-----|------|
| VEHICLE_HW | 1.0m | 车半宽 → 全宽 2.0m |
| VEHICLE_FWD | 1.5m | 后轴到车头 |
| VEHICLE_REV | 1.0m | 后轴到车尾 → 总长 2.5m |

碰撞盒和动画中的车辆视觉尺寸严格一致，均以后轴为中心。HA\* 碰撞检测和走廊矩形均使用膨胀后的尺寸（COLLISION_MARGIN 代替 VEHICLE_HW，前/后延伸各 +0.2m），确保路径和走廊都留有半个车宽+0.2 的安全冗余。

## 可视化要素

### 赛道与障碍物
- 黑色实线 — 真实道路边界（外边界）
- 白色填充 — 孔洞（地图自带岛屿）
- 红色填充 — 静态障碍物（原始尺寸）

### 轨迹
- 绿色虚线 — HA\* 原始路径
- 蓝色实线 — B-Spline 平滑路径
- 红色实线 + 箭头 — 车辆实际轨迹（MPC 跟踪结果）
- 灰色虚线 — 碰撞边界线（参考轨迹 ± COLLISION_MARGIN）

### 安全走廊
- 青色半透明填充 + 青色边界线 + 横截面线 — 走廊区域，用 outer 边界 clip_path 裁剪确保不超出道路
- 绘制顺序：outer 边界 → 走廊（clip） → 孔洞（白底覆盖） → 障碍物（红底覆盖） → 轨迹

### 车辆
- 视觉车身尺寸与碰撞盒严格一致：宽 2.0m × 长 2.5m，以后轴为中心
- 前轮位于后轴前方 `FWD × 0.7 = 1.05m`，后轮位于后轴后方 `REV × 0.5 = 0.5m`，均在车身内

## 运行

```bash
# 编译 C++ 模块
cd build && cmake --build .

# 运行仿真（需 conda 环境 CRL, Python 3.11）
python pipeline/sim_static_obstacles.py

# 动画回放
python pipeline/sim_static_obstacles_animate.py

# 保存 GIF
python pipeline/sim_static_obstacles_animate.py --save output/obs.gif
```

## 赛道对比

| | path2.png | path1.jpg |
|------|------|------|
| 图像尺寸 | 1109×1110 px | 1280×1280 px |
| 中心线长度 | 447m | 441m |
| 平均路宽 | ~13m | ~15m |
| 孔洞数 | 3 | 2 |
| Gate 数 | 29 | 29 |
| B-Spline 控制点 | 100 | 100 |

两个赛道尺寸接近，可通过修改 `sim_static_obstacles.py` 中的 `img` 路径切换。注意 path1 为 `.jpg` 格式。

## 文件索引

| 文件 | 说明 |
|------|------|
| `config/obstacles.json` | 障碍物配置文件 |
| `pipeline/static_obstacles/__init__.py` | 公开 API：Obstacle, ObstacleLayer |
| `pipeline/static_obstacles/_core.py` | 核心实现（几何转换、栅格注入） |
| `pipeline/static_obstacles/README.md` | 模块使用文档 |
| `pipeline/test_static_obstacles.py` | 模块单元测试（27 项） |
| `pipeline/sim_static_obstacles.py` | 主仿真脚本（8 步管线） |
| `pipeline/sim_static_obstacles_animate.py` | 动画回放脚本 |
| `pnc/motion/hybrid_astar/` | HA\* 规划（含 grid origin 修复） |
| `pnc/motion/safe_corridor/` | SafeCorridor 矩形扩张（v3） |
| `pnc/motion/bspline/` | B-Spline 拟合与重采样 |
