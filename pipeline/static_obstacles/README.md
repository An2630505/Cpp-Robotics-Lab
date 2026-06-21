# static_obstacles — 静态障碍物模块

在赛道上自主定义静态障碍物（矩形、圆形、多边形），注入到占用栅格和 SafeCorridor 中。

## 快速开始

```python
from pipeline.static_obstacles import ObstacleLayer

obs = ObstacleLayer()

# 编程式添加
obs.add_rectangle(center=(15, 20), width=2.0, height=5.0, yaw=0.3)
obs.add_circle(center=(30, 15), radius=1.5)
obs.add_polygon(vertices=[(10, 10), (12, 10), (12, 13), (11, 14), (10, 13)])

# 从 JSON 配置文件加载
obs.add_from_json("config/obstacles.json")

# 应用到占用栅格 (0=自由, 1=障碍物)
obs.apply_to_grid(grid, grid_meta)

# 转为多边形列表 (供 SafeCorridor 使用)
extra_holes = obs.to_polygons()
```

## JSON 配置文件格式

```json
{
  "obstacles": [
    {
      "type": "rectangle",
      "center": [15.0, 20.0],
      "width": 2.0,
      "height": 5.0,
      "yaw": 0.3,
      "dilate_margin": 0.2
    },
    {
      "type": "circle",
      "center": [30.0, 15.0],
      "radius": 1.5,
      "dilate_margin": 0.1
    },
    {
      "type": "polygon",
      "vertices": [[10, 10], [12, 10], [12, 13], [11, 14], [10, 13]],
      "dilate_margin": 0.0
    }
  ]
}
```

## API 参考

### `Obstacle`

| 字段 | 类型 | 说明 |
|------|------|------|
| `type` | `str` | `"rectangle"`, `"circle"`, `"polygon"` |
| `params` | `dict` | 障碍物几何参数 |
| `dilate_margin` | `float` | 额外膨胀距离 (m)，默认 0.0 |

### `ObstacleLayer`

#### 添加方法

| 方法 | 参数 | 说明 |
|------|------|------|
| `add_rectangle(center, width, height, yaw=0.0, dilate_margin=0.0)` | center=(x,y), width, height (m), yaw (rad) | 添加矩形障碍物 |
| `add_circle(center, radius, dilate_margin=0.0, segments=32)` | center=(x,y), radius (m) | 添加圆形障碍物 |
| `add_polygon(vertices, dilate_margin=0.0)` | vertices=[(x,y),...] ≥3个 | 添加多边形障碍物 |
| `add_from_json(json_path)` → `int` | json_path: JSON 文件路径 | 批量加载，返回成功数量 |

#### 核心方法

| 方法 | 说明 |
|------|------|
| `apply_to_grid(grid, grid_meta)` | 将所有障碍物标记到占用栅格（1=障碍物），in-place 修改 |
| `to_polygons()` → `list[np.ndarray]` | 所有障碍物转为 `(M,2)` 多边形列表，供 SafeCorridor 使用 |

### `grid_meta` 格式

```python
{
    "x_min": float,   # 栅格世界坐标原点 X
    "y_min": float,   # 栅格世界坐标原点 Y
    "cols": int,      # 列数
    "rows": int,      # 行数
    "cell_size": float # 每格尺寸 (m)
}
```

## 验证测试

```bash
# 单元测试 (纯 Python，无 C++ 依赖)
python pipeline/test_static_obstacles.py

# 可视化测试
python pipeline/test_static_obstacles.py --visualize

# 集成测试 (需要编译 pnc 库)
./build_pnc.sh
python pipeline/sim_trajectory_optimization.py
```

## 依赖

- `numpy` — 多边形计算
- (可选) `matplotlib` — 可视化测试
