"""
_core.py — 静态障碍物核心实现

Obstacle 数据类 + ObstacleLayer 集合 → 占用栅格注入 + 多边形输出。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

import numpy as np

# =====================================================================
#  数据类
# =====================================================================


@dataclass
class Obstacle:
    """单个静态障碍物。

    type : "rectangle" | "circle" | "polygon"
    params : 障碍物几何参数 (dict)
    dilate_margin : 额外膨胀距离 (m)
    """

    type: str
    params: dict
    dilate_margin: float = 0.0


# =====================================================================
#  几何工具函数
# =====================================================================

_CIRCLE_SEGMENTS = 32


def _polygon_scanline_intersections(
    y: float, poly: list[tuple[float, float]]
) -> list[float]:
    """扫描线与多边形所有边的水平交点 (x 值列表)."""
    xs: list[float] = []
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 <= y < y2) or (y2 <= y < y1):
            if abs(y2 - y1) > 1e-12:
                x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
                xs.append(x)
    xs.sort()
    return xs


def _obstacle_to_polygon(o: Obstacle) -> np.ndarray:
    """将 Obstacle 转换为世界坐标多边形顶点 (M, 2)."""
    if o.type == "rectangle":
        cx, cy = o.params["center"]
        w = o.params["width"]
        h = o.params["height"]
        yaw = o.params.get("yaw", 0.0)

        # 未旋转的四个角点 (以 center 为原点)
        corners = np.array([
            [-w / 2, -h / 2],
            [w / 2, -h / 2],
            [w / 2, h / 2],
            [-w / 2, h / 2],
        ], dtype=np.float64)

        if abs(yaw) > 1e-12:
            c = math.cos(yaw)
            s = math.sin(yaw)
            R = np.array([[c, -s], [s, c]], dtype=np.float64)
            corners = corners @ R.T

        corners[:, 0] += cx
        corners[:, 1] += cy
        return corners

    if o.type == "circle":
        cx, cy = o.params["center"]
        r = o.params["radius"]
        n_seg = o.params.get("segments", _CIRCLE_SEGMENTS)

        angles = np.linspace(0, 2 * math.pi, n_seg, endpoint=False)
        xs = cx + r * np.cos(angles)
        ys = cy + r * np.sin(angles)
        return np.column_stack([xs, ys])

    if o.type == "polygon":
        verts = np.array(o.params["vertices"], dtype=np.float64)
        if verts.shape[1] != 2:
            raise ValueError(
                f"polygon vertices 必须是 (N, 2)，当前 shape={verts.shape}"
            )
        return verts

    raise ValueError(
        f"未知的障碍物类型: '{o.type}'。可选: rectangle, circle, polygon"
    )


def _dilate_grid_manual(
    grid: list[list[int]], radius: int
) -> None:
    """手动膨胀 grid 中的障碍物 cell（Chebyshev 距离）。"""
    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0
    if rows == 0 or radius <= 0:
        return

    # 收集所有障碍物 cell
    obs_cells = [
        (r, c) for r in range(rows) for c in range(cols) if grid[r][c] == 1
    ]

    for r, c in obs_cells:
        for dr in range(-radius, radius + 1):
            nr = r + dr
            if nr < 0 or nr >= rows:
                continue
            for dc in range(-radius, radius + 1):
                nc = c + dc
                if nc < 0 or nc >= cols:
                    continue
                grid[nr][nc] = 1


# =====================================================================
#  ObstacleLayer
# =====================================================================


class ObstacleLayer:
    """静态障碍物集合。

    支持编程式添加 (add_rectangle / add_circle / add_polygon)
    和从 JSON 配置文件加载 (add_from_json)。

    典型用法:
        obs = ObstacleLayer()
        obs.add_rectangle(center=(15, 20), width=2.0, height=5.0)
        obs.add_from_json("config/obstacles.json")
        obs.apply_to_grid(grid, grid_meta)
    """

    def __init__(self) -> None:
        self.obstacles: list[Obstacle] = []

    # ---- 添加方法 ----

    def add_rectangle(
        self,
        center: tuple[float, float],
        width: float,
        height: float,
        yaw: float = 0.0,
        dilate_margin: float = 0.0,
    ) -> Obstacle:
        """添加矩形障碍物。

        center : (x, y) 世界坐标中心
        width / height : 矩形尺寸 (m)
        yaw : 绕中心旋转角 (rad)
        dilate_margin : 膨胀边距 (m)
        """
        if width <= 0 or height <= 0:
            raise ValueError(
                f"width 和 height 必须 > 0，当前 width={width}, height={height}"
            )
        o = Obstacle(
            type="rectangle",
            params={"center": tuple(center), "width": width,
                    "height": height, "yaw": yaw},
            dilate_margin=dilate_margin,
        )
        self.obstacles.append(o)
        return o

    def add_circle(
        self,
        center: tuple[float, float],
        radius: float,
        dilate_margin: float = 0.0,
        segments: int = _CIRCLE_SEGMENTS,
    ) -> Obstacle:
        """添加圆形障碍物。

        center : (x, y) 世界坐标圆心
        radius : 半径 (m)
        dilate_margin : 膨胀边距 (m)
        segments : 圆近似分段数
        """
        if radius <= 0:
            raise ValueError(f"radius 必须 > 0，当前 radius={radius}")
        if segments < 8:
            raise ValueError(f"segments 至少为 8，当前 segments={segments}")
        o = Obstacle(
            type="circle",
            params={"center": tuple(center), "radius": radius,
                    "segments": segments},
            dilate_margin=dilate_margin,
        )
        self.obstacles.append(o)
        return o

    def add_polygon(
        self,
        vertices: list[tuple[float, float]],
        dilate_margin: float = 0.0,
    ) -> Obstacle:
        """添加多边形障碍物。

        vertices : [(x, y), ...] 顶点列表，自动闭合
        dilate_margin : 膨胀边距 (m)
        """
        if len(vertices) < 3:
            raise ValueError(
                f"polygon 至少需要 3 个顶点，当前 {len(vertices)} 个"
            )
        o = Obstacle(
            type="polygon",
            params={"vertices": list(vertices)},
            dilate_margin=dilate_margin,
        )
        self.obstacles.append(o)
        return o

    def add_from_json(self, json_path: str) -> int:
        """从 JSON 配置文件批量加载障碍物。

        返回成功加载的障碍物数量。

        JSON 格式:
            {
              "obstacles": [
                {"type": "rectangle", "center": [x,y], "width": w, ...},
                {"type": "circle",    "center": [x,y], "radius": r, ...},
                {"type": "polygon",   "vertices": [[x,y], ...]}
              ]
            }
        """
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        if not isinstance(data, dict):
            raise ValueError(f"JSON 根节点必须是 dict，当前 {type(data).__name__}")

        entries = data.get("obstacles", [])
        if not isinstance(entries, list):
            raise ValueError(
                f'"obstacles" 必须是 list，当前 {type(entries).__name__}'
            )

        count = 0
        for entry in entries:
            if not isinstance(entry, dict):
                print(f"  [WARN] 跳过非 dict 条目: {entry}")
                continue

            o_type = entry.get("type", "")
            dilate = float(entry.get("dilate_margin", 0.0))

            if o_type == "rectangle":
                center = tuple(entry["center"])
                self.add_rectangle(
                    center=center,
                    width=float(entry["width"]),
                    height=float(entry["height"]),
                    yaw=float(entry.get("yaw", 0.0)),
                    dilate_margin=dilate,
                )
                count += 1

            elif o_type == "circle":
                center = tuple(entry["center"])
                self.add_circle(
                    center=center,
                    radius=float(entry["radius"]),
                    dilate_margin=dilate,
                    segments=int(entry.get("segments", _CIRCLE_SEGMENTS)),
                )
                count += 1

            elif o_type == "polygon":
                verts = entry["vertices"]
                self.add_polygon(
                    vertices=[tuple(v) for v in verts],
                    dilate_margin=dilate,
                )
                count += 1

            else:
                print(f"  [WARN] 未知障碍物类型 '{o_type}'，跳过")

        return count

    # ---- 栅格操作 ----

    def apply_to_grid(
        self,
        grid: list[list[int]],
        grid_meta: dict,
    ) -> None:
        """将障碍物标记到占用栅格（in-place 修改）。

        对每个障碍物：
        1. 转为世界坐标多边形
        2. 在 AABB 范围内用扫描线填充 grid cell = 1
        3. 若有 dilate_margin，做局部膨胀

        grid : list[list[int]] — 0=自由, 1=障碍物
        grid_meta : dict — {x_min, y_min, cell_size, rows, cols}
        """
        if not self.obstacles:
            return

        x_min = grid_meta["x_min"]
        y_min = grid_meta["y_min"]
        cell_size = grid_meta["cell_size"]
        rows = grid_meta["rows"]
        cols = grid_meta["cols"]

        for o in self.obstacles:
            poly = _obstacle_to_polygon(o)
            poly_list = [(float(p[0]), float(p[1])) for p in poly]

            # 世界 AABB
            ox_min = float(np.min(poly[:, 0]))
            ox_max = float(np.max(poly[:, 0]))
            oy_min = float(np.min(poly[:, 1]))
            oy_max = float(np.max(poly[:, 1]))

            # 扩展 AABB 以包含膨胀边距
            margin = o.dilate_margin
            ox_min -= margin
            ox_max += margin
            oy_min -= margin
            oy_max += margin

            # 栅格范围
            r_min = max(0, int((oy_min - y_min) / cell_size))
            r_max = min(rows - 1, int((oy_max - y_min) / cell_size))
            c_min = max(0, int((ox_min - x_min) / cell_size))
            c_max = min(cols - 1, int((ox_max - x_min) / cell_size))

            # 扫描线填充
            for r in range(r_min, r_max + 1):
                y = y_min + r * cell_size + cell_size / 2.0
                xs = _polygon_scanline_intersections(y, poly_list)
                for k in range(0, len(xs) - 1, 2):
                    xl = xs[k]
                    xr = xs[k + 1]
                    cl = int((xl - x_min) / cell_size)
                    cr = int((xr - x_min) / cell_size)
                    cl = max(0, cl)
                    cr = min(cols - 1, cr)
                    for c in range(cl, cr + 1):
                        grid[r][c] = 1

            # 二次膨胀
            if margin > 0:
                dilate_r = max(1, int(math.ceil(margin / cell_size)))
                # 只膨胀 AABB 范围内的新障碍物 cell
                for r in range(
                    max(0, r_min - dilate_r),
                    min(rows, r_max + dilate_r + 1),
                ):
                    for c in range(
                        max(0, c_min - dilate_r),
                        min(cols, c_max + dilate_r + 1),
                    ):
                        if grid[r][c] != 1:
                            continue
                        for dr in range(-dilate_r, dilate_r + 1):
                            nr = r + dr
                            if nr < 0 or nr >= rows:
                                continue
                            for dc in range(-dilate_r, dilate_r + 1):
                                nc = c + dc
                                if nc < 0 or nc >= cols:
                                    continue
                                grid[nr][nc] = 1

        # 打印统计
        obs_count = sum(sum(1 for c in row if c == 1) for row in grid)
        total = rows * cols
        print(
            f"  障碍物注入: {len(self.obstacles)} 个 → "
            f"grid {obs_count}/{total} occupied "
            f"({100.0 * obs_count / total:.1f}%)"
        )

    # ---- 多边形输出 ----

    def to_polygons(self) -> list[np.ndarray]:
        """将所有障碍物转为多边形列表，供 SafeCorridor 使用。

        返回 list of np.ndarray，每个 shape=(M,2)。
        """
        return [_obstacle_to_polygon(o) for o in self.obstacles]

    def __len__(self) -> int:
        return len(self.obstacles)

    def __repr__(self) -> str:
        type_counts: dict[str, int] = {}
        for o in self.obstacles:
            type_counts[o.type] = type_counts.get(o.type, 0) + 1
        parts = [f"{t}×{c}" for t, c in sorted(type_counts.items())]
        return f"ObstacleLayer({', '.join(parts) if parts else 'empty'})"
