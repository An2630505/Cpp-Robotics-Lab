"""
_core.py — 中心线图提取主管线

渲染掩码 → 骨架化 → 拓扑图 → 样条平滑 → JSON 输出。
"""

from __future__ import annotations

import numpy as np

from ._skeleton import extract_skeleton_graph
from ._smooth_open import smooth_open_curve


def _pixel_path_to_world(
    pixel_path: list[tuple[int, int]],
    actual_scale: float,
    offset_x: float,
    offset_y: float,
) -> np.ndarray:
    """将像素路径转换为世界坐标。offset = margin - min_* * scale。"""
    pts = np.array(pixel_path, dtype=np.float64)
    pts[:, 0] = (pts[:, 0] - offset_x) / actual_scale
    pts[:, 1] = (pts[:, 1] - offset_y) / actual_scale
    return pts


def extract_centerline_graph(
    outer_boundary: list[list[float]],
    holes: list[list[list[float]]],
    pixels_per_meter: float = 12.8,
    smoothing_factor: float = 0.02,
    resample_spacing_m: float | None = None,
    render_resolution: int = 1024,
    prune_spur_length_m: float = 2.0,
) -> dict:
    """
    从赛道边界提取中心线拓扑图。

    参数
    ----
    outer_boundary : 外边界，[(x,y), ...]
    holes : 孔洞列表，[[(x,y), ...], ...]
    pixels_per_meter : 参考缩放比例（默认 12.8）
    smoothing_factor : 样条平滑系数（默认 0.02）
    resample_spacing_m : 弧长重采样间距，米。None 时自动取 1/pixels_per_meter
    render_resolution : 渲染掩码最大边长（默认 1024）
    prune_spur_length_m : 骨架毛刺剪除阈值，米（默认 2.0）

    返回
    ----
    dict:
        {
            "nodes": [{"id": 0, "x": ..., "y": ...}, ...],
            "edges": [{"id": 0, "from": 0, "to": 1, "points": [...], "length_m": ...}, ...],
            "metadata": {...}
        }
    """
    outer = np.asarray(outer_boundary, dtype=np.float64)
    holes_arr = [np.asarray(h, dtype=np.float64) for h in holes]

    if len(outer) < 3:
        raise ValueError("outer_boundary 至少需要 3 个点")

    # ---- Step 1-4: 骨架图提取 ----
    nodes_world, edges_pixel, skeleton, actual_scale = extract_skeleton_graph(
        outer_boundary=outer,
        holes=holes_arr,
        pixels_per_meter=pixels_per_meter,
        render_resolution=render_resolution,
        prune_spur_length_m=prune_spur_length_m,
    )

    # 计算偏移量用于像素→世界转换
    min_x, max_x, min_y, max_y = _bbox(outer, holes_arr)
    max_extent = max(max_x - min_x, max_y - min_y)
    margin = 5
    canvas_sz = int(np.ceil(max_extent * actual_scale))
    canvas_sz_padded = canvas_sz + 2 * margin
    offset_x = -min_x * actual_scale + margin
    offset_y = -min_y * actual_scale + margin

    # ---- Step 5: 像素 → 世界 + 样条平滑 ----
    resample_sp = resample_spacing_m
    if resample_sp is None:
        resample_sp = 1.0 / pixels_per_meter

    smoothed_edges: list[dict] = []
    for i, edge in enumerate(edges_pixel):
        world_raw = _pixel_path_to_world(
            edge["pixel_path"], actual_scale, offset_x, offset_y
        )

        if len(world_raw) < 4:
            # 点太少，不用样条，直接降采样
            smoothed = world_raw
        else:
            # 所有边都用开曲线平滑——"loop" 是拓扑属性不是几何属性
            smoothed = smooth_open_curve(
                world_raw, smoothing_factor, resample_sp
            )

        length_m = float(_arc_length(smoothed))

        # 过滤过短的 artifact 边
        if length_m < 0.5 or len(smoothed) < 5:
            continue

        smoothed_edges.append({
            "id": i,
            "from": int(edge["from"]),
            "to": int(edge["to"]),
            "points": [[float(x), float(y)] for x, y in smoothed],
            "length_m": round(length_m, 3),
        })

    # ---- Step 6: 组装输出 ----
    node_list = [
        {"id": i, "x": round(float(x), 4), "y": round(float(y), 4)}
        for i, (x, y) in enumerate(nodes_world)
    ]

    outer_perimeter = _arc_length(outer)

    return {
        "nodes": node_list,
        "edges": smoothed_edges,
        "metadata": {
            "outer_perimeter_m": round(outer_perimeter, 1),
            "num_nodes": len(node_list),
            "num_edges": len(smoothed_edges),
            "pixels_per_meter": pixels_per_meter,
            "render_resolution": render_resolution,
            "actual_scale": round(actual_scale, 2),
            "smoothing_factor": smoothing_factor,
            "resample_spacing_m": round(resample_sp, 4),
            "prune_spur_length_m": prune_spur_length_m,
        },
    }


def _arc_length(pts: np.ndarray) -> float:
    """计算多段线弧长。"""
    if len(pts) < 2:
        return 0.0
    diffs = np.diff(pts, axis=0)
    return float(np.sum(np.sqrt(np.sum(diffs ** 2, axis=1))))


def _bbox(
    outer: np.ndarray, holes: list[np.ndarray]
) -> tuple[float, float, float, float]:
    all_pts = [outer]
    all_pts.extend(holes)
    xs = np.concatenate([p[:, 0] for p in all_pts])
    ys = np.concatenate([p[:, 1] for p in all_pts])
    return float(xs.min()), float(xs.max()), float(ys.min()), float(ys.max())
