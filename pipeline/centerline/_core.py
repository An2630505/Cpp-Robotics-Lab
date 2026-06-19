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
    starting_line: list[list[list[float]]] | None = None,
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
    starting_line : 起跑线列表，[[(x,y),...],...]。提供时会在其穿过中心线
                    处插入起始/终止节点，标记为 start_node。

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

    # ---- Step 5.5: 构建节点列表 ----
    node_list = [
        {"id": i, "x": round(float(x), 4), "y": round(float(y), 4)}
        for i, (x, y) in enumerate(nodes_world)
    ]

    # ---- Step 6: 插入起跑线节点 ----
    start_node_id: int | None = None
    if starting_line and smoothed_edges:
        # 起跑线中心点（所有条纹的质心）
        sl_all = np.concatenate([np.asarray(s, dtype=np.float64) for s in starting_line])
        sl_center = sl_all.mean(axis=0)  # (x, y)

        # 在所有边上找离起跑线最近的点
        best_dist = float("inf")
        best_edge_idx = -1
        best_pt_idx = -1
        for e_idx, e in enumerate(smoothed_edges):
            pts = np.array(e["points"])
            dists = np.sqrt(np.sum((pts - sl_center) ** 2, axis=1))
            idx = int(np.argmin(dists))
            if dists[idx] < best_dist:
                best_dist = dists[idx]
                best_edge_idx = e_idx
                best_pt_idx = idx

        if best_edge_idx >= 0 and best_dist < 20.0:  # 必须在合理范围内
            # 拆分该边，在最近点处插入新节点
            e_split = smoothed_edges[best_edge_idx]
            split_pts = np.array(e_split["points"])
            split_x, split_y = float(split_pts[best_pt_idx, 0]), float(split_pts[best_pt_idx, 1])

            # 创建新节点（起跑线节点）
            new_node_id = len(node_list)
            node_list.append({"id": new_node_id, "x": round(split_x, 4), "y": round(split_y, 4)})
            start_node_id = new_node_id

            # 拆成两条边：A = from → 起跑线, B = 起跑线 → to
            pts_a = split_pts[:best_pt_idx + 1]
            pts_b = split_pts[best_pt_idx:]

            if len(pts_a) >= 2:
                len_a = round(float(_arc_length(pts_a)), 3)
                if len_a > 0.5:
                    smoothed_edges.append({
                        "from": int(e_split["from"]),
                        "to": new_node_id,
                        "points": [[float(x), float(y)] for x, y in pts_a],
                        "length_m": len_a,
                    })
            if len(pts_b) >= 2:
                len_b = round(float(_arc_length(pts_b)), 3)
                if len_b > 0.5:
                    smoothed_edges.append({
                        "from": new_node_id,
                        "to": int(e_split["to"]),
                        "points": [[float(x), float(y)] for x, y in pts_b],
                        "length_m": len_b,
                    })

            # 移除原边
            del smoothed_edges[best_edge_idx]

            # 重新编号
            for i, e in enumerate(smoothed_edges):
                e["id"] = i

    # ---- Step 7: 组装输出 ----
    outer_perimeter = _arc_length(outer)

    result = {
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
    if start_node_id is not None:
        result["metadata"]["start_node_id"] = start_node_id
    return result


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
