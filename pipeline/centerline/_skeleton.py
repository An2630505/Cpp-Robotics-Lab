"""
_skeleton.py — 栅格骨架化 → 拓扑图提取

将赛道边界渲染为二值掩码，提取骨架，
检测交汇点，追踪边，构建拓扑图结构。
"""

from __future__ import annotations

import cv2
import numpy as np
from scipy.ndimage import convolve
from scipy.spatial import KDTree

try:
    from skimage.morphology import skeletonize as _skel
    _HAS_SKIMAGE = True
except ImportError:
    _HAS_SKIMAGE = False


def _poly_to_mask(pts: np.ndarray, sz: int, scale: float) -> np.ndarray:
    """将世界坐标多边形渲染为掩码。"""
    pts_px = (pts * scale).astype(np.int32)
    mask = np.zeros((sz, sz), dtype=np.uint8)
    cv2.fillPoly(mask, [pts_px], 255)
    return mask


def _world_bbox(
    outer: np.ndarray, holes: list[np.ndarray]
) -> tuple[float, float, float, float]:
    """计算所有边界的包围盒 (min_x, max_x, min_y, max_y) 世界坐标。"""
    all_pts = [outer]
    all_pts.extend(holes)
    xs = np.concatenate([p[:, 0] for p in all_pts])
    ys = np.concatenate([p[:, 1] for p in all_pts])
    return float(xs.min()), float(xs.max()), float(ys.min()), float(ys.max())


def _neighbor_counts(skeleton: np.ndarray) -> np.ndarray:
    """计算骨架每个像素的邻居数。0 = 背景，1+self = degree。"""
    kernel = np.ones((3, 3), dtype=np.uint8)
    # convolve 得到 3×3 邻域的和，包含自身像素
    counts = convolve(skeleton.astype(np.uint8), kernel, mode="constant", cval=0)
    counts[~skeleton] = 0
    return counts


def _find_junction_pixels(
    neighbor_counts: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    寻找骨架特征像素。

    返回 (junction_pts, endpoint_pts, normal_pts)
    每个为 (N, 2) [x, y] 像素坐标。
    """
    skel = neighbor_counts > 0
    degree = neighbor_counts.copy()
    degree[~skel] = 0
    # degree 计数包含自身(1) + 邻居
    # endpoint: degree == 2 (自身 + 1 邻居)
    # normal: degree == 3 (自身 + 2 邻居)
    # junction: degree >= 4 (自身 + >=3 邻居)

    jy, jx = np.where(degree >= 4)
    ey, ex = np.where(degree == 2)

    junctions = np.column_stack([jx, jy]) if len(jx) > 0 else np.empty((0, 2), dtype=int)
    endpoints = np.column_stack([ex, ey]) if len(ex) > 0 else np.empty((0, 2), dtype=int)

    return junctions, endpoints


def _cluster_junctions(
    junctions: np.ndarray, radius: int = 5
) -> tuple[list[np.ndarray], np.ndarray]:
    """
    将相邻的交汇像素聚类为拓扑节点。

    返回 (clusters, node_coords) 其中 node_coords 为 (M, 2) 质心像素坐标。
    """
    if len(junctions) == 0:
        return [], np.empty((0, 2))

    tree = KDTree(junctions.astype(float))
    visited: set[int] = set()
    clusters: list[np.ndarray] = []

    for i in range(len(junctions)):
        if i in visited:
            continue
        nearby = tree.query_ball_point(junctions[i].astype(float), radius)
        visited.update(nearby)
        clusters.append(junctions[list(nearby)])

    node_coords = np.array([c.mean(axis=0) for c in clusters])
    return clusters, node_coords


def _merge_close_nodes(
    clusters: list[np.ndarray],
    node_coords: np.ndarray,
    threshold_px: float = 20.0,
) -> tuple[list[np.ndarray], np.ndarray]:
    """
    合并距离 < threshold_px 的节点簇。

    返回 (merged_clusters, merged_node_coords)。
    """
    if len(node_coords) <= 1:
        return clusters, node_coords

    n = len(node_coords)
    # Union-find
    parent = list(range(n))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for i in range(n):
        for j in range(i + 1, n):
            dist = np.linalg.norm(node_coords[i] - node_coords[j])
            if dist < threshold_px:
                union(i, j)

    # 按根分组
    groups: dict[int, list[int]] = {}
    for i in range(n):
        root = find(i)
        groups.setdefault(root, []).append(i)

    if len(groups) == n:
        return clusters, node_coords  # 无合并

    merged_clusters: list[np.ndarray] = []
    merged_coords_list: list[np.ndarray] = []

    for indices in groups.values():
        combined = np.vstack([clusters[i] for i in indices])
        merged_clusters.append(combined)
        merged_coords_list.append(combined.mean(axis=0))

    return merged_clusters, np.array(merged_coords_list)


def _prune_spurs(
    skeleton: np.ndarray,
    neighbor_counts: np.ndarray,
    min_length_px: int,
) -> tuple[np.ndarray, np.ndarray, int]:
    """
    剪除骨架毛刺（从端点走到第一个交汇点的短分支）。

    返回 (cleaned_skeleton, cleaned_counts, pruned_count)。
    """
    skel = skeleton.copy()
    counts = neighbor_counts.copy()
    pruned = 0

    for _ in range(10):  # 最多 10 轮迭代
        ey, ex = np.where((counts == 2) & (skel > 0))
        if len(ex) == 0:
            break

        changed = False
        for x, y in zip(ex, ey):
            # 从该端点沿骨架走
            path = _walk_skeleton(skel, x, y)
            if path is None:
                continue
            if len(path) >= min_length_px:
                continue
            # 如果路径末端是交汇点（非骨架自然终点），删除
            ex_last, ey_last = path[-1]
            if counts[ey_last, ex_last] < 4:
                continue
            # 删除此毛刺
            for px, py in path[:-1]:  # 保留末端的交汇点
                skel[py, px] = False
                counts[py, px] = 0
            pruned += 1
            changed = True

        if not changed:
            break

    return skel, counts, pruned


def _walk_skeleton(
    skeleton: np.ndarray, start_x: int, start_y: int
) -> list[tuple[int, int]] | None:
    """
    从骨架上的点沿单一路径走，直到交汇点或端点。

    返回路径像素列表 [(x, y), ...]，包含起点。
    """
    h, w = skeleton.shape
    path: list[tuple[int, int]] = [(start_x, start_y)]
    visited: set[tuple[int, int]] = {(start_x, start_y)}

    cx, cy = start_x, start_y

    while True:
        # 找邻居
        neighbors: list[tuple[int, int]] = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < w and 0 <= ny < h:
                    if skeleton[ny, nx] and (nx, ny) not in visited:
                        neighbors.append((nx, ny))

        if len(neighbors) == 0:
            return path  # 到达端点
        if len(neighbors) > 1:
            return path  # 到达交汇点

        nx, ny = neighbors[0]
        visited.add((nx, ny))
        path.append((nx, ny))
        cx, cy = nx, ny


def _assign_endpoint_to_node(
    endpoint_px: tuple[int, int],
    skeleton: np.ndarray,
    junction_mask: np.ndarray,
    junction_clusters: list[np.ndarray],
    node_coords: np.ndarray,
    dilated: np.ndarray,
) -> int:
    """
    将一个 CC 端点分配到最可能的节点。

    端点位于被移除的 junction 膨胀区域边缘。
    查找该端点附近最近的 junction 像素 → 确定属于哪个簇。
    回退为到 node_coords 的最近距离。
    """
    if len(node_coords) == 0:
        return 0

    ex, ey = endpoint_px
    h, w = skeleton.shape

    # 在端点周围 10 像素内找 junction 像素
    r = 10
    x1, x2 = max(0, ex - r), min(w - 1, ex + r)
    y1, y2 = max(0, ey - r), min(h - 1, ey + r)

    local_junc = np.argwhere(junction_mask[y1:y2 + 1, x1:x2 + 1])
    if len(local_junc) > 0:
        # 找到了附近 junction 像素 → 确定属于哪个簇
        junc_px = local_junc[0] + np.array([y1, x1])
        junc_pt = np.array([junc_px[1], junc_px[0]], dtype=float)  # (x, y)
        # 找包含此像素的簇
        for ci, cluster in enumerate(junction_clusters):
            if np.any((cluster[:, 0] == junc_pt[0]) & (cluster[:, 1] == junc_pt[1])):
                return ci
        # 不在任何簇中 → 用最近簇
        return int(np.argmin(np.sum((node_coords - junc_pt) ** 2, axis=1)))

    # 回退：找最近节点
    pt = np.array([ex, ey], dtype=float)
    return int(np.argmin(np.sum((node_coords - pt) ** 2, axis=1)))


def _trace_edges(
    skeleton: np.ndarray,
    neighbor_counts: np.ndarray,
    node_coords: np.ndarray,
    junction_clusters: list[np.ndarray] | None = None,
) -> list[dict]:
    """
    从骨架中提取边：移除交汇区域，找骨架连通分量。

    对两端连到同一节点的长路径（岛旁环回），在中点分割为两条边。

    返回边列表：[{"from": i, "to": j, "pixel_path": [(x,y),...]}, ...]
    """
    h, w = skeleton.shape
    skel = skeleton.copy()
    junction_mask = neighbor_counts >= 4

    if junction_mask.any():
        kernel = np.ones((7, 7), dtype=np.uint8)
        dilated = cv2.dilate(junction_mask.astype(np.uint8), kernel).astype(bool)
    else:
        dilated = np.zeros_like(junction_mask)

    skel[dilated] = False

    num_labels, labels = cv2.connectedComponents(skel.astype(np.uint8), connectivity=8)

    if junction_clusters is None:
        junction_clusters = []

    edges: list[dict] = []
    for label in range(1, num_labels):
        comp_mask = labels == label
        if np.count_nonzero(comp_mask) < 2:
            continue

        yx = np.argwhere(comp_mask)
        pixel_path = _trace_component(comp_mask, yx)

        if pixel_path is None or len(pixel_path) < 2:
            pts = np.column_stack(np.where(comp_mask))[:, ::-1]
            if len(pts) < 2:
                continue
            pixel_path = _sort_pixels(pts)

        if pixel_path is None or len(pixel_path) < 2:
            continue

        if len(node_coords) == 0:
            edges.append({
                "from": 0, "to": 0, "pixel_path": pixel_path,
            })
            continue

        # 通过端点附近的 junction 像素确定节点归属
        from_i = _assign_endpoint_to_node(
            pixel_path[0], skeleton, junction_mask,
            junction_clusters, node_coords, dilated,
        )
        to_i = _assign_endpoint_to_node(
            pixel_path[-1], skeleton, junction_mask,
            junction_clusters, node_coords, dilated,
        )

        # 两端连到同一节点且路径够长 → 岛旁 U 形环回 → 在中点分割
        if from_i == to_i and len(pixel_path) > 50:
            mid = len(pixel_path) // 2
            edge_a = pixel_path[:mid + 1]
            edge_b = pixel_path[mid:]
            edge_b.reverse()

            edges.append({
                "from": from_i, "to": from_i,
                "pixel_path": edge_a, "_split": True,
            })
            edges.append({
                "from": from_i, "to": from_i,
                "pixel_path": edge_b, "_split": True,
            })
            continue

        edges.append({
            "from": from_i,
            "to": to_i,
            "pixel_path": pixel_path,
        })

    return edges


def _trace_component(
    mask: np.ndarray, all_yx: np.ndarray,
) -> list[tuple[int, int]] | None:
    """从一个连通分量追踪出有序路径。"""
    # 找端点（在 mask 中只有 1 个邻居的点）
    endpoints: list[tuple[int, int]] = []
    for y, x in all_yx:
        n_count = 0
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                ny, nx = y + dy, x + dx
                if 0 <= ny < mask.shape[0] and 0 <= nx < mask.shape[1]:
                    if mask[ny, nx]:
                        n_count += 1
        if n_count == 1:
            endpoints.append((x, y))

    if len(endpoints) < 2:
        return None

    # 从第一个端点 BFS 到第二个
    start = endpoints[0]
    goal = endpoints[1]
    return _bfs_path(mask, start, goal)


def _bfs_path(
    mask: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> list[tuple[int, int]] | None:
    """BFS 在骨架掩码上找最短路径。"""
    from collections import deque
    h, w = mask.shape
    q = deque([start])
    parent: dict[tuple[int, int], tuple[int, int] | None] = {start: None}

    while q:
        curr = q.popleft()
        if curr == goal:
            # 重建路径
            path: list[tuple[int, int]] = []
            node: tuple[int, int] | None = curr
            while node is not None:
                path.append(node)
                node = parent[node]
            path.reverse()
            return path

        cx, cy = curr
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < w and 0 <= ny < h:
                    if mask[ny, nx] and (nx, ny) not in parent:
                        parent[(nx, ny)] = curr
                        q.append((nx, ny))

    return None


def _sort_pixels(pts: np.ndarray) -> list[tuple[int, int]] | None:
    """将无序像素集排序成路径（贪婪最近邻）。"""
    if len(pts) < 2:
        return None

    remaining = list(range(len(pts)))
    ordered: list[int] = [remaining.pop(0)]

    while remaining:
        last = pts[ordered[-1]]
        # 找最近的未访问点
        dists = np.sum((pts[remaining] - last) ** 2, axis=1)
        nearest_idx = int(np.argmin(dists))
        ordered.append(remaining.pop(nearest_idx))

    return [(int(pts[i][0]), int(pts[i][1])) for i in ordered]


def extract_skeleton_graph(
    outer_boundary: np.ndarray,   # (N, 2) 世界坐标
    holes: list[np.ndarray],      # 各为 (Mi, 2) 世界坐标
    pixels_per_meter: float = 12.8,
    render_resolution: int = 1024,
    prune_spur_length_m: float = 2.0,
) -> tuple[list[tuple[float, float]], list[dict], np.ndarray, float]:
    """
    从赛道边界提取骨架图。

    参数
    ----
    outer_boundary : (N, 2) 外边界世界坐标
    holes : list of (Mi, 2) 孔洞世界坐标
    pixels_per_meter : 参考缩放比例
    render_resolution : 渲染画布最大边长
    prune_spur_length_m : 毛刺剪除长度阈值（米）

    返回
    ----
    (nodes_world, edges_pixel, skeleton_mask, actual_scale)
        nodes_world: [(x_m, y_m), ...] 世界坐标节点列表
        edges_pixel: [{"from": i, "to": j, "pixel_path": [...]}, ...]
        skeleton_mask: 骨架二值图
        actual_scale: 实际 px/m
    """
    if not _HAS_SKIMAGE:
        raise ImportError(
            "centerline._skeleton 需要 scikit-image。请执行: pip install scikit-image"
        )

    # 计算包围盒，决定画布尺寸
    min_x, max_x, min_y, max_y = _world_bbox(outer_boundary, holes)
    bbox_w = max_x - min_x
    bbox_h = max_y - min_y
    max_extent = max(bbox_w, bbox_h)
    if max_extent < 1e-12:
        raise ValueError("赛道边界范围过小")

    # 实际渲染参数
    actual_scale = render_resolution / max_extent
    canvas_sz = int(np.ceil(max_extent * actual_scale))
    # 偏移（将世界原点映射到画布边距）
    margin = 5
    canvas_sz_padded = canvas_sz + 2 * margin
    offset_x = -min_x * actual_scale + margin
    offset_y = -min_y * actual_scale + margin

    def world_to_canvas(pts: np.ndarray) -> np.ndarray:
        p = pts.copy()
        p[:, 0] = p[:, 0] * actual_scale + offset_x
        p[:, 1] = p[:, 1] * actual_scale + offset_y
        return p

    # Step 1: 渲染掩码
    outer_px = world_to_canvas(outer_boundary).astype(np.int32)
    track_mask = np.zeros((canvas_sz_padded, canvas_sz_padded), dtype=np.uint8)
    cv2.fillPoly(track_mask, [outer_px], 255)

    for hole in holes:
        hole_px = world_to_canvas(hole).astype(np.int32)
        cv2.fillPoly(track_mask, [hole_px], 0)

    # Step 2: 骨架化
    skeleton = _skel(track_mask.astype(bool))

    # Step 3: 交汇点检测
    nbrs = _neighbor_counts(skeleton)
    junctions, endpoints = _find_junction_pixels(nbrs)

    # 步骤 3b: 毛刺剪除
    prune_len_px = int(prune_spur_length_m * actual_scale)
    skeleton, nbrs, pruned = _prune_spurs(skeleton, nbrs, prune_len_px)

    # 重新检测交汇点和端点
    junctions, endpoints = _find_junction_pixels(nbrs)

    # Step 4: 交汇点聚类
    junction_clusters, node_coords = _cluster_junctions(junctions, radius=5)

    # 合并过近的节点簇（间距 < 20 像素 → 视为同一拓扑节点）
    if len(node_coords) >= 2:
        merged_clusters, merged_coords = _merge_close_nodes(
            junction_clusters, node_coords, threshold_px=20,
        )
        junction_clusters = merged_clusters
        node_coords = merged_coords

    # 如果没有节点：纯闭合环，在骨架最左端引入一个虚拟节点
    no_junctions = len(node_coords) == 0
    if no_junctions:
        # 找骨架最左端像素作为虚拟节点
        skel_pts = np.column_stack(np.where(skeleton))
        if len(skel_pts) > 0:
            # 按 x 排序（第二列），取最左
            leftmost_yx = skel_pts[np.argmin(skel_pts[:, 1])]
            virtual_x = float(leftmost_yx[1])
            virtual_y = float(leftmost_yx[0])
            node_coords = np.array([[virtual_x, virtual_y]], dtype=float)
        else:
            # 完全没骨架——返回空
            return [], [], skeleton, actual_scale

    # Step 5: 追踪边
    edges_pixel = _trace_edges(skeleton, nbrs, node_coords, junction_clusters)

    # 节点坐标转世界
    nodes_world: list[tuple[float, float]] = []
    for nc in node_coords:
        wx = (nc[0] - offset_x) / actual_scale
        wy = (nc[1] - offset_y) / actual_scale
        nodes_world.append((float(wx), float(wy)))

    # 对于纯闭合环（无自然节点），做特殊标记
    if no_junctions and len(edges_pixel) == 1 and node_coords is not None:
        # 单闭合环：边从虚拟节点到自身
        edges_pixel[0]["from"] = 0
        edges_pixel[0]["to"] = 0
        edges_pixel[0]["is_loop"] = True

    return nodes_world, edges_pixel, skeleton, actual_scale
