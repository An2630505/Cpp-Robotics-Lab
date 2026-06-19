"""
_smooth.py — 角点检测 + 分段样条平滑 + 弧长均匀重采样

检测轮廓中的尖锐角点，在角点处拆分后分段使用开曲线三次样条，
保留路口交叉处的尖锐几何特征。对无角点的平滑轮廓同样适用。
"""

import numpy as np

try:
    from scipy.interpolate import splprep, splev
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False

# 角点判定阈值：局部切线方向变化超过此角度即标记为角点
_CORNER_ANGLE_THRESHOLD_DEG = 35.0


# ---------------------------------------------------------------------------
# 内部工具
# ---------------------------------------------------------------------------

def _subsample(points: np.ndarray, n: int, endpoint: bool = False) -> np.ndarray:
    """将 N 个点均匀降采样到 n 个点。"""
    if len(points) <= n:
        return points
    indices = np.linspace(0, len(points) - 1, n, endpoint=endpoint, dtype=int)
    return points[indices]


def _arc_length(points: np.ndarray) -> float:
    """多点折线的总弧长。"""
    if len(points) < 2:
        return 0.0
    diffs = np.diff(points, axis=0)
    return float(np.sum(np.sqrt(np.sum(diffs ** 2, axis=1))))


def _resample_by_arc_length(
    fine_points: np.ndarray,
    spacing: float,
    *,
    endpoint: bool = False,
) -> np.ndarray:
    """对细粒度点做等弧长重采样。"""
    diffs = np.diff(fine_points, axis=0)
    seg_lens = np.sqrt(np.sum(diffs ** 2, axis=1))
    cum_len = np.concatenate([[0.0], np.cumsum(seg_lens)])
    total_len = cum_len[-1]

    if total_len < 1e-12:
        return fine_points[:1]

    num_samples = max(2, int(np.ceil(total_len / spacing)))
    target_arclen = np.linspace(0, total_len, num_samples, endpoint=endpoint)

    x_re = np.interp(target_arclen, cum_len, fine_points[:, 0])
    y_re = np.interp(target_arclen, cum_len, fine_points[:, 1])
    return np.column_stack([x_re, y_re])


def _resample_linear(points: np.ndarray, spacing: float) -> np.ndarray:
    """折线线性插值重采样（点数不足时回退使用）。"""
    if len(points) < 2:
        return points
    return _resample_by_arc_length(points, spacing, endpoint=True)


# ---------------------------------------------------------------------------
# 角点检测
# ---------------------------------------------------------------------------

def _detect_corners(
    points: np.ndarray,
    stride: int = 5,
    min_spacing: int = 10,
) -> np.ndarray:
    """
    通过局部切线方向变化检测轮廓中的尖锐角点。

    返回角点在 points 中的索引（升序）。无角点则返回空数组。
    """
    n = len(points)
    if n < 2 * stride + 3:
        return np.array([], dtype=int)

    threshold_rad = np.deg2rad(_CORNER_ANGLE_THRESHOLD_DEG)

    # 逐点计算转弯角度
    angles = np.zeros(n)
    for i in range(n):
        i_fwd = (i + stride) % n
        i_bwd = (i - stride) % n

        v1 = points[i_fwd] - points[i]
        v2 = points[i] - points[i_bwd]

        len1 = float(np.linalg.norm(v1))
        len2 = float(np.linalg.norm(v2))
        if len1 < 1e-12 or len2 < 1e-12:
            angles[i] = 0.0
            continue

        cos_angle = np.clip(np.dot(v1, v2) / (len1 * len2), -1.0, 1.0)
        angles[i] = float(np.arccos(cos_angle))

    corner_mask = angles > threshold_rad
    candidate_indices = np.where(corner_mask)[0]
    if len(candidate_indices) == 0:
        return np.array([], dtype=int)

    # 非极大值抑制：连续候选点归组，每组保留角度最大的
    groups: list[list[int]] = []
    current_group = [int(candidate_indices[0])]
    for idx in candidate_indices[1:]:
        if int(idx) - current_group[-1] <= 2:
            current_group.append(int(idx))
        else:
            groups.append(current_group)
            current_group = [int(idx)]
    groups.append(current_group)

    peaks = np.array([max(g, key=lambda i: angles[i]) for g in groups], dtype=int)
    if len(peaks) <= 1:
        return peaks

    # 最小间距过滤：贪心保留角度最大的
    order = np.argsort(-angles[peaks])
    kept: set[int] = set()
    for idx in order:
        p = int(peaks[idx])
        too_close = any(min(abs(p - k), n - abs(p - k)) < min_spacing for k in kept)
        if not too_close:
            kept.add(p)

    return np.array(sorted(kept), dtype=int)


# ---------------------------------------------------------------------------
# 轮廓拆分与拼接
# ---------------------------------------------------------------------------

def _split_at_corners(
    points: np.ndarray,
    corner_indices: np.ndarray,
) -> list[np.ndarray]:
    """在角点处将闭合轮廓拆分为若干开曲线段。无角点时返回单段。"""
    if len(corner_indices) == 0:
        return [points]

    corners = np.sort(corner_indices)
    segments: list[np.ndarray] = []

    for i in range(len(corners)):
        start = corners[i]
        end = corners[(i + 1) % len(corners)]
        if end > start:
            segments.append(points[start:end + 1])
        else:
            segments.append(np.concatenate([points[start:], points[:end + 1]]))

    return segments


def _concat_segments(segments: list[np.ndarray]) -> np.ndarray:
    """将多段拼接回闭合轮廓，去掉段间重复的端点角点。"""
    if len(segments) == 1:
        return segments[0]
    parts = [segments[0]]
    for seg in segments[1:]:
        parts.append(seg[1:])  # 跳过与上一段末尾重复的角点
    return np.concatenate(parts)


# ---------------------------------------------------------------------------
# 单段开曲线样条平滑
# ---------------------------------------------------------------------------

def _smooth_segment(
    segment: np.ndarray,
    num_control_points: int,
    smoothing_factor: float,
    resample_spacing: float,
) -> np.ndarray:
    """
    对一段开曲线做 cubic spline (per=0) 平滑 + 弧长重采样。

    端点锚定到原始角点坐标，保留尖锐几何。
    点数不足 4 时回退到折线线性重采样。
    """
    corner_start = segment[0].copy()
    corner_end = segment[-1].copy()

    n_ctrl = min(num_control_points, len(segment))
    pts = _subsample(segment, n_ctrl, endpoint=True)

    if len(pts) < 4:
        return _resample_linear(segment, resample_spacing)

    try:
        tck, _ = splprep(
            [pts[:, 0], pts[:, 1]],
            s=smoothing_factor,
            per=0,
            k=min(3, len(pts) - 1),
        )
    except Exception:
        return _resample_linear(segment, resample_spacing)

    n_eval = max(num_control_points * 5, 200)
    u_fine = np.linspace(0, 1, n_eval)
    x_fine, y_fine = splev(u_fine, tck)
    fine_points = np.column_stack([x_fine, y_fine])

    result = _resample_by_arc_length(fine_points, resample_spacing, endpoint=True)

    # 锚定端点 — 保留尖锐几何的关键
    result[0] = corner_start
    result[-1] = corner_end
    return result


# ---------------------------------------------------------------------------
# 公开 API
# ---------------------------------------------------------------------------

def smooth_contour(
    points: np.ndarray,
    smoothing_factor: float = 0.0,
    num_control_points: int = 200,
    resample_spacing: float | None = None,
) -> np.ndarray:
    """
    对一条闭合轮廓做角点检测 + 分段 open cubic spline 平滑。

    自动检测轮廓中的尖锐角点，在角点处拆分后分段拟合，
    保留路口交叉处的尖锐几何。若无角点则整段平滑。

    Parameters
    ----------
    points : np.ndarray, shape (N, 2)
        原始轮廓点（世界坐标，米）。
    smoothing_factor : float
        splprep 的 s 参数。0 = 纯插值，越大越平滑。默认 0.0。
    num_control_points : int
        样条拟合控制点总数，按各段弧长比例分配。默认 200。
    resample_spacing : float | None
        输出点弧长间距（米）。None 时不重采样。

    Returns
    -------
    np.ndarray, shape (M, 2)
        平滑后的闭合轮廓点。
    """
    if not _HAS_SCIPY:
        raise ImportError(
            "map_parser._smooth 需要 scipy。请执行: pip install scipy"
        )

    # 确保闭合（去掉重复终点）
    if np.linalg.norm(points[0] - points[-1]) < 1e-12:
        points = points[:-1]

    if len(points) < 4:
        raise ValueError(
            f"轮廓至少需要 4 个点才能做 cubic spline，当前 {len(points)} 个点"
        )

    # 角点检测
    corners = _detect_corners(points)

    # 在角点处拆分
    segments = _split_at_corners(points, corners)

    # 按各段弧长比例分配控制点
    seg_lengths = [_arc_length(seg) for seg in segments]
    total_len = sum(seg_lengths)
    if total_len < 1e-12:
        return points[:1]

    smoothed_segments: list[np.ndarray] = []

    for seg, seg_len in zip(segments, seg_lengths):
        n_ctrl = max(4, int(num_control_points * seg_len / total_len))

        if resample_spacing is None:
            # 不重采样：细粒度求值 + 锚定端点
            pts = _subsample(seg, n_ctrl, endpoint=True)
            if len(pts) < 4:
                smoothed_segments.append(seg)
                continue
            try:
                tck, _ = splprep(
                    [pts[:, 0], pts[:, 1]],
                    s=smoothing_factor, per=0,
                    k=min(3, len(pts) - 1),
                )
            except Exception:
                smoothed_segments.append(seg)
                continue
            u_eval = np.linspace(0, 1, max(n_ctrl, 200))
            x_s, y_s = splev(u_eval, tck)
            smoothed = np.column_stack([x_s, y_s])
            smoothed[0] = seg[0]
            smoothed[-1] = seg[-1]
            smoothed_segments.append(smoothed)
        elif len(seg) >= 4:
            smoothed_segments.append(
                _smooth_segment(seg, n_ctrl, smoothing_factor, resample_spacing)
            )
        else:
            smoothed_segments.append(_resample_linear(seg, resample_spacing))

    return _concat_segments(smoothed_segments)


def smooth_all_contours(
    outer: np.ndarray | None,
    holes: list[np.ndarray],
    smoothing_factor: float = 0.0,
    num_control_points: int = 200,
    resample_spacing: float | None = None,
) -> tuple[np.ndarray | None, list[np.ndarray]]:
    """对一条外边界 + N 条孔洞做平滑。"""
    smoothed_outer = None
    if outer is not None and len(outer) >= 4:
        smoothed_outer = smooth_contour(
            outer, smoothing_factor, num_control_points, resample_spacing,
        )

    smoothed_holes = []
    for hole in holes:
        if len(hole) >= 4:
            smoothed_holes.append(
                smooth_contour(
                    hole, smoothing_factor, num_control_points, resample_spacing,
                )
            )

    return smoothed_outer, smoothed_holes
