"""
_smooth_open.py — 开/闭曲线样条平滑 + 弧长重采样

区别于 map_parser._smooth：
- smooth_open_curve 处理开曲线（splprep per=0）
- smooth_closed_curve 处理闭合曲线（splprep per=1）
"""

from __future__ import annotations

import numpy as np

try:
    from scipy.interpolate import splprep, splev
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False


def _subsample(points: np.ndarray, n: int) -> np.ndarray:
    """均匀降采样到 n 个点。"""
    if len(points) <= n:
        return points
    indices = np.linspace(0, len(points) - 1, n, endpoint=True, dtype=int)
    return points[indices]


def _arc_len_resample(
    tck: tuple,
    n_eval: int,
    resample_spacing: float,
    closed: bool = False,
) -> np.ndarray:
    """在样条上做弧长均匀重采样。"""
    u_fine = np.linspace(0, 1, n_eval)
    x_fine, y_fine = splev(u_fine, tck)
    fine_points = np.column_stack([x_fine, y_fine])

    diffs = np.diff(fine_points, axis=0)
    seg_lens = np.sqrt(np.sum(diffs ** 2, axis=1))
    cum_len = np.concatenate([[0.0], np.cumsum(seg_lens)])
    total_len = cum_len[-1]

    if total_len < 1e-12:
        return fine_points[:1]

    if closed:
        num_samples = max(2, int(np.ceil(total_len / resample_spacing)))
        target_arclen = np.linspace(0, total_len, num_samples, endpoint=False)
    else:
        num_samples = max(2, int(np.ceil(total_len / resample_spacing)) + 1)
        target_arclen = np.linspace(0, total_len, num_samples)

    x_res = np.interp(target_arclen, cum_len, x_fine)
    y_res = np.interp(target_arclen, cum_len, y_fine)

    return np.column_stack([x_res, y_res])


def smooth_open_curve(
    points: np.ndarray,
    smoothing_factor: float = 0.02,
    resample_spacing: float = 0.078,
    num_control_points: int = 200,
) -> np.ndarray:
    """
    对开曲线做 cubic spline 拟合 + 弧长重采样。

    参数
    ----
    points : (N, 2) 原始点
    smoothing_factor : splprep s 参数
    resample_spacing : 输出弧长间距（米）
    num_control_points : 降采样控制点数

    返回
    ----
    (M, 2) 平滑 + 重采样后的点序列
    """
    if not _HAS_SCIPY:
        raise ImportError("需要 scipy。请执行: pip install scipy")

    if len(points) < 4:
        return points.copy()

    pts = _subsample(points, min(num_control_points, len(points)))

    try:
        tck, _u = splprep(
            [pts[:, 0], pts[:, 1]],
            s=smoothing_factor,
            per=0,
            k=3,
        )
    except Exception as e:
        raise RuntimeError(
            f"开曲线样条拟合失败（smoothing_factor={smoothing_factor}, "
            f"N={len(pts)}）。原始错误: {e}"
        ) from e

    n_eval = max(num_control_points * 8, 500)
    return _arc_len_resample(tck, n_eval, resample_spacing, closed=False)


def smooth_closed_curve(
    points: np.ndarray,
    smoothing_factor: float = 0.02,
    resample_spacing: float = 0.078,
    num_control_points: int = 200,
) -> np.ndarray:
    """
    对闭合曲线做 periodic cubic spline 拟合 + 弧长重采样。
    与 map_parser._smooth.smooth_contour 逻辑一致，但简化版。
    """
    if not _HAS_SCIPY:
        raise ImportError("需要 scipy。请执行: pip install scipy")

    # 确保闭合（移除重复端点，per=1 不需要）
    if np.linalg.norm(points[0] - points[-1]) < 1e-12:
        points = points[:-1]

    if len(points) < 4:
        return points.copy()

    pts = _subsample(points, min(num_control_points, len(points)))

    try:
        tck, _u = splprep(
            [pts[:, 0], pts[:, 1]],
            s=smoothing_factor,
            per=1,
            k=3,
        )
    except Exception as e:
        raise RuntimeError(
            f"闭合曲线样条拟合失败（smoothing_factor={smoothing_factor}, "
            f"N={len(pts)}）。原始错误: {e}"
        ) from e

    n_eval = max(num_control_points * 8, 500)
    return _arc_len_resample(tck, n_eval, resample_spacing, closed=True)
