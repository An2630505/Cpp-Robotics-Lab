"""
_smooth.py — 样条平滑 + 弧长均匀重采样

对闭合轮廓做 cubic periodic spline 拟合，消除像素级锯齿，
并在弧长上均匀重采样，保证输出点间距一致。
"""

import numpy as np

try:
    from scipy.interpolate import splprep, splev
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False


def _subsample(points: np.ndarray, n: int) -> np.ndarray:
    """将 N 个点均匀降采样到 n 个点（保持闭合、不含重复终点）。"""
    if len(points) <= n:
        return points
    indices = np.linspace(0, len(points) - 1, n, endpoint=False, dtype=int)
    return points[indices]


def smooth_contour(
    points: np.ndarray,
    smoothing_factor: float = 0.0,
    num_control_points: int = 200,
    resample_spacing: float | None = None,
) -> np.ndarray:
    """
    对一条闭合轮廓做 periodic cubic spline 拟合 + 弧长重采样。

    参数
    ----
    points : np.ndarray, shape (N, 2)
        原始轮廓点（世界坐标，米）。会自动降采样到 num_control_points。
    smoothing_factor : float
        splprep 的 s 参数。0 = 插值（完全忠实），越大越平滑。
        默认 0.0 = 插值，保留原始精度。需要平滑时可设 0.01~0.1。
    num_control_points : int
        送入样条拟合的采样点数。原始轮廓会先均匀降采样到此数量，
        控制拟合精度和计算开销。
    resample_spacing : float | None
        输出点的弧长间距（米）。None 时不做重采样，直接返回样条
        在细粒度上的采样点。

    返回
    ----
    np.ndarray, shape (M, 2)
        平滑 + 重采样后的闭合轮廓点。
    """
    if not _HAS_SCIPY:
        raise ImportError(
            "map_parser._smooth 需要 scipy。请执行: pip install scipy"
        )

    # 确保闭合（移除重复的终点，per=1 的 splprep 不需要它）
    if np.linalg.norm(points[0] - points[-1]) < 1e-12:
        points = points[:-1]

    if len(points) < 4:
        raise ValueError(f"轮廓至少需要 4 个点才能做 cubic spline，当前 {len(points)} 个点")

    # 降采样控制拟合开销
    pts = _subsample(points, num_control_points)

    # 样条拟合 — per=1 自动保证 C2 闭合
    try:
        tck, _u = splprep(
            [pts[:, 0], pts[:, 1]],
            s=smoothing_factor,
            per=1,
            k=3,
        )
    except Exception as e:
        raise RuntimeError(
            f"splprep 拟合失败（smoothing_factor={smoothing_factor}, "
            f"N={len(pts)}）。可尝试减小 smoothing_factor。原始错误: {e}"
        ) from e

    if resample_spacing is None:
        # 不做重采样，返回样条在细粒度上的求值
        u_eval = np.linspace(0, 1, max(num_control_points, 200))
        x_smooth, y_smooth = splev(u_eval, tck)
        return np.column_stack([x_smooth, y_smooth])

    # ---- 弧长均匀重采样 ----
    # 在细粒度上求值以准确计算弧长
    n_eval = max(num_control_points * 5, 1000)
    u_fine = np.linspace(0, 1, n_eval)
    x_fine, y_fine = splev(u_fine, tck)
    fine_points = np.column_stack([x_fine, y_fine])

    # 累积弧长
    diffs = np.diff(fine_points, axis=0)
    seg_lens = np.sqrt(np.sum(diffs ** 2, axis=1))
    cum_len = np.concatenate([[0.0], np.cumsum(seg_lens)])
    total_len = cum_len[-1]

    if total_len < 1e-12:
        return fine_points[:1]

    # 等弧长采样（不包含终点，闭合曲线终点=起点）
    num_samples = max(2, int(np.ceil(total_len / resample_spacing)))
    target_arclen = np.linspace(0, total_len, num_samples, endpoint=False)

    x_resampled = np.interp(target_arclen, cum_len, x_fine)
    y_resampled = np.interp(target_arclen, cum_len, y_fine)

    return np.column_stack([x_resampled, y_resampled])


def smooth_all_contours(
    outer: np.ndarray | None,
    holes: list[np.ndarray],
    smoothing_factor: float = 0.0,
    num_control_points: int = 200,
    resample_spacing: float | None = None,
) -> tuple[np.ndarray | None, list[np.ndarray]]:
    """对一条外边界 + N 条孔洞做平滑。纯粹的批量调用封装。"""
    smoothed_outer = None
    if outer is not None and len(outer) >= 4:
        smoothed_outer = smooth_contour(
            outer, smoothing_factor, num_control_points, resample_spacing
        )

    smoothed_holes = []
    for hole in holes:
        if len(hole) >= 4:
            smoothed_holes.append(
                smooth_contour(
                    hole, smoothing_factor, num_control_points, resample_spacing
                )
            )

    return smoothed_outer, smoothed_holes
