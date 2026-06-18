"""
_core.py — 地图解析核心管线

图像 → 灰度 → 二值化 → 轮廓提取 → 分类 → 世界坐标转换 → 样条平滑
"""

from __future__ import annotations

import cv2
import numpy as np

from ._smooth import smooth_all_contours


def _binarize(
    gray: np.ndarray,
    method: str = "otsu",
    manual_threshold: int | None = None,
) -> tuple[np.ndarray, int]:
    """
    将灰度图二值化，亮色 = 赛道 = 255，暗色 = 背景 = 0。

    返回 (binary_image, threshold_used)。
    """
    if method == "otsu":
        thresh_val, binary = cv2.threshold(
            gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )
        return binary, int(thresh_val)

    if method == "adaptive":
        binary = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            blockSize=11,
            C=2,
        )
        return binary, -1  # adaptive 没有单一阈值

    if method == "manual":
        if manual_threshold is None:
            raise ValueError("threshold_method='manual' 时必须提供 manual_threshold")
        if not 0 <= manual_threshold <= 255:
            raise ValueError(f"manual_threshold 必须在 0-255 之间，当前 {manual_threshold}")
        _, binary = cv2.threshold(gray, manual_threshold, 255, cv2.THRESH_BINARY)
        return binary, manual_threshold

    raise ValueError(
        f"未知的 threshold_method: '{method}'。可选: otsu, adaptive, manual"
    )


def _extract_contours(
    binary: np.ndarray,
    min_contour_area: int = 100,
) -> tuple[list[np.ndarray], list[np.ndarray], np.ndarray | None]:
    """
    从二值图中提取并分类轮廓。

    返回 (outer_boundaries, hole_boundaries, hierarchy)
    """
    contours, hierarchy = cv2.findContours(
        binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE
    )
    if hierarchy is None:
        return [], [], None

    outer_boundaries: list[np.ndarray] = []
    hole_boundaries: list[np.ndarray] = []

    for i, cnt in enumerate(contours):
        if len(cnt) < min_contour_area:
            continue
        # hierarchy[0][i] = [next, prev, first_child, parent]
        if hierarchy[0][i][3] == -1:
            outer_boundaries.append(cnt)
        else:
            hole_boundaries.append(cnt)

    return outer_boundaries, hole_boundaries, hierarchy


def _contour_to_world(
    cnt: np.ndarray,
    pixels_per_meter: float,
) -> np.ndarray:
    """将 OpenCV 轮廓 (N, 1, 2) 转为世界坐标 (N, 2)。"""
    points = cnt[:, 0, :].astype(np.float64)
    return points / pixels_per_meter


def parse_map(  # noqa: PLR0913
    image_path: str,
    pixels_per_meter: float = 12.8,
    smoothing_factor: float = 0.0,
    num_control_points: int = 200,
    resample_spacing_m: float | None = None,
    threshold_method: str = "otsu",
    manual_threshold: int | None = None,
    min_contour_area: int = 100,
) -> dict:
    """
    从赛道渲染图中提取几何边界。

    参数
    ----
    image_path : str
        输入 JPG/PNG 图像的路径。
    pixels_per_meter : float
        图像像素到世界米的缩放比例（默认 12.8 = 1280px/100m）。
    smoothing_factor : float
        样条平滑系数。0 = 完全插值（无平滑），越大越平滑。
        默认 0.0 保留原始精度。当图像有锯齿时可设为 0.01~0.05。
    num_control_points : int
        splprep 控制点数量，影响样条拟合精度。
    resample_spacing_m : float | None
        输出点弧长间距（米）。None 时自动取 1/pixels_per_meter。
    threshold_method : str
        二值化方法：'otsu'（默认）、'adaptive'、'manual'。
    manual_threshold : int | None
        threshold_method='manual' 时的 0-255 阈值。
    min_contour_area : int
        最小轮廓像素数，小于此值的视为噪声丢弃。

    返回
    ----
    dict — 可直接 json.dumps:
        {
            "outer_boundary": [[x, y], ...],   # 外边界，shape (M, 2)
            "holes": [ [[x,y],...], ... ],      # 孔洞列表
            "metadata": { ... }
        }
    """
    # --- Step 1: 加载图像 ---
    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError(f"无法读取图像: {image_path}")

    h, w = img.shape[:2]

    # --- Step 2: 灰度化 + 二值化 ---
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    binary, thresh_used = _binarize(gray, method=threshold_method,
                                    manual_threshold=manual_threshold)

    # --- Step 3 + 4: 轮廓提取 + 分类 ---
    outer_list, hole_list, _ = _extract_contours(binary, min_contour_area)

    # --- 转换为世界坐标 ---
    outer_world = [_contour_to_world(c, pixels_per_meter) for c in outer_list]
    hole_world = [_contour_to_world(c, pixels_per_meter) for c in hole_list]

    # --- Step 5: 样条平滑 ---
    resample_sp = resample_spacing_m
    if resample_sp is None:
        resample_sp = 1.0 / pixels_per_meter

    # 取第一条（也是唯一一条）外边界。多外边界取其最大者。
    outer_raw: np.ndarray | None = None
    if outer_world:
        if len(outer_world) > 1:
            # 按周长选最大的
            best_idx = max(
                range(len(outer_world)),
                key=lambda i: len(outer_world[i]),
            )
            outer_raw = outer_world[best_idx]
        else:
            outer_raw = outer_world[0]

    smoothed_outer, smoothed_holes = smooth_all_contours(
        outer_raw,
        hole_world,
        smoothing_factor=smoothing_factor,
        num_control_points=num_control_points,
        resample_spacing=resample_sp,
    )

    # --- 组装返回 ---
    return {
        "outer_boundary": (
            smoothed_outer.tolist() if smoothed_outer is not None else []
        ),
        "holes": [h.tolist() for h in smoothed_holes],
        "metadata": {
            "image_path": image_path,
            "image_size": [w, h],
            "pixels_per_meter": pixels_per_meter,
            "num_outer_contours_found": len(outer_list),
            "num_holes_found": len(hole_list),
            "threshold_used": thresh_used,
            "smoothing_factor": smoothing_factor,
        },
    }
