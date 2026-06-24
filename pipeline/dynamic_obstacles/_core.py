"""
NPC 车辆运动学与栅格管理。
"""

import math
import numpy as np


class NpcVehicle:
    """沿道路中心线匀速行驶的 NPC 车辆。

    车辆坐标系: x 向前 (车头方向), y 向左。
    位姿 (x, y, heading) 以后轴中心为参考点。
    """

    def __init__(self, start_ratio: float, speed: float,
                 half_width: float, half_len_fwd: float, half_len_rev: float):
        """
        Args:
            start_ratio: 起始弧长比例, 0~1 (占中心线总长的比例)
            speed: 匀速行驶速度, m/s
            half_width: 车辆半宽, m
            half_len_fwd: 后轴到车头距离, m
            half_len_rev: 后轴到车尾距离, m
        """
        self.start_ratio = start_ratio
        self.speed = speed
        self.half_width = half_width
        self.half_len_fwd = half_len_fwd
        self.half_len_rev = half_len_rev

        # 当前状态
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.s = 0.0  # 当前弧长

    def update(self, t: float, centerline_pts: np.ndarray,
               cum_s: np.ndarray, total_len: float):
        """在时刻 t 更新 NPC 位姿。

        Args:
            t: 仿真时间, s
            centerline_pts: (N,2) 中心线点列
            cum_s: (N,) 累积弧长
            total_len: 中心线总长, m
        """
        s_travel = self.speed * t
        self.s = (self.start_ratio * total_len + s_travel) % total_len
        self.x, self.y, self.heading = _interpolate_pose(
            self.s, centerline_pts, cum_s)

    def get_pose(self, t: float, centerline_pts: np.ndarray,
                 cum_s: np.ndarray, total_len: float):
        """预测时刻 t 的位姿，不修改内部状态。

        Returns:
            (x, y, heading)
        """
        s_travel = self.speed * t
        s = (self.start_ratio * total_len + s_travel) % total_len
        return _interpolate_pose(s, centerline_pts, cum_s)

    def get_corners(self) -> list:
        """返回车辆 4 个角点的世界坐标，顺序: 左前, 右前, 右后, 左后。

        Returns:
            [(x,y), (x,y), (x,y), (x,y)]
        """
        return _vehicle_corners(self.x, self.y, self.heading,
                                self.half_width, self.half_len_fwd,
                                self.half_len_rev)


class NpcManager:
    """管理所有 NPC 车辆，提供批量更新、栅格注入和碰撞检测。"""

    def __init__(self):
        self.npcs: list[NpcVehicle] = []

    def add_npc(self, npc: NpcVehicle):
        self.npcs.append(npc)

    def __len__(self):
        return len(self.npcs)

    def update(self, t: float, centerline_pts: np.ndarray,
               cum_s: np.ndarray, total_len: float):
        """更新所有 NPC 在时刻 t 的位姿。"""
        for npc in self.npcs:
            npc.update(t, centerline_pts, cum_s, total_len)

    def apply_to_grid(self, grid: list, grid_meta: dict,
                      dilation_m: float):
        """将所有 NPC 作为障碍物画入占用栅格。

        用膨胀后的半尺寸直接绘制矩形，不调用 pnc.dilate_grid，
        避免二次膨胀道路边界。

        Args:
            grid: list[list[int]], 0=自由, 1=障碍物 (原地修改)
            grid_meta: dict with x_min, y_min, cell_size, cols, rows
            dilation_m: 膨胀距离, m
        """
        x_min = grid_meta['x_min']
        y_min = grid_meta['y_min']
        cell_size = grid_meta['cell_size']
        cols = grid_meta['cols']
        rows = grid_meta['rows']

        for npc in self.npcs:
            # 膨胀后的半尺寸
            dl_hw = npc.half_width + dilation_m
            dl_hf = npc.half_len_fwd + dilation_m
            dl_hr = npc.half_len_rev + dilation_m

            corners = _vehicle_corners(npc.x, npc.y, npc.heading,
                                       dl_hw, dl_hf, dl_hr)
            corner_arr = np.array(corners)

            # 世界坐标 bounding box
            wx_min, wy_min = corner_arr.min(axis=0)
            wx_max, wy_max = corner_arr.max(axis=0)

            # 栅格坐标 bounding box
            c_min = max(0, int((wx_min - x_min) / cell_size))
            c_max = min(cols - 1, int((wx_max - x_min) / cell_size))
            r_min = max(0, int((wy_min - y_min) / cell_size))
            r_max = min(rows - 1, int((wy_max - y_min) / cell_size))

            # 变换到局部坐标系所需的量
            c_h = math.cos(npc.heading)
            s_h = math.sin(npc.heading)
            cx, cy = npc.x, npc.y

            for r in range(r_min, r_max + 1):
                for c in range(c_min, c_max + 1):
                    # Cell 中心世界坐标
                    wx = x_min + c * cell_size + cell_size / 2.0
                    wy = y_min + r * cell_size + cell_size / 2.0

                    # 变换到 NPC 局部坐标系
                    dx = wx - cx
                    dy = wy - cy
                    lx = dx * c_h + dy * s_h
                    ly = -dx * s_h + dy * c_h

                    if -dl_hr <= lx <= dl_hf and -dl_hw <= ly <= dl_hw:
                        grid[r][c] = 1

    def check_collision_with_ego(self, ego_x: float, ego_y: float,
                                 ego_heading: float,
                                 ego_hw: float, ego_hf: float, ego_hr: float
                                 ) -> bool:
        """SAT (Separating Axis Theorem) 检测自车是否与任何 NPC 碰撞。

        Args:
            ego_x, ego_y: 自车后轴中心世界坐标
            ego_heading: 自车航向角
            ego_hw, ego_hf, ego_hr: 自车半宽/前半长/后半长

        Returns:
            True if collision detected
        """
        for npc in self.npcs:
            if _obb_collision(
                    ego_x, ego_y, ego_heading, ego_hw, ego_hf, ego_hr,
                    npc.x, npc.y, npc.heading,
                    npc.half_width, npc.half_len_fwd, npc.half_len_rev):
                return True
        return False

    def get_positions(self):
        """返回所有 NPC 的当前位姿列表。

        Returns:
            [(x, y, heading), ...]
        """
        return [(npc.x, npc.y, npc.heading) for npc in self.npcs]


# =====================================================================
#  工具函数
# =====================================================================

def _interpolate_pose(s: float, pts: np.ndarray, cum_s: np.ndarray):
    """在中心线上按弧长 s 插值位姿 (x, y, heading)。

    Args:
        s: 弧长, m
        pts: (N,2) 中心线点列
        cum_s: (N,) 累积弧长

    Returns:
        (x, y, heading)
    """
    n = len(pts)
    if n < 2:
        return float(pts[0, 0]), float(pts[0, 1]), 0.0

    # 处理 s 超出范围的情况 (closed loop)
    total_len = float(cum_s[-1])
    s = s % total_len

    # 找到 s 所在的线段
    idx = int(np.searchsorted(cum_s, s))
    idx = min(idx, n - 1)

    if idx == 0:
        x, y = float(pts[0, 0]), float(pts[0, 1])
        dx = pts[1, 0] - pts[0, 0]
        dy = pts[1, 1] - pts[0, 1]
    else:
        s0, s1 = cum_s[idx - 1], cum_s[idx]
        t = (s - s0) / (s1 - s0) if s1 > s0 else 0.0
        t = max(0.0, min(1.0, t))
        x = (1.0 - t) * float(pts[idx - 1, 0]) + t * float(pts[idx, 0])
        y = (1.0 - t) * float(pts[idx - 1, 1]) + t * float(pts[idx, 1])
        dx = pts[idx, 0] - pts[idx - 1, 0]
        dy = pts[idx, 1] - pts[idx - 1, 1]

    heading = math.atan2(dy, dx)
    return x, y, heading


def _vehicle_corners(cx: float, cy: float, heading: float,
                     hw: float, hf: float, hr: float) -> list:
    """计算车辆 4 个角点的世界坐标。

    局部坐标系: x 向前, y 向左。
    角点顺序: 左前, 右前, 右后, 左后。
    """
    c = math.cos(heading)
    s = math.sin(heading)

    local = [
        (hf, hw),    # 左前
        (hf, -hw),   # 右前
        (-hr, -hw),  # 右后
        (-hr, hw),   # 左后
    ]

    corners = []
    for lx, ly in local:
        wx = cx + lx * c - ly * s
        wy = cy + lx * s + ly * c
        corners.append((wx, wy))
    return corners


def _obb_collision(ax: float, ay: float, ah: float,
                   ahw: float, ahf: float, ahr: float,
                   bx: float, by: float, bh: float,
                   bhw: float, bhf: float, bhr: float) -> bool:
    """SAT (Separating Axis Theorem) 检测两个 OBB 是否重叠。

    两个 OBB 均以后轴中心为参考点。
    检测 4 条边法向 (A 的 2 条 + B 的 2 条) 上的投影是否分离。
    """
    ca = _vehicle_corners(ax, ay, ah, ahw, ahf, ahr)
    cb = _vehicle_corners(bx, by, bh, bhw, bhf, bhr)

    # 取两条边的法向作为分离轴
    for corners in [ca, cb]:
        for i in range(2):
            ex = corners[i + 1][0] - corners[i][0]
            ey = corners[i + 1][1] - corners[i][1]
            # 法向 = (-ey, ex)
            nx = -ey
            ny = ex
            norm = math.hypot(nx, ny)
            if norm < 1e-9:
                continue
            nx /= norm
            ny /= norm

            # A 投影
            proj_a = [cx * nx + cy * ny for cx, cy in ca]
            min_a, max_a = min(proj_a), max(proj_a)

            # B 投影
            proj_b = [cx * nx + cy * ny for cx, cy in cb]
            min_b, max_b = min(proj_b), max(proj_b)

            if max_a < min_b - 1e-9 or max_b < min_a - 1e-9:
                return False  # 找到分离轴 → 无碰撞

    return True  # 无分离轴 → 碰撞
