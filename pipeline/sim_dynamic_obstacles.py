"""
sim_dynamic_obstacles.py — 动态障碍物场景仿真

管线: path2.png → map_parser → centerline → circuit
      → occupancy grid → mpc_planner 轨迹优化 (梯度下降 + 碰撞势场)
      → MPC 跟踪

用法: python pipeline/sim_dynamic_obstacles.py (需 conda 环境 CRL, Python 3.11)
"""

from __future__ import annotations

import os, sys, math
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.path import Path as MplPath
from matplotlib.patches import PathPatch, Polygon as MplPolygon

_self_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.dirname(_self_dir)
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

sys.path.insert(0, os.path.join(_self_dir, "..", "build", "pnc"))
import pnc

from pipeline.dynamic_obstacles import NpcVehicle, NpcManager

# ========================== 参数 ==========================

MASS     = 1573.0
IZ       = 2873.0
LF       = 1.1
LR       = 1.58
L_WB     = LF + LR
C_AF     = 80000.0
C_AR     = 80000.0
VX       = 10.0
DT       = 0.1
N_HORIZON = 40
LANE_WIDTH = 3.5
MAX_STEER  = math.radians(30.0)

CELL_SIZE      = 0.2
GATE_SPACING   = 15.0
HA_ARC_LENGTH  = 0.6
VEHICLE_HW     = 0.45   # 半宽, 全宽0.9m ≈ 赛道1/4
VEHICLE_FWD    = 0.8
VEHICLE_REV    = 0.5
SAFETY_MARGIN  = VEHICLE_HW + 0.15
COLLISION_MARGIN = VEHICLE_HW + 0.15
CORRIDOR_MARGIN  = COLLISION_MARGIN

BSPLINE_DEGREE       = 3
BSPLINE_NUM_CTRL     = 100
BSPLINE_RESAMPLE     = 0.5

NPC_SPEED    = 5.0
NPC_HW       = 1.0
NPC_FWD      = 1.5
NPC_REV      = 1.0
NPC_1_START  = 0.25
NPC_2_START  = 0.40

# 动态障碍物重规划参数
DYNAMIC_OBS_DILATION = 1.5   # NPC 在栅格中的膨胀距离 (m)
REPLAN_DIST   = 80.0         # NPC 进入此弧长距离触发重规划 (m)
LOOK_AHEAD    = 50.0         # HA* 局部规划前视距离 (m)
MIN_REPLAN_GAP = 5           # 两次重规划之间最少步数 (0.5s)

# =====================================================================
#  占用栅格构建
# =====================================================================

def _polygon_scanline_intersections(y, poly):
    xs = []
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


def build_occupancy_grid(outer, holes, cell_size, safety_margin):
    all_pts = np.vstack([outer] + list(holes))
    x_min, y_min = np.min(all_pts, axis=0)
    x_max, y_max = np.max(all_pts, axis=0)
    pad = safety_margin + 1.0
    x_min -= pad; y_min -= pad
    x_max += pad; y_max += pad
    cols = int((x_max - x_min) / cell_size) + 1
    rows = int((y_max - y_min) / cell_size) + 1
    print(f"  Grid: {rows}×{cols} cells, {(x_max-x_min):.0f}×{(y_max-y_min):.0f} m")

    outer_list = outer.tolist()
    holes_list = [h.tolist() for h in holes]
    grid = [[0] * cols for _ in range(rows)]

    for r in range(rows):
        y = y_min + r * cell_size + cell_size / 2.0
        outer_xs = _polygon_scanline_intersections(y, outer_list)
        for k in range(0, len(outer_xs) - 1, 2):
            xl, xr = outer_xs[k], outer_xs[k + 1]
            cl = int((xl - x_min) / cell_size); cr = int((xr - x_min) / cell_size)
            cl = max(0, cl); cr = min(cols - 1, cr)
            for c in range(cl, cr + 1):
                grid[r][c] = 1
        for hole in holes_list:
            hole_xs = _polygon_scanline_intersections(y, hole)
            for k in range(0, len(hole_xs) - 1, 2):
                xl, xr = hole_xs[k], hole_xs[k + 1]
                cl = int((xl - x_min) / cell_size); cr = int((xr - x_min) / cell_size)
                cl = max(0, cl); cr = min(cols - 1, cr)
                for c in range(cl, cr + 1):
                    grid[r][c] = 0
        for c in range(cols):
            grid[r][c] = 1 - grid[r][c]

    dilate_radius = int(safety_margin / cell_size)
    if dilate_radius > 0:
        pnc.dilate_grid(grid, dilate_radius)

    return grid, {
        "x_min": x_min, "y_min": y_min, "x_max": x_max, "y_max": y_max,
        "cols": cols, "rows": rows, "cell_size": cell_size,
    }


# =====================================================================
#  Gate 生成 + HA* Gate 规划
# =====================================================================

def generate_gates(centerline_pts, spacing_m, lane_width):
    n = len(centerline_pts)
    diffs = np.diff(centerline_pts, axis=0)
    seg_lens = np.sqrt(np.sum(diffs**2, axis=1))
    cum_s = np.concatenate([[0.0], np.cumsum(seg_lens)])
    total_len = float(cum_s[-1])
    tangents = np.zeros_like(centerline_pts)
    for i in range(n):
        prev_i = (i - 1) % n; next_i = (i + 1) % n
        dx = centerline_pts[next_i, 0] - centerline_pts[prev_i, 0]
        dy = centerline_pts[next_i, 1] - centerline_pts[prev_i, 1]
        norm = np.sqrt(dx*dx + dy*dy)
        if norm > 1e-9:
            tangents[i, 0] = dx / norm; tangents[i, 1] = dy / norm
    num_gates = max(2, int(total_len / spacing_m))
    gates = []
    half_w = lane_width / 2.0
    for i in range(num_gates):
        s = total_len * i / num_gates
        idx = min(np.searchsorted(cum_s, s), n - 1)
        if idx == 0:
            pt = centerline_pts[0]; tx, ty = tangents[0]
        else:
            s0, s1 = cum_s[idx-1], cum_s[idx]
            t = (s - s0) / (s1 - s0) if s1 > s0 else 0.0
            pt = (1-t) * centerline_pts[idx-1] + t * centerline_pts[idx]
            tx = (1-t) * tangents[idx-1, 0] + t * tangents[idx, 0]
            ty = (1-t) * tangents[idx-1, 1] + t * tangents[idx, 1]
            tnorm = np.sqrt(tx*tx + ty*ty)
            if tnorm > 1e-9: tx /= tnorm; ty /= tnorm
        nx, ny = -ty, tx
        gate_a = pnc.Vec2d(); gate_a.x = pt[0] + nx * half_w; gate_a.y = pt[1] + ny * half_w
        gate_b = pnc.Vec2d(); gate_b.x = pt[0] - nx * half_w; gate_b.y = pt[1] - ny * half_w
        gates.append((gate_a, gate_b))
    return gates


def plan_through_gates(grid, grid_meta, start_pose, gates):
    ha = pnc.HybridAStar(grid)
    ha.set_cell_size(CELL_SIZE); ha.set_wheelbase(L_WB)
    ha.set_max_steer(MAX_STEER); ha.set_num_steer(5)
    ha.set_arc_length(HA_ARC_LENGTH)
    ha.set_goal_xy_tol(1.0); ha.set_goal_th_tol(1.0)
    ha.set_vehicle_dims(COLLISION_MARGIN, VEHICLE_FWD + 0.2, VEHICLE_REV + 0.2)
    ha.set_grid_origin(grid_meta["x_min"], grid_meta["y_min"])
    full_path = []
    current = start_pose
    consecutive_failures = 0
    for gi, (gate_a, gate_b) in enumerate(gates):
        if gi == 0:
            mid = pnc.Pose()
            mid.x = (gate_a.x + gate_b.x) * 0.5; mid.y = (gate_a.y + gate_b.y) * 0.5
            mid.theta = 0.0
            try: seg = ha.plan(current, mid)
            except Exception: seg = []
        else:
            try: seg = ha.plan_to_gate(current, gate_a, gate_b)
            except Exception: seg = []
        if seg is None or len(seg) == 0:
            if gi > 0:
                ha.set_goal_xy_tol(3.0)
                try: seg = ha.plan_to_gate(current, gate_a, gate_b)
                except Exception: seg = []
                ha.set_goal_xy_tol(1.0)
            if seg is None or len(seg) == 0:
                consecutive_failures += 1
                if consecutive_failures >= 3: break
                continue
        consecutive_failures = 0
        if len(full_path) > 0 and len(seg) > 0:
            d = math.hypot(full_path[-1][0] - seg[0].x, full_path[-1][1] - seg[0].y)
            if d < 0.1: seg = seg[1:]
        for p in seg: full_path.append((p.x, p.y, p.theta))
        current = pnc.Pose()
        current.x = full_path[-1][0]; current.y = full_path[-1][1]
        current.theta = full_path[-1][2]
        if (gi + 1) % 5 == 0 or gi == len(gates) - 1:
            print(f"  Gate {gi+1}/{len(gates)}: path_len={len(full_path)}")
    print(f"  HA* 总路径: {len(full_path)} 点")
    return full_path


# =====================================================================
#  Safe Corridor + B-Spline
# =====================================================================

def smooth_path(raw_path, grid, grid_meta):
    ref_path = []
    for x, y, th in raw_path:
        pose = pnc.Pose(); pose.x = x; pose.y = y; pose.theta = th
        ref_path.append(pose)
    sc = pnc.SafeCorridor()
    sc.set_margin(CORRIDOR_MARGIN); sc.set_sample_interval(2.0)
    sc.set_vehicle_half_width(COLLISION_MARGIN)
    corridors = sc.build(ref_path, grid, grid_meta["x_min"], grid_meta["y_min"],
                         grid_meta["cell_size"], grid_meta["cols"], grid_meta["rows"])
    print(f"  安全走廊: {len(corridors)} sections")
    bs = pnc.BSpline()
    params = pnc.BSplineParams()
    params.degree = BSPLINE_DEGREE; params.num_control_points = BSPLINE_NUM_CTRL
    params.closed = False; params.resample_spacing = BSPLINE_RESAMPLE
    bs.set_params(params)
    fitted = bs.fit(ref_path, corridors)
    resampled = bs.resample(fitted)
    print(f"  B样条: {len(fitted)}→{len(resampled)} pts")
    result = [(p.x, p.y, p.theta) for p in resampled]
    return result, corridors


# =====================================================================
#  Trajectory
# =====================================================================

class Trajectory:
    def __init__(self, points: np.ndarray):
        self.points = points
        n = len(points)
        diffs = np.diff(points, axis=0)
        seg_lens = np.sqrt(np.sum(diffs**2, axis=1))
        self.cum_s = np.concatenate([[0.0], np.cumsum(seg_lens)])
        self.total_len = float(self.cum_s[-1])
        self.psi = np.zeros(n)
        if n >= 2:
            self.psi[0] = math.atan2(points[1,1]-points[0,1], points[1,0]-points[0,0])
            self.psi[-1] = math.atan2(points[-1,1]-points[-2,1], points[-1,0]-points[-2,0])
        for i in range(1, n-1):
            self.psi[i] = math.atan2(points[i+1,1]-points[i-1,1], points[i+1,0]-points[i-1,0])
        self.kappa = np.zeros(n)
        for i in range(1, n-1):
            dx = points[i+1,0]-points[i-1,0]; dy = points[i+1,1]-points[i-1,1]
            ddx = points[i+1,0]-2*points[i,0]+points[i-1,0]
            ddy = points[i+1,1]-2*points[i,1]+points[i-1,1]
            den = (dx*dx + dy*dy)**1.5
            self.kappa[i] = (dx*ddy - dy*ddx) / den if den > 1e-8 else 0.0
        from scipy.ndimage import uniform_filter1d
        ks = min(21, max(3, n-1))
        if ks % 2 == 0: ks -= 1
        self.kappa = uniform_filter1d(self.kappa, size=ks, mode='nearest')

    def get_state(self, s: float) -> np.ndarray:
        s_mod = s % self.total_len if self.total_len > 0 else s
        idx = min(np.searchsorted(self.cum_s, s_mod), len(self.cum_s)-1)
        if idx == 0:
            return np.array([self.points[0,0], self.points[0,1], self.psi[0], self.kappa[0]])
        s0, s1 = self.cum_s[idx-1], self.cum_s[idx]
        t = (s_mod - s0) / (s1 - s0) if s1 > s0 else 0.0
        x = (1-t)*self.points[idx-1,0] + t*self.points[idx,0]
        y = (1-t)*self.points[idx-1,1] + t*self.points[idx,1]
        d = self.psi[idx] - self.psi[idx-1]
        d = (d + math.pi) % (2*math.pi) - math.pi
        psi = self.psi[idx-1] + t * d
        k = (1-t)*self.kappa[idx-1] + t*self.kappa[idx]
        return np.array([x, y, psi, k])


# =====================================================================
#  MPC 矩阵 & 工具
# =====================================================================

def build_cont_matrices():
    A = np.array([
        [0, 1, 0, 0],
        [0, -(2*C_AF+2*C_AR)/(MASS*VX), (2*C_AF+2*C_AR)/MASS, -(2*C_AF*LF-2*C_AR*LR)/(MASS*VX)],
        [0, 0, 0, 1],
        [0, -(2*C_AF*LF-2*C_AR*LR)/(IZ*VX), (2*C_AF*LF-2*C_AR*LR)/IZ, -(2*C_AF*LF*LF+2*C_AR*LR*LR)/(IZ*VX)],
    ])
    B1 = np.array([[0], [2*C_AF/MASS], [0], [2*C_AF*LF/IZ]])
    B2 = np.array([[0], [-(2*C_AF*LF-2*C_AR*LR)/(MASS*VX) - VX], [0], [-(2*C_AF*LF*LF+2*C_AR*LR*LR)/(IZ*VX)]])
    return A, B1, B2


def feedforward(kappa):
    L = L_WB
    return L * kappa + (LR/(L*C_AF) - LF/(L*C_AR)) * MASS/2 * VX*VX * kappa


def get_ego_world_pose(ref, e_y, e_psi):
    x = ref[0] - e_y * math.sin(ref[2])
    y = ref[1] + e_y * math.cos(ref[2])
    heading = ref[2] + e_psi
    return x, y, heading


def _vehicle_corners_world(cx, cy, heading, hw, hf, hr):
    c = math.cos(heading); s = math.sin(heading)
    local = [(hf, hw), (hf, -hw), (-hr, -hw), (-hr, hw)]
    corners = []
    for lx, ly in local:
        wx = cx + lx * c - ly * s; wy = cy + lx * s + ly * c
        corners.append((wx, wy))
    return corners


def check_ego_in_bounds(ego_x, ego_y, ego_heading, outer_poly, mpl_path):
    corners = _vehicle_corners_world(ego_x, ego_y, ego_heading, VEHICLE_HW, VEHICLE_FWD, VEHICLE_REV)
    for cx, cy in corners:
        if not mpl_path.contains_point((cx, cy)): return False
    return True


def find_ego_centerline_s(ego_x, ego_y, centerline_pts, cum_s):
    """找自车在中心线上的最近弧长位置。"""
    dx = centerline_pts[:, 0] - ego_x
    dy = centerline_pts[:, 1] - ego_y
    idx = int(np.argmin(dx * dx + dy * dy))
    return float(cum_s[idx])


def interpolate_centerline_heading(x, y, centerline_pts, cum_s):
    """在中心线上插值 heading (避免 B-Spline 边界效应)。"""
    s = find_ego_centerline_s(x, y, centerline_pts, cum_s)
    n = len(centerline_pts)
    total = float(cum_s[-1])
    s = s % total
    idx = int(np.searchsorted(cum_s, s))
    idx = min(idx, n - 1)
    if idx == 0:
        return math.atan2(centerline_pts[1, 1] - centerline_pts[0, 1],
                          centerline_pts[1, 0] - centerline_pts[0, 0])
    # 前后段平均 heading
    h_prev = math.atan2(centerline_pts[idx, 1] - centerline_pts[idx-1, 1],
                        centerline_pts[idx, 0] - centerline_pts[idx-1, 0])
    if idx < n - 1:
        h_next = math.atan2(centerline_pts[idx+1, 1] - centerline_pts[idx, 1],
                            centerline_pts[idx+1, 0] - centerline_pts[idx, 0])
        # 插值
        s0, s1 = cum_s[idx-1], cum_s[idx]
        t = (s - s0) / (s1 - s0) if s1 > s0 else 0.0
        d = h_next - h_prev
        d = (d + math.pi) % (2 * math.pi) - math.pi
        return h_prev + t * d
    return h_prev


def compute_nearest_npc_dist(ego_x, ego_y, npc_manager, centerline_pts, cum_s):
    """自车到最近 NPC 的沿中心线正向弧长距离。"""
    ego_s = find_ego_centerline_s(ego_x, ego_y, centerline_pts, cum_s)
    total = float(cum_s[-1])
    best = float('inf')
    for npc_v in npc_manager.npcs:
        delta = (npc_v.s - ego_s) % total
        best = min(best, delta)
    return best


def find_look_ahead_pose(ego_x, ego_y, centerline_pts, cum_s, look_ahead_m):
    """在中心线上找自车前方 look_ahead_m 处的目标位姿。"""
    ego_s = find_ego_centerline_s(ego_x, ego_y, centerline_pts, cum_s)
    total = float(cum_s[-1])
    s_target = (ego_s + look_ahead_m) % total
    n = len(centerline_pts)
    idx = int(np.searchsorted(cum_s, s_target))
    idx = min(idx, n - 1)
    if idx == 0:
        x, y = float(centerline_pts[0, 0]), float(centerline_pts[0, 1])
        dx, dy = centerline_pts[1, 0] - centerline_pts[0, 0], centerline_pts[1, 1] - centerline_pts[0, 1]
    else:
        s0, s1 = cum_s[idx - 1], cum_s[idx]
        t = (s_target - s0) / (s1 - s0) if s1 > s0 else 0.0
        t = max(0.0, min(1.0, t))
        x = (1 - t) * float(centerline_pts[idx - 1, 0]) + t * float(centerline_pts[idx, 0])
        y = (1 - t) * float(centerline_pts[idx - 1, 1]) + t * float(centerline_pts[idx, 1])
        dx = centerline_pts[idx, 0] - centerline_pts[idx - 1, 0]
        dy = centerline_pts[idx, 1] - centerline_pts[idx - 1, 1]
    heading = math.atan2(dy, dx)
    return x, y, heading


def replace_trajectory_segment(traj_orig, ego_x, ego_y, new_smoothed_pts,
                                look_ahead_m, ego_local_s, replace_pts=80):
    """用重规划路径替换原始 Trajectory 的局部段落。

    始终基于 traj_orig (不变的初始轨迹) 做替换, 用固定点数窗口控制膨胀。
    ego_local_s 是自车在 traj_orig 上的弧长位置。
    replace_pts 是替换窗口的点数 (≈40m on original, expands to ~150m after smoothing)。
    """
    pts = traj_orig.points
    cum_s = traj_orig.cum_s
    n = len(pts)

    # 用 local_s 定位起始索引
    s_start = ego_local_s
    start_idx = int(np.searchsorted(cum_s, s_start))
    start_idx = max(start_idx, 3)
    start_idx = min(start_idx, n - replace_pts - 5)

    # 固定点数窗口
    end_idx = min(start_idx + replace_pts, n - 2)

    new_arr = np.array([(p[0], p[1]) for p in new_smoothed_pts])

    # 拼接: prefix + bridge + new_path + rejoin_point + suffix
    bridge_start = np.array([[ego_x, ego_y]])
    bridge_end = pts[end_idx:end_idx + 1]

    combined = np.vstack([
        pts[:start_idx],
        bridge_start,
        new_arr,
        bridge_end,
        pts[end_idx + 1:],
    ])

    if len(combined) < 10:
        return None

    return Trajectory(combined)


def _smooth_trajectory_pts(pts, sigma=1.5):
    """对轨迹 (x,y) 做高斯平滑, 消除拼接处的硬拐角。"""
    from scipy.ndimage import gaussian_filter1d
    smoothed = pts.copy()
    smoothed[:, 0] = gaussian_filter1d(pts[:, 0], sigma, mode='nearest')
    smoothed[:, 1] = gaussian_filter1d(pts[:, 1], sigma, mode='nearest')
    return smoothed


# =====================================================================
#  Main simulation
# =====================================================================

def run():
    print("=" * 60)
    print("  动态障碍物场景 — mpc_planner + MPC")
    print("=" * 60)

    from pipeline.map_parser import parse_map
    from pipeline.centerline import extract_centerline_graph
    from pipeline.sim_lane_keeping_real import assemble_go_straight_circuit

    # [1] 解析地图
    img = os.path.join(_project_root, "map", "path2.png")
    print(f"\n[1/6] 解析地图: {img}")
    bounds = parse_map(img, pixels_per_meter=12.8, smoothing_factor=0.0,
                       num_control_points=200, resample_spacing_m=0.1,
                       has_starting_line=True)
    outer = np.array(bounds["outer_boundary"])
    map_holes = [np.array(h) for h in bounds["holes"]]
    print(f"  Outer: {len(outer)} pts, Holes: {len(map_holes)}")

    # [2] 中心线 + 回路
    print("[2/6] 中心线 & 回路 ...")
    graph = extract_centerline_graph(bounds["outer_boundary"], bounds["holes"],
                                     pixels_per_meter=12.8, smoothing_factor=0.02,
                                     starting_line=bounds.get("starting_line"))
    loop_pts = assemble_go_straight_circuit(graph)
    cl_diffs = np.diff(loop_pts, axis=0)
    cl_cum_s = np.concatenate([[0.0], np.cumsum(np.sqrt(np.sum(cl_diffs**2, axis=1)))])
    cl_total_len = float(cl_cum_s[-1])
    print(f"  Centerline: {len(loop_pts)} pts, {cl_total_len:.1f} m")

    # [3] 占用栅格
    print("[3/6] 构建占用栅格 ...")
    base_grid, grid_meta = build_occupancy_grid(outer, map_holes, CELL_SIZE, SAFETY_MARGIN)

    # 辅助: 局部 mpc_planner 轨迹优化 (替代 HA* + SafeCorridor + B-Spline)
    def mpc_local_plan(grid_dyn, ego_x, ego_y, ego_heading, t_now=0.0):
        """从 ego 位姿起, 用 MPCTrajectoryPlanner 做局部轨迹优化。

        在中心线上采样参考路径, 通过梯度下降优化速度/转向序列,
        collisionCost (1/d² 排斥势场) 自动推开轨迹远离 NPC。
        t_now: 当前仿真时间, 用于动态障碍物预测。
        """
        n_horizon = 30
        dt_mpc = 0.1
        desired_speed = VX

        # 构建参考路径: 自车位姿 + 中心线前方等弧长采样
        ref_path = []
        ego_pose = pnc.Pose(); ego_pose.x = ego_x; ego_pose.y = ego_y
        ego_pose.theta = ego_heading
        ref_path.append(ego_pose)

        ego_s = find_ego_centerline_s(ego_x, ego_y, loop_pts, cl_cum_s)
        total_len = float(cl_cum_s[-1])
        horizon_dist = desired_speed * n_horizon * dt_mpc  # ≈30m

        for k in range(1, n_horizon + 1):
            s_target = (ego_s + horizon_dist * k / n_horizon) % total_len
            idx = int(np.searchsorted(cl_cum_s, s_target))
            idx = min(idx, len(loop_pts) - 1)
            if idx == 0:
                x, y = float(loop_pts[0, 0]), float(loop_pts[0, 1])
                dx = loop_pts[1, 0] - loop_pts[0, 0]
                dy = loop_pts[1, 1] - loop_pts[0, 1]
            else:
                s0, s1 = cl_cum_s[idx-1], cl_cum_s[idx]
                t_val = (s_target - s0) / (s1 - s0) if s1 > s0 else 0.0
                t_val = max(0.0, min(1.0, t_val))
                x = (1-t_val) * float(loop_pts[idx-1, 0]) + t_val * float(loop_pts[idx, 0])
                y = (1-t_val) * float(loop_pts[idx-1, 1]) + t_val * float(loop_pts[idx, 1])
                dx = loop_pts[idx, 0] - loop_pts[idx-1, 0]
                dy = loop_pts[idx, 1] - loop_pts[idx-1, 1]
            heading = math.atan2(dy, dx)
            ref_p = pnc.Pose(); ref_p.x = x; ref_p.y = y; ref_p.theta = heading
            ref_path.append(ref_p)

        # 创建优化器, 设置动态栅格 + 原点 + 权重
        planner = pnc.MPCTrajectoryPlanner(grid_dyn)
        planner.set_grid_origin(grid_meta["x_min"], grid_meta["y_min"],
                                grid_meta["cell_size"])
        planner.set_horizon(n_horizon)
        planner.set_dt(dt_mpc)
        planner.set_desired_speed(desired_speed)
        planner.set_max_iterations(50)    # 上限, 通常 20-30 次即收敛
        planner.set_gradient_step(0.02)
        # 碰撞权重 800 >> 位置权重 10, 确保排斥势场主导避障
        planner.set_cost_weights(10.0, 5.0, 2.0, 15.0, 800.0, 1.0)
        # 注册所有动态障碍物
        for obs in dynamic_obs_list:
            planner.add_dynamic_obstacle(obs)

        try:
            traj, velocities, steers = planner.plan(ref_path, t_now)
        except Exception:
            return None

        if traj is None or len(traj) < 3:
            return None

        return [(p.x, p.y, p.theta) for p in traj]

    # [4] 初始化 NPC (暂时跳过, 先验证纯轨迹跟踪)
    print("\n[4/6] 跳过 NPC (纯轨迹跟踪验证) ...")
    npc_manager = NpcManager()
    # npc_manager.add_npc(NpcVehicle(NPC_1_START, NPC_SPEED, NPC_HW, NPC_FWD, NPC_REV))
    # npc_manager.add_npc(NpcVehicle(NPC_2_START, NPC_SPEED, NPC_HW, NPC_FWD, NPC_REV))
    print(f"  NPC: 0 辆")

    # [5] 全局 HA* + SafeCorridor + B-Spline → 平滑基线轨迹 (仅一次)
    print("\n[5/6] 全局 HA* + SafeCorridor + B-Spline ...")
    gates = generate_gates(loop_pts, GATE_SPACING, LANE_WIDTH)
    print(f"  Gates: {len(gates)}")
    gates.append(gates[0])  # 闭环
    start_pose = pnc.Pose()
    start_pose.x = (gates[0][0].x + gates[0][1].x) * 0.5
    start_pose.y = (gates[0][0].y + gates[0][1].y) * 0.5
    gx = gates[0][0].x - gates[0][1].x
    gy = gates[0][0].y - gates[0][1].y
    start_pose.theta = math.atan2(-gx, gy)

    raw_path = plan_through_gates(base_grid, grid_meta, start_pose, gates)
    if len(raw_path) < 3:
        print("  ⚠ HA* 失败, 退回中心线")
        raw_path = [(x, y, 0.0) for x, y in loop_pts]

    smoothed_pts, corridors = smooth_path(raw_path, base_grid, grid_meta)
    base_pts = np.array([(p[0], p[1]) for p in smoothed_pts])
    base_traj = Trajectory(base_pts)
    traj = base_traj
    print(f"  基线轨迹: {traj.total_len:.1f} m, {len(base_pts)} pts, "
          f"max|kappa|={np.max(np.abs(traj.kappa)):.4f}")

    # 创建动态障碍物
    def _create_obs(s_obs, amp, period, phase=0.0):
        _s = s_obs % cl_total_len
        _idx = min(int(np.searchsorted(cl_cum_s, _s)), len(loop_pts) - 1)
        if _idx == 0:
            x_ref, y_ref = float(loop_pts[0, 0]), float(loop_pts[0, 1])
            h_ref = 0.0
        else:
            s0, s1 = cl_cum_s[_idx-1], cl_cum_s[_idx]
            _t = (_s - s0) / (s1 - s0) if s1 > s0 else 0.0
            _t = max(0.0, min(1.0, _t))
            x_ref = (1-_t) * float(loop_pts[_idx-1, 0]) + _t * float(loop_pts[_idx, 0])
            y_ref = (1-_t) * float(loop_pts[_idx-1, 1]) + _t * float(loop_pts[_idx, 1])
            h_ref = math.atan2(float(loop_pts[_idx, 1] - loop_pts[_idx-1, 1]),
                               float(loop_pts[_idx, 0] - loop_pts[_idx-1, 0]))
        obs = pnc.SinusoidalObstacle(x_ref, y_ref, h_ref, amp, period, phase)
        print(f"  障碍物: s={s_obs:.0f}m ({x_ref:.1f},{y_ref:.1f}), ±{amp:.1f}m / {period:.1f}s phase={phase:.2f}")
        return obs, (x_ref, y_ref, h_ref, amp, period, phase)

    dynamic_obs_list = []
    obs_info_list = []
    # 障碍物1: s=150m, ±3m, 5s, 相位π/2 让 ego 到达时在最大振幅
    obs1, info1 = _create_obs(150.0, 3.0, 5.0, math.pi/2)
    dynamic_obs_list.append(obs1); obs_info_list.append(info1)
    # 障碍物2: s=300m, ±2.5m, 3s, 相位π/2
    obs2, info2 = _create_obs(300.0, 2.5, 3.0, math.pi/2)
    dynamic_obs_list.append(obs2); obs_info_list.append(info2)

    print(f"\n[5/6] MPC 仿真 (ego={VX:.0f}m/s) ...")
    A_c, B1_c, B2_c = build_cont_matrices()
    A_d = np.eye(4) + A_c * DT; B1_d = B1_c * DT
    print(f"  |eig(A_d)| = {[f'{e:.4f}' for e in sorted(np.abs(np.linalg.eigvals(A_d)), reverse=True)]}")

    C_mat = np.eye(4); D_mat = np.zeros((4, 1))
    model = pnc.BicycleModel(A_c, B1_c, B2_c, C_mat, D_mat)
    init_state = np.array([-0.3, 0.0, 0.05, 0.0])
    P_kf = np.eye(4) * 1.0; Q_kf = np.eye(4) * 0.01
    R_kf = np.diag([0.1, 0.1, 0.025, 0.005])
    model.kf.init(A_d, B1_d, C_mat, P_kf, Q_kf, R_kf, init_state)
    model.init(init_state)
    Q = np.diag([80.0, 0.5, 15.0, 0.5]); R = np.array([[0.1]])
    S_term = np.eye(4) * 1.0
    mpc = pnc.MPC(); mpc.init(A_d, B1_d, C_mat, Q, R, S_term, N_HORIZON)

    N_STEPS = int(base_traj.total_len / (VX * DT)) + 50
    N_STEPS = min(N_STEPS, 5000)
    print(f"  Max {N_STEPS} steps = {N_STEPS*DT:.1f}s")

    mpl_outer = MplPath(outer)
    log, npc_log = [], []
    steer = 0.0; local_s = 0.0
    collision_npc = False; collision_boundary = False
    total_replans = 0; failed_replans = 0
    avoid_offset = 0.0  # 持久横向偏移

    for step in range(N_STEPS):
        t = step * DT; s_travel = t * VX

        # 更新 NPC (跳过 — 纯轨迹跟踪验证)
        # npc_manager.update(t, loop_pts, cl_cum_s, cl_total_len)
        npc_log.append([])

        # 参考状态 & 自车位姿
        ref = traj.get_state(local_s)
        kappa_cur = float(ref[3])
        e_y = float(model.x[0]); e_psi = float(model.x[2])
        ego_x, ego_y, ego_heading = get_ego_world_pose(ref, e_y, e_psi)
        ego_x = float(ego_x); ego_y = float(ego_y); ego_heading = float(ego_heading)

        # ---- 重规划 (只在障碍物附近生成避障偏移) ----
        ego_cl_s = find_ego_centerline_s(ego_x, ego_y, loop_pts, cl_cum_s)
        # 到最近障碍物的距离
        obs_positions = [o[0] for o in obs_info_list]  # s 值从 _create_obs 参数
        obs_positions = [150.0, 300.0]  # 硬编码对应两个障碍物的 s 位置
        dist_to_obs = min(
            min(abs(ego_cl_s - s), cl_total_len - abs(ego_cl_s - s))
            for s in obs_positions)

        if dist_to_obs < 80.0 and step % MIN_REPLAN_GAP == 0:
            total_replans += 1
            grid_dyn = [row[:] for row in base_grid]
            npc_manager.apply_to_grid(grid_dyn, grid_meta, DYNAMIC_OBS_DILATION)

            new_pts = mpc_local_plan(grid_dyn, ego_x, ego_y, ego_heading, t)
            if new_pts is not None and len(new_pts) > 2:
                # 从 mpc_planner 输出中提取期望横向偏移
                # 轨迹第 1→2 步的法向位移 = 避障方向
                dx = new_pts[1][0] - new_pts[0][0]
                dy = new_pts[1][1] - new_pts[0][1]
                nx = -math.sin(ref[2])
                ny = math.cos(ref[2])
                step_offset = dx * nx + dy * ny
                # 累加偏移方向 (用 MPC 纵向误差 target[0] 实现)
                if abs(step_offset) > 0.02:
                    avoid_offset = math.copysign(min(abs(step_offset) * 3.0, 2.5), step_offset)

                if total_replans <= 3 or total_replans % 20 == 0:
                    print(f"  [{step:4d}] 重规划 #{total_replans}: "
                          f"t={t:.1f}s offset={avoid_offset:+.2f}m "
                          f"obs_dist={dist_to_obs:.0f}m")
            else:
                failed_replans += 1

        # MPC
        model.kf.update(model.y, np.array([steer]))
        target = np.array([avoid_offset, 0.0, 0.0, 0.0])
        u_fb = mpc.predict(target, model.kf.x_post)
        steer_ff = feedforward(kappa_cur)
        steer = float(u_fb[0] + steer_ff)
        steer = max(-MAX_STEER, min(MAX_STEER, steer))
        model.step(DT, kappa_cur * VX, np.array([steer]))
        local_s += VX * DT

        log.append((step, t, e_y, e_psi, steer, ego_x, ego_y, ego_heading))

        # 碰撞检测 (NPC跳过, 只检边界)
        if not check_ego_in_bounds(ego_x, ego_y, ego_heading, outer, mpl_outer):
            if not collision_boundary:
                print(f"  ❌ [{step:4d}] t={t:.1f}s 超出道路边界!")
                collision_boundary = True
            break  # 一旦超出边界立即停止

        # 进度
        if step % 100 == 0:
            print(f"  {step:4d} t={t:5.1f}s s={s_travel:6.0f}m "
                  f"e_y={e_y:+.3f}m e_psi={math.degrees(e_psi):+.1f}deg "
                  f"str={math.degrees(steer):+.1f}deg")

        # 一圈完成
        if s_travel >= base_traj.total_len:
            print(f"\n  → 自车完成一圈! s={s_travel:.1f}m >= {base_traj.total_len:.1f}m")
            break

    # 统计
    final = log[-1]
    print(f"\n[仿真完成]")
    print(f"  步数: {len(log)}, 时间: {final[1]:.1f}s, 距离: {final[1]*VX:.0f}m")
    print(f"  重规划: {total_replans} 次 (失败 {failed_replans})")
    print(f"  NPC 碰撞: {'❌ 是' if collision_npc else '✅ 否'}")
    print(f"  边界碰撞: {'❌ 是' if collision_boundary else '✅ 否'}")
    print(f"  Final: e_y={final[2]:.4f}m  e_psi={math.degrees(final[3]):.2f}deg")

    # [6] 保存 & 可视化
    print("\n[6/6] 保存 & 可视化 ...")
    os.makedirs("output", exist_ok=True)
    arr = np.array(log)

    out_txt = "output/sim_dynamic_obstacles.txt"
    with open(out_txt, "w") as f:
        f.write(f"# REF: type=dynamic_obstacles len={cl_total_len:.1f}m dt={DT} Vx={VX}\n")
        f.write("Step\ttime\te_y\te_psi\tsteer\tego_x\tego_y\tego_heading\n")
        for row in log:
            f.write(f"{int(row[0])}\t" + "\t".join(f"{v:.6f}" for v in row[1:]) + "\n")
    np.save("output/sim_dynamic_obstacles_outer.npy", outer)
    np.save("output/sim_dynamic_obstacles_traj.npy", traj.points)
    for i, h in enumerate(map_holes):
        np.save(f"output/sim_dynamic_obstacles_hole_{i}.npy", h)
    np.save("output/sim_dynamic_obstacles_log.npy", arr)
    if len(npc_log) > 0 and len(npc_log[0]) > 0:
        npc_arr = np.array(npc_log)
        np.save("output/sim_dynamic_obstacles_npc.npy", npc_arr)

    # 保存动态障碍物参数 + 时间轴 (供动画脚本使用)
    obs_params = np.array(obs_info_list)  # (N_obs, 4): x_ref,y_ref,h_ref,amp,period
    np.save("output/sim_dynamic_obstacles_obs.npy", obs_params)
    t_arr = np.array([row[1] for row in log])
    np.save("output/sim_dynamic_obstacles_obs_t.npy", t_arr)

    visualize(log, traj, outer, map_holes, npc_log,
              loop_pts, collision_npc, collision_boundary, obs_info_list)


# =====================================================================
#  Visualization
# =====================================================================

def visualize(log, traj, outer, holes, npc_log,
              centerline_pts, collision_npc, collision_boundary,
              obs_info_list=None):
    arr = np.array(log, dtype=float)
    if arr.ndim != 2:
        arr = arr.reshape(-1, 8)
    N = len(arr)
    ts   = arr[:, 1]; ey = arr[:, 2]; ep = arr[:, 3]; st = arr[:, 4]
    ego_wx = arr[:, 5]; ego_wy = arr[:, 6]; ego_wh = arr[:, 7]

    # NPC 轨迹
    has_npcs = len(npc_log) > 0 and len(npc_log[0]) > 0
    if has_npcs:
        npc_arr = np.array(npc_log)[:N]
        npc1_x, npc1_y = npc_arr[:, 0, 0], npc_arr[:, 0, 1]
        npc2_x, npc2_y = npc_arr[:, 1, 0], npc_arr[:, 1, 1]
    else:
        npc1_x = npc1_y = npc2_x = npc2_y = np.array([])

    fig = plt.figure(figsize=(20, 14))
    status = "⚠ COLLISION" if (collision_npc or collision_boundary) else "✅ OK"
    fig.suptitle(f"Dynamic Obstacles — Lane Keeping + NPC Tracking  [{status}]", fontsize=14)

    # --- 主图: 轨迹鸟瞰 ---
    ax = fig.add_subplot(2, 3, (1, 4))
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
    ax.set_title("Track, NPCs & Ego Trajectory")

    ax.plot(outer[:, 0], outer[:, 1], "k-", lw=1.5, alpha=0.5, label="Track boundary")
    for i, h in enumerate(holes):
        ax.fill(h[:, 0], h[:, 1], fc="white", ec="k", lw=0.8, alpha=0.95)

    # 参考路径 (最终的追加式轨迹)
    if traj is not None and traj.points is not None:
        ax.plot(traj.points[:, 0], traj.points[:, 1], "b-", lw=1.5, alpha=0.4, label="Reference")

    # NPC 轨迹
    if has_npcs:
        skip = max(1, N // 300)
        ax.plot(npc1_x[::skip], npc1_y[::skip], "orange", lw=1.5, alpha=0.8, ls="--", label="NPC 1")
        ax.plot(npc2_x[::skip], npc2_y[::skip], "purple", lw=1.5, alpha=0.8, ls="--", label="NPC 2")
        ax.plot(npc1_x[0], npc1_y[0], "o", color="orange", ms=8)
        ax.plot(npc2_x[0], npc2_y[0], "o", color="purple", ms=8)

        # NPC 车辆矩形 (每 ~5s)
        for i in range(0, N, max(1, int(5.0 / DT))):
            for color, npx, npy, nph in [
                    ("orange", npc1_x[i], npc1_y[i], npc_arr[i, 0, 2]),
                    ("purple", npc2_x[i], npc2_y[i], npc_arr[i, 1, 2])]:
                corners = _vehicle_corners_world(npx, npy, nph, NPC_HW, NPC_FWD, NPC_REV)
                ax.add_patch(MplPolygon(corners, closed=True, fc=color, ec="k", alpha=0.3, lw=0.5))

    # 自车轨迹
    skip_e = max(1, N // 500)
    ax.plot(ego_wx[::skip_e], ego_wy[::skip_e], "r-", lw=1.8, alpha=0.9, label="Ego")
    ax.plot(ego_wx[0], ego_wy[0], "go", ms=10, label="Start")
    ax.plot(ego_wx[-1], ego_wy[-1], "mo", ms=10, label="End")
    for i in range(0, N, max(1, N // 8)):
        ax.arrow(ego_wx[i], ego_wy[i], math.cos(ego_wh[i]) * 2.5, math.sin(ego_wh[i]) * 2.5,
                 head_width=1.2, fc="r", ec="r", alpha=0.4)

    # 动态障碍物: 中心点 + ±振幅扫描范围
    if obs_info_list is not None:
        colors = ['orange', 'cyan']
        for i, obs_info in enumerate(obs_info_list):
            ox, oy, oh, amp = obs_info[:4]
            c = colors[i % len(colors)]
            nx, ny = -math.sin(oh), math.cos(oh)
            ax.plot(ox, oy, 'D', color=c, ms=10, label=f'Obs{i+1}')
            for sign in [-1, 1]:
                end_x = ox + sign * amp * nx
                end_y = oy + sign * amp * ny
                ax.plot([ox, end_x], [oy, end_y], color=c, lw=2, alpha=0.5)
            ax.plot([ox - amp*nx, ox + amp*nx], [oy - amp*ny, oy + amp*ny],
                    color=c, lw=1, ls='--', alpha=0.3)

    ax.legend(loc="upper right", fontsize=7, ncol=2)

    # --- 横向误差 ---
    ax = fig.add_subplot(2, 3, 2)
    ax.plot(ts, ey, "b-", lw=1.5)
    ax.axhline(0, color="k", ls="--", lw=0.5)
    ax.set_ylabel("m"); ax.set_title("Lateral Error")
    ax.grid(True, alpha=0.3)

    # --- 航向误差 ---
    ax = fig.add_subplot(2, 3, 3)
    ax.plot(ts, np.degrees(ep), "r-", lw=1.5)
    ax.axhline(0, color="k", ls="--", lw=0.5)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("deg")
    ax.set_title("Heading Error"); ax.grid(True, alpha=0.3)

    # --- NPC 距离 ---
    ax = fig.add_subplot(2, 3, 5)
    if has_npcs:
        d1 = np.sqrt((ego_wx - npc1_x)**2 + (ego_wy - npc1_y)**2)
        d2 = np.sqrt((ego_wx - npc2_x)**2 + (ego_wy - npc2_y)**2)
        ax.plot(ts, d1, "orange", lw=1.0, alpha=0.7, label="NPC 1")
        ax.plot(ts, d2, "purple", lw=1.0, alpha=0.7, label="NPC 2")
        ax.axhline(2.5, color="red", ls=":", lw=0.8, alpha=0.5, label="collision")
        min_d1, min_d2 = float(np.min(d1[N//10:])), float(np.min(d2[N//10:]))
    else:
        d1 = d2 = np.array([])
        min_d1, min_d2 = float('inf'), float('inf')
        ax.text(0.5, 0.5, "无 NPC", transform=ax.transAxes, ha='center', va='center')
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Distance (m)")
    ax.set_title("Ego-NPC Distance")
    if has_npcs:
        ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # --- 转向 ---
    ax = fig.add_subplot(2, 3, 6)
    ax.plot(ts, np.degrees(st), "g-", lw=1.5)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("deg")
    ax.set_title("Steering Angle"); ax.grid(True, alpha=0.3)

    plt.tight_layout()

    skip_w = max(1, N // 10)
    ey_rms = float(np.sqrt(np.mean(ey[skip_w:]**2)))
    ep_rms = float(np.sqrt(np.mean(ep[skip_w:]**2)))
    if has_npcs and len(d1) > skip_w:
        min_d1 = float(np.min(d1[skip_w:]))
        min_d2 = float(np.min(d2[skip_w:]))
        print(f"  RMS: e_y={ey_rms:.4f}m  e_psi={math.degrees(ep_rms):.2f}deg")
        print(f"  Min NPC dist: NPC1={min_d1:.2f}m  NPC2={min_d2:.2f}m")
    else:
        print(f"  RMS: e_y={ey_rms:.4f}m  e_psi={math.degrees(ep_rms):.2f}deg")

    out_png = "output/sim_dynamic_obstacles.png"
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"  Plot: {out_png}")
    plt.show()


if __name__ == "__main__":
    run()
