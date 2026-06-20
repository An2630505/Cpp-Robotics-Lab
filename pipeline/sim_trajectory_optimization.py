"""
sim_trajectory_optimization.py — Hybrid A* + B-Spline 轨迹优化

管线: path2.jpg → map_parser → centerline → gates → Hybrid A*
      → SafeCorridor → BSpline → MPC → 仿真

用法: python pipeline/sim_trajectory_optimization.py
"""

from __future__ import annotations

import os, sys, math
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

_self_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.dirname(_self_dir)
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

sys.path.insert(0, os.path.join(_self_dir, "..", "build", "pnc"))
import pnc

# ========================== 参数 ==========================

# 车辆物理参数 (与 sim_lane_keeping_real.py 一致)
MASS     = 1573.0
IZ       = 2873.0
LF       = 1.1
LR       = 1.58
L_WB     = LF + LR
C_AF     = 80000.0
C_AR     = 80000.0
VX       = 10.0       # m/s, 恒定速度
DT       = 0.1        # 仿真步长
N_HORIZON = 40
LANE_WIDTH = 3.5
MAX_STEER  = math.radians(30.0)

# Hybrid A* 参数
CELL_SIZE      = 0.2       # 栅格分辨率 (m)
SAFETY_MARGIN  = 0.5       # 安全边距 (m)
GATE_SPACING   = 15.0      # Gate 间距 (m)
HA_ARC_LENGTH  = 0.6       # HA* 单步弧长 (缩小以应对窄弯)
VEHICLE_HW     = 0.5       # 车半宽 (网格已膨胀，减小避免过度保守)
VEHICLE_FWD    = 0.8       # 前方延伸
VEHICLE_REV    = 0.5       # 后方延伸

# B-Spline 参数
BSPLINE_DEGREE       = 3
BSPLINE_NUM_CTRL     = 50
BSPLINE_RESAMPLE     = 0.5    # 重采样间距 (m)


# =====================================================================
#  占用栅格构建
# =====================================================================

def _polygon_scanline_intersections(y, poly):
    """
    扫描线与多边形所有边的水平交点 (x 值列表).
    """
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
    """
    从边界多边形构建占用栅格 (扫描线填充, 快).

    outer : (N,2) numpy — 外边界
    holes : list of (M,2) numpy — 孔洞
    cell_size : float — 栅格分辨率 (m)
    safety_margin : float — 膨胀距离 (m)

    Returns: grid (list of list of int), meta dict
    """
    all_pts = np.vstack([outer] + list(holes))
    x_min, y_min = np.min(all_pts, axis=0)
    x_max, y_max = np.max(all_pts, axis=0)

    pad = safety_margin + 1.0
    x_min -= pad; y_min -= pad
    x_max += pad; y_max += pad

    cols = int((x_max - x_min) / cell_size) + 1
    rows = int((y_max - y_min) / cell_size) + 1
    print(f"  Grid: {rows}×{cols} cells, "
          f"{(x_max-x_min):.0f}×{(y_max-y_min):.0f} m")

    outer_list = outer.tolist()
    holes_list = [h.tolist() for h in holes]

    grid = [[0] * cols for _ in range(rows)]

    for r in range(rows):
        y = y_min + r * cell_size + cell_size / 2.0

        # 外边界交点
        outer_xs = _polygon_scanline_intersections(y, outer_list)
        # 从外边界交点的偶-奇区间填充
        for k in range(0, len(outer_xs) - 1, 2):
            xl = outer_xs[k]
            xr = outer_xs[k + 1]
            cl = int((xl - x_min) / cell_size)
            cr = int((xr - x_min) / cell_size)
            cl = max(0, cl); cr = min(cols - 1, cr)
            for c in range(cl, cr + 1):
                grid[r][c] = 1

        # 孔洞: 挖空
        for hole in holes_list:
            hole_xs = _polygon_scanline_intersections(y, hole)
            for k in range(0, len(hole_xs) - 1, 2):
                xl = hole_xs[k]
                xr = hole_xs[k + 1]
                cl = int((xl - x_min) / cell_size)
                cr = int((xr - x_min) / cell_size)
                cl = max(0, cl); cr = min(cols - 1, cr)
                for c in range(cl, cr + 1):
                    grid[r][c] = 0

        # 翻转: grid 中 0=障碍物(外/孔洞), 1=自由 → 需变为 0=自由, 1=障碍物
        for c in range(cols):
            grid[r][c] = 1 - grid[r][c]

    # 膨胀安全边距 (in-place 修改 grid)
    dilate_radius = int(safety_margin / cell_size)
    if dilate_radius > 0:
        pnc.dilate_grid(grid, dilate_radius)

    meta = {
        "x_min": x_min, "y_min": y_min,
        "x_max": x_max, "y_max": y_max,
        "cols": cols, "rows": rows,
        "cell_size": cell_size,
    }
    return grid, meta


# =====================================================================
#  Gate 生成
# =====================================================================

def generate_gates(centerline_pts, spacing_m, lane_width):
    """
    沿中心线等距生成横截面 Gate 线段。

    centerline_pts: (N,2) numpy — 中心线点
    spacing_m: float — Gate 间距 (弧长 m)
    lane_width: float — 车道宽度 (m)

    Returns: list of (Vec2d, Vec2d) — 每道门 (a, b)
    """
    n = len(centerline_pts)
    # 累积弧长
    diffs = np.diff(centerline_pts, axis=0)
    seg_lens = np.sqrt(np.sum(diffs**2, axis=1))
    cum_s = np.concatenate([[0.0], np.cumsum(seg_lens)])
    total_len = float(cum_s[-1])

    # 切线方向 (中心差分, 闭环绕组)
    tangents = np.zeros_like(centerline_pts)
    for i in range(n):
        prev_i = (i - 1) % n
        next_i = (i + 1) % n
        dx = centerline_pts[next_i, 0] - centerline_pts[prev_i, 0]
        dy = centerline_pts[next_i, 1] - centerline_pts[prev_i, 1]
        norm = np.sqrt(dx*dx + dy*dy)
        if norm > 1e-9:
            tangents[i, 0] = dx / norm
            tangents[i, 1] = dy / norm

    # 沿弧长采样
    num_gates = max(2, int(total_len / spacing_m))
    gates = []
    half_w = lane_width / 2.0

    for i in range(num_gates):
        s = total_len * i / num_gates
        # 二分查找弧长对应点
        idx = min(np.searchsorted(cum_s, s), n - 1)
        if idx == 0:
            pt = centerline_pts[0]
            tx, ty = tangents[0]
        else:
            s0, s1 = cum_s[idx-1], cum_s[idx]
            t = (s - s0) / (s1 - s0) if s1 > s0 else 0.0
            pt = (1-t) * centerline_pts[idx-1] + t * centerline_pts[idx]
            tx  = (1-t) * tangents[idx-1, 0] + t * tangents[idx, 0]
            ty  = (1-t) * tangents[idx-1, 1] + t * tangents[idx, 1]
            tnorm = np.sqrt(tx*tx + ty*ty)
            if tnorm > 1e-9:
                tx /= tnorm; ty /= tnorm

        # 法向: 左旋 90°
        nx, ny = -ty, tx
        gate_a = pnc.Vec2d()
        gate_a.x = pt[0] + nx * half_w
        gate_a.y = pt[1] + ny * half_w
        gate_b = pnc.Vec2d()
        gate_b.x = pt[0] - nx * half_w
        gate_b.y = pt[1] - ny * half_w
        gates.append((gate_a, gate_b))

    return gates


# =====================================================================
#  Hybrid A* Gate 规划
# =====================================================================

def plan_through_gates(grid, grid_meta, start_pose, gates):
    """
    用 Hybrid A* 逐 Gate 规划最短路径。

    Returns: list of (x, y, theta)
    """
    ha = pnc.HybridAStar(grid)
    ha.set_cell_size(CELL_SIZE)
    ha.set_wheelbase(L_WB)
    ha.set_max_steer(MAX_STEER)
    ha.set_num_steer(5)
    ha.set_arc_length(HA_ARC_LENGTH)
    ha.set_goal_xy_tol(1.0)
    ha.set_goal_th_tol(1.0)
    ha.set_vehicle_dims(VEHICLE_HW, VEHICLE_FWD, VEHICLE_REV)

    full_path = []
    current = start_pose
    last_good_gate = -1
    consecutive_failures = 0

    for gi, (gate_a, gate_b) in enumerate(gates):
        if gi == 0:
            mid = pnc.Pose()
            mid.x = (gate_a.x + gate_b.x) * 0.5
            mid.y = (gate_a.y + gate_b.y) * 0.5
            mid.theta = 0.0
            try:
                seg = ha.plan(current, mid)
            except Exception:
                seg = []
        else:
            try:
                seg = ha.plan_to_gate(current, gate_a, gate_b)
            except Exception:
                seg = []

        if seg is None or len(seg) == 0:
            # 回退: 增大 goal_xy_tol 重试
            if gi > 0:
                old_tol = 1.0
                ha.set_goal_xy_tol(3.0)
                try:
                    seg = ha.plan_to_gate(current, gate_a, gate_b)
                except Exception:
                    seg = []
                ha.set_goal_xy_tol(old_tol)

            if seg is None or len(seg) == 0:
                consecutive_failures += 1
                print(f"  ⚠ Gate {gi}: 规划失败, 跳过 "
                      f"(连续失败={consecutive_failures})")
                if consecutive_failures >= 3:
                    print(f"  → 连续{consecutive_failures}次失败, 停止门规划")
                    break
                continue

        consecutive_failures = 0
        last_good_gate = gi

        # 拼接 (去重)
        if len(full_path) > 0 and len(seg) > 0:
            d = math.hypot(full_path[-1][0] - seg[0].x,
                           full_path[-1][1] - seg[0].y)
            if d < 0.1:
                seg = seg[1:]

        for p in seg:
            full_path.append((p.x, p.y, p.theta))

        current = pnc.Pose()
        current.x = full_path[-1][0]
        current.y = full_path[-1][1]
        current.theta = full_path[-1][2]

        if (gi + 1) % 5 == 0 or gi == len(gates) - 1:
            print(f"  Gate {gi+1}/{len(gates)}: path_len={len(full_path)}")

    print(f"  HA* 总路径: {len(full_path)} 点 "
          f"(最后成功 gate={last_good_gate})")
    return full_path


# =====================================================================
#  Safe Corridor + B-Spline 平滑
# =====================================================================

def smooth_path(raw_path, outer, holes):
    """
    对 Hybrid A* 原始路径构建安全走廊 + B 样条拟合。

    raw_path: list of (x, y, theta)
    outer: (N,2) numpy
    holes: list of (M,2) numpy

    Returns: list of (x, y, theta) 平滑等弧长路径
    """
    # 转换为 C++ Pose 列表
    ref_path = []
    for x, y, th in raw_path:
        pose = pnc.Pose()
        pose.x = x; pose.y = y; pose.theta = th
        ref_path.append(pose)

    # 转换边界为 Vec2d 列表
    outer_vec = []
    for x, y in outer:
        v = pnc.Vec2d(); v.x = x; v.y = y
        outer_vec.append(v)

    holes_vec = []
    for h in holes:
        hv = []
        for x, y in h:
            v = pnc.Vec2d(); v.x = x; v.y = y
            hv.append(v)
        holes_vec.append(hv)

    # Step 1: 构建安全走廊
    sc = pnc.SafeCorridor()
    sc.set_margin(SAFETY_MARGIN)
    sc.set_sample_interval(2.0)
    corridors = sc.build(ref_path, outer_vec, holes_vec)
    print(f"  安全走廊: {len(corridors)} sections")

    # Step 2: B 样条拟合
    bs = pnc.BSpline()
    params = pnc.BSplineParams()
    params.degree = BSPLINE_DEGREE
    params.num_control_points = BSPLINE_NUM_CTRL
    params.closed = True
    params.resample_spacing = BSPLINE_RESAMPLE
    bs.set_params(params)

    fitted = bs.fit(ref_path, corridors)
    print(f"  B样条拟合: {len(fitted)} 点")

    # Step 3: 等弧长重采样
    resampled = bs.resample(fitted)
    print(f"  重采样: {len(resampled)} 点")

    # 转回 tuple 列表
    result = [(p.x, p.y, p.theta) for p in resampled]
    return result


# =====================================================================
#  Trajectory (from smoothed points, 复用 sim_lane_keeping_real 模式)
# =====================================================================

class Trajectory:
    def __init__(self, points: np.ndarray):
        """
        points: (N,2) numpy — (x,y) 闭环绕行轨迹
        """
        self.points = points
        n = len(points)

        diffs = np.diff(points, axis=0)
        seg_lens = np.sqrt(np.sum(diffs**2, axis=1))
        self.cum_s = np.concatenate([[0.0], np.cumsum(seg_lens)])
        self.total_len = float(self.cum_s[-1])

        self.psi = np.zeros(n)
        self.psi[0] = math.atan2(points[1,1]-points[0,1],
                                  points[1,0]-points[0,0])
        self.psi[-1] = math.atan2(points[-1,1]-points[-2,1],
                                   points[-1,0]-points[-2,0])
        for i in range(1, n-1):
            self.psi[i] = math.atan2(points[i+1,1]-points[i-1,1],
                                      points[i+1,0]-points[i-1,0])

        self.kappa = np.zeros(n)
        for i in range(1, n-1):
            dx  = points[i+1,0] - points[i-1,0]
            dy  = points[i+1,1] - points[i-1,1]
            ddx = points[i+1,0] - 2*points[i,0] + points[i-1,0]
            ddy = points[i+1,1] - 2*points[i,1] + points[i-1,1]
            den = (dx*dx + dy*dy)**1.5
            self.kappa[i] = (dx*ddy - dy*ddx) / den if den > 1e-8 else 0.0

        from scipy.ndimage import uniform_filter1d
        # 强平滑避免 kappa 突变导致 MPC 发散
        self.kappa = uniform_filter1d(self.kappa, size=21, mode='wrap')

    def get_state(self, s: float) -> np.ndarray:
        """[x, y, psi, kappa] at arc-length s."""
        s_mod = s % self.total_len if self.total_len > 0 else s
        idx = min(np.searchsorted(self.cum_s, s_mod), len(self.cum_s)-1)
        if idx == 0:
            return np.array([self.points[0,0], self.points[0,1],
                             self.psi[0], self.kappa[0]])
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
#  Continuous state-space matrices (复用)
# =====================================================================

def build_cont_matrices():
    A = np.array([
        [0, 1, 0, 0],
        [0, -(2*C_AF+2*C_AR)/(MASS*VX),  (2*C_AF+2*C_AR)/MASS,
         -(2*C_AF*LF-2*C_AR*LR)/(MASS*VX)],
        [0, 0, 0, 1],
        [0, -(2*C_AF*LF-2*C_AR*LR)/(IZ*VX),  (2*C_AF*LF-2*C_AR*LR)/IZ,
         -(2*C_AF*LF*LF+2*C_AR*LR*LR)/(IZ*VX)],
    ])
    B1 = np.array([[0], [2*C_AF/MASS], [0], [2*C_AF*LF/IZ]])
    B2 = np.array([
        [0],
        [-(2*C_AF*LF-2*C_AR*LR)/(MASS*VX) - VX],
        [0],
        [-(2*C_AF*LF*LF+2*C_AR*LR*LR)/(IZ*VX)],
    ])
    return A, B1, B2


def feedforward(kappa):
    L = L_WB
    return L * kappa + (LR/(L*C_AF) - LF/(L*C_AR)) * MASS/2 * VX*VX * kappa


# =====================================================================
#  Main simulation
# =====================================================================

def run():
    print("=" * 60)
    print("  轨迹优化 — Hybrid A* + B-Spline + MPC")
    print("=" * 60)

    from pipeline.map_parser import parse_map
    from pipeline.centerline import extract_centerline_graph
    from pipeline.sim_lane_keeping_real import assemble_go_straight_circuit

    # Step 1: 解析地图
    img = os.path.join(_project_root, "map", "path2.png")
    print(f"\n[1/7] Parsing: {img}")
    bounds = parse_map(img, pixels_per_meter=12.8, smoothing_factor=0.0,
                       num_control_points=200, resample_spacing_m=0.1,
                       has_starting_line=True)
    outer = np.array(bounds["outer_boundary"])
    holes = [np.array(h) for h in bounds["holes"]]
    print(f"  Outer: {len(outer)} pts, Holes: {len(holes)}")
    # 保存边界供动画使用
    np.save("output/sim_trajectory_optimization_outer.npy", outer)
    for i, h in enumerate(holes):
        np.save(f"output/sim_trajectory_optimization_hole_{i}.npy", h)

    # Step 2: 提取中心线
    print("[2/7] Extracting centerline ...")
    graph = extract_centerline_graph(bounds["outer_boundary"], bounds["holes"],
                                     pixels_per_meter=12.8, smoothing_factor=0.02,
                                     starting_line=bounds.get("starting_line"))
    print(f"  {len(graph['nodes'])} nodes, {len(graph['edges'])} edges")

    # Step 3: 组装闭环回路 (中心线)
    print("[3/7] Building centerline circuit ...")
    loop_pts = assemble_go_straight_circuit(graph)
    print(f"  Centerline: {len(loop_pts)} pts")

    # Step 4: 构建占用栅格
    print("[4/7] Building occupancy grid ...")
    grid, grid_meta = build_occupancy_grid(outer, holes, CELL_SIZE, SAFETY_MARGIN)
    rows, cols = grid_meta["rows"], grid_meta["cols"]
    obs_count = sum(sum(row) for row in grid)
    free_count = rows * cols - obs_count
    print(f"  Free: {free_count}, Occupied: {obs_count}")

    # Step 5: 生成 Gate + HA* 规划
    print("[5/7] Planning with Hybrid A* ...")
    gates = generate_gates(loop_pts, GATE_SPACING, LANE_WIDTH)
    print(f"  Gates: {len(gates)} (spacing={GATE_SPACING}m)")

    # 将 gate[0] 追加为最后一个 gate, 确保路径自然闭合
    gates.append(gates[0])

    # 起点: 第一个 Gate 的中点
    start_pose = pnc.Pose()
    start_pose.x = (gates[0][0].x + gates[0][1].x) * 0.5
    start_pose.y = (gates[0][0].y + gates[0][1].y) * 0.5
    start_pose.theta = 0.0

    raw_path = plan_through_gates(grid, grid_meta, start_pose, gates)

    if len(raw_path) < 3:
        print("  ❌ Hybrid A* 规划失败, 退回中心线")
        raw_path = [(x, y, 0.0) for x, y in loop_pts]

    # Step 6: Safe Corridor + B-Spline
    print("[6/7] Smoothing with SafeCorridor + BSpline ...")
    smoothed = smooth_path(raw_path, outer, holes)

    # 转换为 Trajectory
    pts = np.array([(p[0], p[1]) for p in smoothed])
    traj = Trajectory(pts)
    # 保存轨迹点供动画使用
    np.save("output/sim_trajectory_optimization_traj.npy", pts)
    print(f"  Optimized trajectory: {traj.total_len:.1f} m, {len(pts)} pts")
    print(f"  Traj diag: max|kappa|={np.max(np.abs(traj.kappa)):.4f}, "
          f"min seg={np.min(np.sqrt(np.sum(np.diff(pts, axis=0)**2, axis=1))):.4f}m, "
          f"start-end dist={np.linalg.norm(pts[0]-pts[-1]):.4f}m")

    # Step 7: MPC 仿真
    print(f"\n[7/7] MPC simulation ({VX:.0f} m/s, dt={DT}s) ...")
    A_c, B1_c, B2_c = build_cont_matrices()
    A_d = np.eye(4) + A_c * DT
    B1_d = B1_c * DT

    eig = np.abs(np.linalg.eigvals(A_d))
    print(f"  |eig(A_d)| = {[f'{e:.4f}' for e in sorted(eig, reverse=True)]}")

    C_mat = np.eye(4)
    D_mat = np.zeros((4, 1))
    model = pnc.BicycleModel(A_c, B1_c, B2_c, C_mat, D_mat)

    init_e_y = -0.3
    init_e_psi = 0.05
    init_state = np.array([init_e_y, 0.0, init_e_psi, 0.0])
    P_kf = np.eye(4) * 1.0
    Q_kf = np.eye(4) * 0.01
    R_kf = np.diag([0.1, 0.1, 0.025, 0.005])
    model.kf.init(A_d, B1_d, C_mat, P_kf, Q_kf, R_kf, init_state)
    model.init(init_state)

    Q = np.diag([80.0, 0.5, 15.0, 0.5])
    R = np.array([[0.1]])
    S_term = np.eye(4) * 1.0
    mpc = pnc.MPC()
    mpc.init(A_d, B1_d, C_mat, Q, R, S_term, N_HORIZON)

    target = np.zeros(4)
    N_STEPS = int(traj.total_len / (VX * DT))
    N_STEPS = min(N_STEPS, 3000)
    print(f"  Simulating {N_STEPS} steps = {N_STEPS*DT:.1f}s = {N_STEPS*VX*DT:.1f}m")

    log = []
    steer = 0.0

    for step in range(N_STEPS):
        t = step * DT
        s_travel = t * VX
        ref = traj.get_state(s_travel)
        kappa_cur = float(ref[3])

        model.kf.update(model.y, np.array([steer]))
        u_fb = mpc.predict(target, model.kf.x_post)
        steer_ff = feedforward(kappa_cur)
        steer = float(u_fb[0] + steer_ff)
        steer = max(-MAX_STEER, min(MAX_STEER, steer))

        model.step(DT, kappa_cur * VX, np.array([steer]))

        log.append((step, t, float(model.x[0]), float(model.x[1]),
                    float(model.x[2]), float(model.x[3]), steer))

        if step % 200 == 0 or step == N_STEPS - 1:
            print(f"  {step:4d}  e_y={model.x[0]:+.4f}m  "
                  f"e_psi={math.degrees(model.x[2]):+.2f}deg  "
                  f"str={math.degrees(steer):+.1f}deg")

    # 保存文本输出
    os.makedirs("output", exist_ok=True)
    out_txt = "output/sim_trajectory_optimization.txt"
    with open(out_txt, "w") as f:
        f.write(f"# REF: type=optimized len={traj.total_len:.1f}m "
                f"dt={DT} Vx={VX}\n")
        f.write("Step\ttime\te_y\tde_y\te_psi\tde_psi\tsteer\n")
        for row in log:
            f.write(f"{int(row[0])}\t" +
                    "\t".join(f"{v:.6f}" for v in row[1:]) + "\n")

    final = log[-1]
    print(f"\n[OK] {out_txt}")
    print(f"  Completed 1 lap ({traj.total_len:.0f}m)")
    print(f"  Final: e_y={final[2]:.4f}m  e_psi={math.degrees(final[4]):.2f}deg  "
          f"s={N_STEPS*VX*DT:.0f}m / {traj.total_len:.0f}m")

    # 可视化
    visualize(log, traj, raw_path, smoothed, outer, holes, grid_meta)


# =====================================================================
#  Visualization
# =====================================================================

def visualize(log, traj, raw_path, smoothed, outer, holes, grid_meta):
    arr = np.array(log)
    N = len(arr)
    ts  = arr[:, 1]
    ey  = arr[:, 2]
    ep  = arr[:, 4]
    st  = arr[:, 6]

    # 重构参考和车辆位置
    x_ref  = np.zeros(N); y_ref = np.zeros(N); psi_ref = np.zeros(N)
    car_x  = np.zeros(N); car_y = np.zeros(N)
    for i in range(N):
        rs = traj.get_state(i * VX * DT)
        x_ref[i], y_ref[i], psi_ref[i] = rs[0], rs[1], rs[2]
        car_x[i] = x_ref[i] - ey[i] * math.sin(psi_ref[i])
        car_y[i] = y_ref[i] + ey[i] * math.cos(psi_ref[i])

    llx = x_ref - LANE_WIDTH/2 * np.sin(psi_ref)
    lly = y_ref + LANE_WIDTH/2 * np.cos(psi_ref)
    lrx = x_ref + LANE_WIDTH/2 * np.sin(psi_ref)
    lry = y_ref - LANE_WIDTH/2 * np.cos(psi_ref)

    fig = plt.figure(figsize=(22, 16))
    fig.suptitle("Trajectory Optimization — Hybrid A* + B-Spline + MPC", fontsize=16)

    # --- 主图: 轨迹鸟瞰 ---
    ax = fig.add_subplot(2, 3, (1, 4))
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_title("Track, Centerline & Optimized Path")

    # 赛道边界
    ax.plot(outer[:,0], outer[:,1], "k-", lw=1.5, alpha=0.5, label="Track boundary")
    for i, h in enumerate(holes):
        ax.fill(h[:,0], h[:,1], fc="white", ec="k", lw=0.8, alpha=0.95,
                label=f"Hole {i}" if i == 0 else "")

    # HA* 原始路径
    if raw_path:
        rx = [p[0] for p in raw_path]
        ry = [p[1] for p in raw_path]
        ax.plot(rx, ry, "g--", lw=1.0, alpha=0.6, label="Hybrid A* raw")

    # B-Spline 平滑路径
    sx = [p[0] for p in smoothed]
    sy = [p[1] for p in smoothed]
    ax.plot(sx, sy, "b-", lw=2.0, alpha=0.9, label="B-Spline smoothed")

    # 车道边界
    ax.plot(llx, lly, "gray", lw=0.5, ls=":", alpha=0.5)
    ax.plot(lrx, lry, "gray", lw=0.5, ls=":", alpha=0.5)

    # 车辆轨迹
    skip = max(1, N // 500)
    ax.plot(car_x[::skip], car_y[::skip], "r-", lw=1.5, alpha=0.85,
            label="Vehicle path")
    ax.plot(car_x[0], car_y[0], "go", ms=10, label="Start")
    ax.plot(car_x[-1], car_y[-1], "mo", ms=10, label="End")
    for i in range(0, N, max(1, N//6)):
        ax.plot(car_x[i], car_y[i], "r.", ms=6)
        hdg = psi_ref[i] + ep[i]
        ax.arrow(car_x[i], car_y[i], math.cos(hdg)*2, math.sin(hdg)*2,
                 head_width=1.0, fc="r", ec="r", alpha=0.5)
    ax.legend(loc="upper right", fontsize=8)

    # --- 曲率 ---
    ax = fig.add_subplot(2, 3, 2)
    kr = np.array([traj.get_state(i*VX*DT)[3] for i in range(N)])
    ax.plot(ts, kr, "b-", lw=1.2, alpha=0.7, label="Ref kappa")
    ax.plot(ts, np.degrees([feedforward(k) for k in kr]),
            "orange", lw=1.0, alpha=0.7, label="FF steer (deg)")
    ax.set_title("Curvature & Feedforward")
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # --- 横向误差 ---
    ax = fig.add_subplot(2, 3, 3)
    ax.plot(ts, ey, "b-", lw=1.5)
    ax.axhline(0, color="k", ls="--", lw=0.5)
    ax.set_ylabel("e_y (m)")
    ax.set_title("Lateral Error")
    ax.grid(True, alpha=0.3)

    # --- 航向误差 ---
    ax = fig.add_subplot(2, 3, 5)
    ax.plot(ts, np.degrees(ep), "r-", lw=1.5)
    ax.axhline(0, color="k", ls="--", lw=0.5)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("e_psi (deg)")
    ax.set_title("Heading Error")
    ax.grid(True, alpha=0.3)

    # --- 转向 ---
    ax = fig.add_subplot(2, 3, 6)
    ax.plot(ts, np.degrees(st), "g-", lw=1.5)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Steer (deg)")
    ax.set_title("Steering Angle")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    # 统计
    skip_w = max(1, N//10)
    ey_rms = float(np.sqrt(np.mean(ey[skip_w:]**2)))
    ep_rms = float(np.sqrt(np.mean(ep[skip_w:]**2)))
    print(f"  RMS (after warmup): e_y={ey_rms:.4f}m  "
          f"e_psi={math.degrees(ep_rms):.2f}deg")

    out_png = "output/sim_trajectory_optimization.png"
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"  Plot: {out_png}")
    plt.show()


if __name__ == "__main__":
    run()
