"""
sim_dynamic_obstacles_lateral.py — 动态障碍物场景 (横向偏移避障)

管线: path2.png → map_parser → centerline → circuit
      → occupancy grid → HA* (全局) → SafeCorridor → BSpline
      → MPC + 横向偏移避障 (势场法)

NPC 沿中心线匀速行驶，自车通过 MPC target 横向偏移完成超越。
不修改参考轨迹，不动 MPC/KF 状态。

用法: python pipeline/sim_dynamic_obstacles_lateral.py
"""

from __future__ import annotations

import os, sys, math
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.path import Path as MplPath
from matplotlib.patches import Polygon as MplPolygon, PathPatch

_self_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.dirname(_self_dir)
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

sys.path.insert(0, os.path.join(_self_dir, "..", "build", "pnc"))
import pnc

from pipeline.dynamic_obstacles import NpcVehicle, NpcManager

# ========================== 参数 ==========================

MASS     = 1573.0; IZ = 2873.0; LF = 1.1; LR = 1.58
L_WB     = LF + LR; C_AF = 80000.0; C_AR = 80000.0
VX       = 10.0; DT = 0.1; N_HORIZON = 40
LANE_WIDTH = 3.5; MAX_STEER = math.radians(30.0)

CELL_SIZE      = 0.2; GATE_SPACING = 15.0; HA_ARC_LENGTH = 0.6
VEHICLE_HW     = 1.0; VEHICLE_FWD = 1.5; VEHICLE_REV = 1.0
SAFETY_MARGIN  = VEHICLE_HW + 0.2
COLLISION_MARGIN = VEHICLE_HW + 0.2
CORRIDOR_MARGIN  = COLLISION_MARGIN

BSPLINE_DEGREE = 3; BSPLINE_NUM_CTRL = 100; BSPLINE_RESAMPLE = 0.5

NPC_SPEED = 5.0; NPC_HW = 1.0; NPC_FWD = 1.5; NPC_REV = 1.0
NPC_1_START = 0.25; NPC_2_START = 0.40

# 横向偏移避障参数
# 状态机避障参数
TRIGGER_DIST = 50.0    # NPC 进入此弧长距离触发超车 (m)
RAMP_LENGTH  = 30.0    # 偏移上升/下降的弧长距离 (m)
MAX_OFFSET   = 2.3     # 保持的横向偏移 (m)
PASSED_DIST  = 15.0    # NPC 超过自车此距离后开始归中 (m)


# =====================================================================
#  占用栅格构建 (复用)
# =====================================================================

def _polygon_scanline_intersections(y, poly):
    xs = []; n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]; x2, y2 = poly[(i+1)%n]
        if (y1 <= y < y2) or (y2 <= y < y1):
            if abs(y2-y1) > 1e-12:
                xs.append(x1 + (y-y1)*(x2-x1)/(y2-y1))
    xs.sort(); return xs

def build_occupancy_grid(outer, holes, cell_size, safety_margin):
    all_pts = np.vstack([outer] + list(holes))
    x_min, y_min = np.min(all_pts, axis=0)
    x_max, y_max = np.max(all_pts, axis=0)
    pad = safety_margin + 1.0
    x_min -= pad; y_min -= pad; x_max += pad; y_max += pad
    cols = int((x_max-x_min)/cell_size) + 1
    rows = int((y_max-y_min)/cell_size) + 1
    print(f"  Grid: {rows}×{cols} cells, {(x_max-x_min):.0f}×{(y_max-y_min):.0f} m")

    outer_list = outer.tolist(); holes_list = [h.tolist() for h in holes]
    grid = [[0]*cols for _ in range(rows)]
    for r in range(rows):
        y = y_min + r*cell_size + cell_size/2.0
        for k in range(0, len(_polygon_scanline_intersections(y, outer_list))-1, 2):
            xl, xr = _polygon_scanline_intersections(y, outer_list)[k], _polygon_scanline_intersections(y, outer_list)[k+1]
            cl = max(0, int((xl-x_min)/cell_size)); cr = min(cols-1, int((xr-x_min)/cell_size))
            for c in range(cl, cr+1): grid[r][c] = 1
        for hole in holes_list:
            hxs = _polygon_scanline_intersections(y, hole)
            for k in range(0, len(hxs)-1, 2):
                cl = max(0, int((hxs[k]-x_min)/cell_size)); cr = min(cols-1, int((hxs[k+1]-x_min)/cell_size))
                for c in range(cl, cr+1): grid[r][c] = 0
        for c in range(cols): grid[r][c] = 1 - grid[r][c]

    dilate_radius = int(safety_margin/cell_size)
    if dilate_radius > 0: pnc.dilate_grid(grid, dilate_radius)

    return grid, {"x_min":x_min,"y_min":y_min,"x_max":x_max,"y_max":y_max,
                  "cols":cols,"rows":rows,"cell_size":cell_size}


# =====================================================================
#  Gate 生成 + HA* Gate 规划 (复用)
# =====================================================================

def generate_gates(centerline_pts, spacing_m, lane_width):
    n = len(centerline_pts)
    diffs = np.diff(centerline_pts, axis=0)
    cum_s = np.concatenate([[0.0], np.cumsum(np.sqrt(np.sum(diffs**2, axis=1)))])
    total_len = float(cum_s[-1])
    tangents = np.zeros_like(centerline_pts)
    for i in range(n):
        pi, ni = (i-1)%n, (i+1)%n
        dx = centerline_pts[ni,0]-centerline_pts[pi,0]; dy = centerline_pts[ni,1]-centerline_pts[pi,1]
        norm = np.sqrt(dx*dx+dy*dy)
        if norm > 1e-9: tangents[i,0]=dx/norm; tangents[i,1]=dy/norm
    gates = []; half_w = lane_width/2.0
    for i in range(max(2, int(total_len/spacing_m))):
        s = total_len*i/max(2, int(total_len/spacing_m))
        idx = min(np.searchsorted(cum_s, s), n-1)
        if idx == 0: pt = centerline_pts[0]; tx,ty = tangents[0]
        else:
            s0,s1 = cum_s[idx-1],cum_s[idx]; t = (s-s0)/(s1-s0) if s1>s0 else 0.0
            pt = (1-t)*centerline_pts[idx-1]+t*centerline_pts[idx]
            tx = (1-t)*tangents[idx-1,0]+t*tangents[idx,0]
            ty = (1-t)*tangents[idx-1,1]+t*tangents[idx,1]
            tn = np.sqrt(tx*tx+ty*ty)
            if tn>1e-9: tx/=tn; ty/=tn
        nx,ny = -ty,tx
        ga = pnc.Vec2d(); ga.x=pt[0]+nx*half_w; ga.y=pt[1]+ny*half_w
        gb = pnc.Vec2d(); gb.x=pt[0]-nx*half_w; gb.y=pt[1]-ny*half_w
        gates.append((ga,gb))
    return gates

def plan_through_gates(grid, grid_meta, start_pose, gates):
    ha = pnc.HybridAStar(grid)
    ha.set_cell_size(CELL_SIZE); ha.set_wheelbase(L_WB)
    ha.set_max_steer(MAX_STEER); ha.set_num_steer(5)
    ha.set_arc_length(HA_ARC_LENGTH)
    ha.set_goal_xy_tol(1.0); ha.set_goal_th_tol(1.0)
    ha.set_vehicle_dims(COLLISION_MARGIN, VEHICLE_FWD+0.2, VEHICLE_REV+0.2)
    ha.set_grid_origin(grid_meta["x_min"], grid_meta["y_min"])
    full_path = []; current = start_pose; cf = 0
    for gi, (ga,gb) in enumerate(gates):
        seg = []
        if gi==0:
            m=pnc.Pose(); m.x=(ga.x+gb.x)*0.5; m.y=(ga.y+gb.y)*0.5; m.theta=0.0
            try: seg = ha.plan(current,m)
            except: pass
        else:
            try: seg = ha.plan_to_gate(current,ga,gb)
            except: pass
        if not seg:
            if gi>0:
                ha.set_goal_xy_tol(3.0)
                try: seg = ha.plan_to_gate(current,ga,gb)
                except: pass
                ha.set_goal_xy_tol(1.0)
            if not seg: cf+=1
            if cf>=3: break
            continue
        cf=0
        if full_path and seg:
            if math.hypot(full_path[-1][0]-seg[0].x, full_path[-1][1]-seg[0].y)<0.1: seg=seg[1:]
        for p in seg: full_path.append((p.x,p.y,p.theta))
        current = pnc.Pose(); current.x=full_path[-1][0]; current.y=full_path[-1][1]; current.theta=full_path[-1][2]
        if (gi+1)%5==0 or gi==len(gates)-1: print(f"  Gate {gi+1}/{len(gates)}: {len(full_path)} pts")
    print(f"  HA*: {len(full_path)} pts")
    return full_path


# =====================================================================
#  Safe Corridor + B-Spline (复用)
# =====================================================================

def smooth_path(raw_path, grid, grid_meta):
    ref_path = [pnc.Pose() for _ in raw_path]
    for i,(x,y,th) in enumerate(raw_path): ref_path[i].x=x; ref_path[i].y=y; ref_path[i].theta=th
    sc = pnc.SafeCorridor()
    sc.set_margin(CORRIDOR_MARGIN); sc.set_sample_interval(2.0)
    sc.set_vehicle_half_width(COLLISION_MARGIN)
    corridors = sc.build(ref_path, grid, grid_meta["x_min"], grid_meta["y_min"],
                          grid_meta["cell_size"], grid_meta["cols"], grid_meta["rows"])
    print(f"  SafeCorridor: {len(corridors)} sections")
    bs = pnc.BSpline(); p = pnc.BSplineParams()
    p.degree=BSPLINE_DEGREE; p.num_control_points=BSPLINE_NUM_CTRL
    p.closed=False; p.resample_spacing=BSPLINE_RESAMPLE
    bs.set_params(p)
    fitted = bs.fit(ref_path, corridors); resampled = bs.resample(fitted)
    print(f"  B-Spline: {len(fitted)}→{len(resampled)} pts")
    return [(pp.x,pp.y,pp.theta) for pp in resampled], corridors


# =====================================================================
#  Trajectory
# =====================================================================

class Trajectory:
    def __init__(self, points: np.ndarray):
        self.points = points; n = len(points)
        d = np.diff(points, axis=0)
        self.cum_s = np.concatenate([[0.0], np.cumsum(np.sqrt(np.sum(d**2, axis=1)))])
        self.total_len = float(self.cum_s[-1])
        self.psi = np.zeros(n)
        if n>=2:
            self.psi[0] = math.atan2(points[1,1]-points[0,1], points[1,0]-points[0,0])
            self.psi[-1]= math.atan2(points[-1,1]-points[-2,1], points[-1,0]-points[-2,0])
        for i in range(1,n-1):
            self.psi[i] = math.atan2(points[i+1,1]-points[i-1,1], points[i+1,0]-points[i-1,0])
        self.kappa = np.zeros(n)
        for i in range(1,n-1):
            dx=points[i+1,0]-points[i-1,0]; dy=points[i+1,1]-points[i-1,1]
            ddx=points[i+1,0]-2*points[i,0]+points[i-1,0]; ddy=points[i+1,1]-2*points[i,1]+points[i-1,1]
            den=(dx*dx+dy*dy)**1.5
            self.kappa[i]=(dx*ddy-dy*ddx)/den if den>1e-8 else 0.0
        from scipy.ndimage import uniform_filter1d
        ks = min(21, max(3, n-1))
        if ks%2==0: ks-=1
        self.kappa = uniform_filter1d(self.kappa, size=ks, mode='nearest')

    def get_state(self, s):
        sm = s % self.total_len if self.total_len>0 else s
        idx = min(np.searchsorted(self.cum_s, sm), len(self.cum_s)-1)
        if idx==0: return np.array([self.points[0,0],self.points[0,1],self.psi[0],self.kappa[0]])
        s0,s1 = self.cum_s[idx-1],self.cum_s[idx]; t = (sm-s0)/(s1-s0) if s1>s0 else 0.0
        x = (1-t)*self.points[idx-1,0]+t*self.points[idx,0]
        y = (1-t)*self.points[idx-1,1]+t*self.points[idx,1]
        d = self.psi[idx]-self.psi[idx-1]; d = (d+math.pi)%(2*math.pi)-math.pi
        return np.array([x, y, self.psi[idx-1]+t*d, (1-t)*self.kappa[idx-1]+t*self.kappa[idx]])


# =====================================================================
#  MPC 矩阵 & 工具
# =====================================================================

def build_cont_matrices():
    A = np.array([[0,1,0,0],
        [0,-(2*C_AF+2*C_AR)/(MASS*VX),(2*C_AF+2*C_AR)/MASS,-(2*C_AF*LF-2*C_AR*LR)/(MASS*VX)],
        [0,0,0,1],
        [0,-(2*C_AF*LF-2*C_AR*LR)/(IZ*VX),(2*C_AF*LF-2*C_AR*LR)/IZ,-(2*C_AF*LF*LF+2*C_AR*LR*LR)/(IZ*VX)]])
    B1 = np.array([[0],[2*C_AF/MASS],[0],[2*C_AF*LF/IZ]])
    B2 = np.array([[0],[-(2*C_AF*LF-2*C_AR*LR)/(MASS*VX)-VX],[0],[-(2*C_AF*LF*LF+2*C_AR*LR*LR)/(IZ*VX)]])
    return A,B1,B2

def feedforward(kappa):
    L = L_WB
    return L*kappa + (LR/(L*C_AF)-LF/(L*C_AR))*MASS/2*VX*VX*kappa

def get_ego_world_pose(ref, e_y, e_psi):
    x = ref[0] - e_y*math.sin(ref[2]); y = ref[1] + e_y*math.cos(ref[2])
    return x, y, ref[2]+e_psi

def _vehicle_corners_world(cx, cy, h, hw, hf, hr):
    c=math.cos(h); s=math.sin(h)
    corners = []
    for lx,ly in [(hf,hw),(hf,-hw),(-hr,-hw),(-hr,hw)]:
        corners.append((cx+lx*c-ly*s, cy+lx*s+ly*c))
    return corners

def check_ego_in_bounds(ego_x, ego_y, ego_h, outer_poly, mpl_path):
    for cx,cy in _vehicle_corners_world(ego_x,ego_y,ego_h,VEHICLE_HW,VEHICLE_FWD,VEHICLE_REV):
        if not mpl_path.contains_point((cx,cy)): return False
    return True

def find_centerline_s(x, y, pts, cum_s, prev_s=None):
    """中心线弧长反查. 若提供 prev_s, 在 ±5m 窗口内局部搜索, 避免分支跳变."""
    dx = pts[:,0]-x; dy = pts[:,1]-y
    if prev_s is not None:
        total = float(cum_s[-1])
        lo = max(0.0, prev_s - 5.0)
        hi = min(total, prev_s + 5.0)
        i0 = int(np.searchsorted(cum_s, lo))
        i1 = int(np.searchsorted(cum_s, hi)) + 1
        i0 = max(0, i0); i1 = min(len(pts), i1)
        if i1 > i0 + 1:
            local = dx[i0:i1]**2 + dy[i0:i1]**2
            return float(cum_s[i0 + int(np.argmin(local))])
    return float(cum_s[int(np.argmin(dx*dx+dy*dy))])

def interpolate_centerline_heading(x, y, pts, cum_s):
    s = find_centerline_s(x, y, pts, cum_s)
    n = len(pts); total = float(cum_s[-1]); s = s%total
    idx = min(int(np.searchsorted(cum_s, s)), n-1)
    if idx==0: return math.atan2(pts[1,1]-pts[0,1], pts[1,0]-pts[0,0])
    hp = math.atan2(pts[idx,1]-pts[idx-1,1], pts[idx,0]-pts[idx-1,0])
    if idx<n-1:
        hn = math.atan2(pts[idx+1,1]-pts[idx,1], pts[idx+1,0]-pts[idx,0])
        s0,s1 = cum_s[idx-1],cum_s[idx]; t = (s-s0)/(s1-s0) if s1>s0 else 0.0
        d = hn-hp; d = (d+math.pi)%(2*math.pi)-math.pi
        return hp+t*d
    return hp

def _interpolate_cl(s, pts, cum_s):
    """中心线弧长插值 → (x, y, heading)."""
    n = len(pts); total = float(cum_s[-1]); s = s % total
    idx = min(int(np.searchsorted(cum_s, s)), n - 1)
    if idx == 0: return float(pts[0,0]), float(pts[0,1]), 0.0
    s0, s1 = cum_s[idx-1], cum_s[idx]
    t = (s-s0)/(s1-s0) if s1>s0 else 0.0; t = max(0.0, min(1.0, t))
    x = (1-t)*float(pts[idx-1,0]) + t*float(pts[idx,0])
    y = (1-t)*float(pts[idx-1,1]) + t*float(pts[idx,1])
    h = math.atan2(pts[idx,1]-pts[idx-1,1], pts[idx,0]-pts[idx-1,0])
    return x, y, h


def _lateral_clearance(x, y, psi, sign, outer_poly):
    """沿 sign 方向 (+1=左, -1=右) 探测到道路边界的距离."""
    nx = -math.sin(psi) * sign  # 左/右法向
    ny = math.cos(psi) * sign
    for d in np.linspace(0.5, 20.0, 40):
        px = x + nx*d; py = y + ny*d
        if not _point_in_poly(px, py, outer_poly):
            return d - 0.5
    return 20.0


def _point_in_poly(x, y, poly):
    """射线法判断点是否在多边形内."""
    inside = False; n = len(poly)
    j = n-1
    for i in range(n):
        if ((poly[i,1] > y) != (poly[j,1] > y)) and \
           (x < (poly[j,0]-poly[i,0])*(y-poly[i,1])/(poly[j,1]-poly[i,1])+poly[i,0]):
            inside = not inside
        j = i
    return inside


# =====================================================================
#  Main simulation
# =====================================================================

def run():
    print("="*60)
    print("  动态障碍物场景 — 横向偏移避障 + MPC")
    print("="*60)

    from pipeline.map_parser import parse_map
    from pipeline.centerline import extract_centerline_graph
    from pipeline.sim_lane_keeping_real import assemble_go_straight_circuit

    # [1] 解析地图
    img = os.path.join(_project_root, "map", "path2.png")
    print(f"\n[1/6] 解析地图: {img}")
    bounds = parse_map(img, pixels_per_meter=12.8, smoothing_factor=0.0,
                       num_control_points=200, resample_spacing_m=0.1, has_starting_line=True)
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

    # [3] 全局 HA* + B-Spline → 全局参考轨迹
    print("[3/6] 全局 HA* + B-Spline ...")
    base_grid, grid_meta = build_occupancy_grid(outer, map_holes, CELL_SIZE, SAFETY_MARGIN)
    gates = generate_gates(loop_pts, GATE_SPACING, LANE_WIDTH)
    print(f"  Gates: {len(gates)}"); gates.append(gates[0])
    start_pose = pnc.Pose()
    start_pose.x = (gates[0][0].x+gates[0][1].x)*0.5
    start_pose.y = (gates[0][0].y+gates[0][1].y)*0.5
    gx = gates[0][0].x-gates[0][1].x; gy = gates[0][0].y-gates[0][1].y
    start_pose.theta = math.atan2(-gx, gy)
    raw_path = plan_through_gates(base_grid, grid_meta, start_pose, gates)
    if len(raw_path) < 3:
        print("  ❌ HA* 失败, 退回中心线")
        raw_path = [(x,y,0.0) for x,y in loop_pts]
    smoothed, corridors = smooth_path(raw_path, base_grid, grid_meta)
    pts = np.array([(p[0],p[1]) for p in smoothed])
    traj = Trajectory(pts)
    print(f"  参考轨迹: {traj.total_len:.1f} m, {len(pts)} pts, max|kappa|={np.max(np.abs(traj.kappa)):.4f}")

    # [4] 初始化 NPC
    print("\n[4/6] NPC ...")
    npc_manager = NpcManager()
    npc_manager.add_npc(NpcVehicle(NPC_1_START, NPC_SPEED, NPC_HW, NPC_FWD, NPC_REV))
    npc_manager.add_npc(NpcVehicle(NPC_2_START, NPC_SPEED, NPC_HW, NPC_FWD, NPC_REV))
    print(f"  NPC 1: {NPC_1_START*100:.0f}%, NPC 2: {NPC_2_START*100:.0f}%, v={NPC_SPEED}m/s")

    # [5] MPC 仿真 + 横向偏移
    print(f"\n[5/6] MPC 仿真 (ego={VX:.0f}m/s, npc={NPC_SPEED:.0f}m/s) ...")
    A_c,B1_c,B2_c = build_cont_matrices()
    A_d = np.eye(4)+A_c*DT; B1_d = B1_c*DT
    print(f"  |eig(A_d)| = {[f'{e:.4f}' for e in sorted(np.abs(np.linalg.eigvals(A_d)), reverse=True)]}")
    C_mat = np.eye(4); D_mat = np.zeros((4,1))
    model = pnc.BicycleModel(A_c,B1_c,B2_c,C_mat,D_mat)
    init_state = np.array([-0.3,0.0,0.05,0.0])
    P_kf = np.eye(4)*1.0; Q_kf = np.eye(4)*0.01; R_kf = np.diag([0.1,0.1,0.025,0.005])
    model.kf.init(A_d,B1_d,C_mat,P_kf,Q_kf,R_kf,init_state)
    model.init(init_state)
    Qw = np.diag([120.0,0.5,15.0,0.5]); Rw = np.array([[0.1]]); S_term = np.eye(4)*1.0
    mpc = pnc.MPC(); mpc.init(A_d,B1_d,C_mat,Qw,Rw,S_term,N_HORIZON)

    N_STEPS = min(int(cl_total_len/(VX*DT))+200, 5000)
    print(f"  Max {N_STEPS} steps = {N_STEPS*DT:.1f}s")
    mpl_outer = MplPath(outer)

    log, npc_log, offset_log = [], [], []
    steer = 0.0; local_s = 0.0
    ego_prev_s = None
    collision_npc = False; collision_boundary = False

    # 状态机变量
    overtake_state = 'IDLE'      # IDLE | RAMPING_UP | HOLDING | RAMPING_DOWN
    overtake_dir   = 0           # +1 左超, -1 右超
    overtake_npc   = None        # 触发当前超车的 NPC 引用
    overtake_ramp_start_s = 0.0
    overtake_ramp_down_start_s = 0.0
    total_overtakes = 0

    for step in range(N_STEPS):
        t = step*DT; s_travel = t*VX

        npc_manager.update(t, loop_pts, cl_cum_s, cl_total_len)
        npc_log.append(npc_manager.get_positions())

        ref = traj.get_state(local_s)
        kappa_cur = float(ref[3])
        e_y = float(model.x[0]); e_psi = float(model.x[2])
        ego_x, ego_y, ego_heading = get_ego_world_pose(ref, e_y, e_psi)

        # ego 弧长 (局部追踪)
        ego_s = find_centerline_s(ego_x, ego_y, loop_pts, cl_cum_s, ego_prev_s)
        ego_prev_s = ego_s

        # ---- 状态机: 超车决策与 offset 曲线 ----
        lat_offset = 0.0

        # 找到最近的 NPC
        nearest_npc = None; nearest_lon = float('inf')
        total = float(cl_cum_s[-1])
        for npc_v in npc_manager.npcs:
            prev_s = getattr(npc_v, 'prev_obs_s', None)
            npc_v_s = find_centerline_s(npc_v.x, npc_v.y, loop_pts, cl_cum_s, prev_s)
            npc_v.prev_obs_s = npc_v_s
            lon = (npc_v_s - ego_s) % total
            if lon > total/2: lon -= total
            if 0 < lon < nearest_lon:
                nearest_lon = lon
                nearest_npc = npc_v

        if overtake_state == 'IDLE':
            if nearest_npc is not None and nearest_lon < TRIGGER_DIST:
                cl_psi = interpolate_centerline_heading(ego_x, ego_y, loop_pts, cl_cum_s)
                room_l = _lateral_clearance(ego_x, ego_y, cl_psi, +1, outer)
                room_r = _lateral_clearance(ego_x, ego_y, cl_psi, -1, outer)
                # 优先左超, 左边空间不够才用右
                overtake_dir = 1 if room_l >= MAX_OFFSET else -1
                overtake_npc = nearest_npc   # 记录触发 NPC
                overtake_ramp_start_s = ego_s
                overtake_state = 'RAMPING_UP'
                total_overtakes += 1
                print(f"  [{step:4d}] 超车 #{total_overtakes} 开始: dir={'左' if overtake_dir>0 else '右'}, "
                      f"npc_lon={nearest_lon:.0f}m room_l={room_l:.1f}m room_r={room_r:.1f}m")

        elif overtake_state == 'RAMPING_UP':
            progress = (ego_s - overtake_ramp_start_s) % total
            if progress > total/2: progress -= total
            lat_offset = overtake_dir * MAX_OFFSET * min(1.0, progress / RAMP_LENGTH)
            if progress >= RAMP_LENGTH:
                overtake_state = 'HOLDING'
                print(f"  [{step:4d}] 超车 #{total_overtakes} 保持偏移 {overtake_dir*MAX_OFFSET:+.1f}m")

        elif overtake_state == 'HOLDING':
            lat_offset = overtake_dir * MAX_OFFSET
            # 只检查触发超车的那台 NPC, 忽略其他
            if overtake_npc is not None:
                npc_s = find_centerline_s(overtake_npc.x, overtake_npc.y,
                                           loop_pts, cl_cum_s,
                                           getattr(overtake_npc, 'prev_obs_s', None))
                lon = (npc_s - ego_s) % total
                if lon > total/2: lon -= total
                if lon < -PASSED_DIST:
                    overtake_ramp_down_start_s = ego_s
                    overtake_state = 'RAMPING_DOWN'
                    print(f"  [{step:4d}] 超车 #{total_overtakes} 开始归中, npc_lon={lon:.0f}m")

        elif overtake_state == 'RAMPING_DOWN':
            progress = (ego_s - overtake_ramp_down_start_s) % total
            if progress > total/2: progress -= total
            lat_offset = overtake_dir * MAX_OFFSET * max(0.0, 1.0 - progress / RAMP_LENGTH)
            if progress >= RAMP_LENGTH:
                lat_offset = 0.0
                overtake_state = 'IDLE'
                overtake_dir = 0
                overtake_npc = None
                print(f"  [{step:4d}] 超车 #{total_overtakes} 完成")

        # 运行时钳位: 不超出当前实际路宽
        if abs(lat_offset) > 0.01:
            cl_psi = interpolate_centerline_heading(ego_x, ego_y, loop_pts, cl_cum_s)
            sign = 1 if lat_offset > 0 else -1
            room = _lateral_clearance(ego_x, ego_y, cl_psi, sign, outer)
            lat_offset = math.copysign(min(abs(lat_offset), room * 0.8), lat_offset)

        # MPC: target[0] = 期望横向偏移
        mpc_target = np.array([float(lat_offset), 0.0, 0.0, 0.0])
        model.kf.update(model.y, np.array([steer]))
        u_fb = mpc.predict(mpc_target, model.kf.x_post)
        steer_ff = feedforward(kappa_cur)
        steer = float(u_fb[0]+steer_ff)
        steer = max(-MAX_STEER, min(MAX_STEER, steer))
        model.step(DT, kappa_cur*VX, np.array([steer]))
        local_s += VX*DT

        log.append((step, t, e_y, e_psi, steer, ego_x, ego_y, ego_heading, lat_offset))
        offset_log.append(lat_offset)

        # 碰撞检测
        if npc_manager.check_collision_with_ego(ego_x,ego_y,ego_heading,
                                                  VEHICLE_HW,VEHICLE_FWD,VEHICLE_REV):
            if not collision_npc: print(f"  ❌ [{step:4d}] t={t:.1f}s NPC 碰撞!"); collision_npc = True
        if not check_ego_in_bounds(ego_x,ego_y,ego_heading,outer,mpl_outer):
            if not collision_boundary: print(f"  ❌ [{step:4d}] t={t:.1f}s 超出边界!"); collision_boundary = True

        if step%100 == 0:
            min_d = min(math.hypot(ego_x-n.x,ego_y-n.y) for n in npc_manager.npcs)
            print(f"  {step:4d} t={t:5.1f}s s={s_travel:6.0f}m e_y={e_y:+.3f}m "
                  f"offset={lat_offset:+.1f}m str={math.degrees(steer):+.1f}deg npc_dist={min_d:.0f}m "
                  f"state={overtake_state}")

        if s_travel >= cl_total_len:
            print(f"\n  → 一圈完成! s={s_travel:.1f}m >= {cl_total_len:.1f}m"); break

    # 统计
    final = log[-1]
    print(f"\n[仿真完成]")
    print(f"  步数: {len(log)}, 时间: {final[1]:.1f}s, 距离: {final[1]*VX:.0f}m")
    print(f"  超车次数: {total_overtakes}, 最终状态: {overtake_state}")
    print(f"  max|offset| = {max(abs(o) for o in offset_log):.2f}m")
    print(f"  NPC 碰撞: {'❌' if collision_npc else '✅'}")
    print(f"  边界碰撞: {'❌' if collision_boundary else '✅'}")
    print(f"  Final: e_y={final[2]:.4f}m  e_psi={math.degrees(final[3]):.2f}deg")

    # [6] 保存 & 可视化
    print("\n[6/6] 保存 & 可视化 ...")
    os.makedirs("output", exist_ok=True)
    arr = np.array(log)
    with open("output/sim_dynamic_obstacles_lateral.txt","w") as f:
        f.write(f"# type=lateral len={cl_total_len:.1f}m dt={DT} Vx={VX}\n")
        f.write("Step\ttime\te_y\te_psi\tsteer\tego_x\tego_y\tego_heading\tlat_offset\n")
        for row in log: f.write(f"{int(row[0])}\t"+"\t".join(f"{v:.6f}" for v in row[1:])+"\n")
    np.save("output/sim_dynamic_obstacles_lateral_outer.npy", outer)
    np.save("output/sim_dynamic_obstacles_lateral_traj.npy", traj.points)
    np.save("output/sim_dynamic_obstacles_lateral_log.npy", arr)
    for i,h in enumerate(map_holes): np.save(f"output/sim_dynamic_obstacles_lateral_hole_{i}.npy", h)
    npc_arr = np.array(npc_log)
    np.save("output/sim_dynamic_obstacles_lateral_npc.npy", npc_arr)

    visualize(log, traj, smoothed, outer, map_holes, npc_log, loop_pts,
              corridors, offset_log, collision_npc, collision_boundary)


# =====================================================================
#  Visualization
# =====================================================================

def visualize(log, traj, smoothed, outer, holes, npc_log, centerline_pts,
              corridors, offset_log, collision_npc, collision_boundary):
    arr = np.array(log); N = len(arr)
    ts = arr[:,1]; ey = arr[:,2]; ep = arr[:,3]; st = arr[:,4]
    ego_wx = arr[:,5]; ego_wy = arr[:,6]; ego_wh = arr[:,7]; lat_off = arr[:,8]

    npc_arr = np.array(npc_log)[:N]
    npc1_x,npc1_y = npc_arr[:,0,0],npc_arr[:,0,1]
    npc2_x,npc2_y = npc_arr[:,1,0],npc_arr[:,1,1]

    fig = plt.figure(figsize=(22,16))
    status = "⚠ COLLISION" if (collision_npc or collision_boundary) else "✅ OK"
    fig.suptitle(f"Dynamic Obstacles — Lateral Offset Avoidance + MPC  [{status}]", fontsize=16)

    # 主图
    ax = fig.add_subplot(2,3,(1,4)); ax.set_aspect("equal"); ax.grid(True,alpha=0.3)
    ax.set_title("Track, NPCs & Ego Trajectory")
    ax.plot(outer[:,0],outer[:,1],"k-",lw=1.5,alpha=0.5,label="Track")
    for i,h in enumerate(holes): ax.fill(h[:,0],h[:,1],fc="white",ec="k",lw=0.8,alpha=0.95)

    if corridors:
        clip = PathPatch(MplPath(outer), transform=ax.transData)
        pk = {"clip_path":clip,"clip_on":True}
        lx=[c.left.x for c in corridors]; ly=[c.left.y for c in corridors]
        rx=[c.right.x for c in corridors]; ry=[c.right.y for c in corridors]
        ax.fill(lx+rx[::-1],ly+ry[::-1],fc="cyan",ec="none",alpha=0.1,**pk)

    ax.plot([p[0] for p in smoothed],[p[1] for p in smoothed],"b-",lw=1.5,alpha=0.4,label="Reference")

    skip = max(1,N//300)
    ax.plot(npc1_x[::skip],npc1_y[::skip],"orange",lw=1.5,alpha=0.8,ls="--",label="NPC 1")
    ax.plot(npc2_x[::skip],npc2_y[::skip],"purple",lw=1.5,alpha=0.8,ls="--",label="NPC 2")
    ax.plot(npc1_x[0],npc1_y[0],"o",color="orange",ms=8); ax.plot(npc2_x[0],npc2_y[0],"o",color="purple",ms=8)

    for i in range(0,N,max(1,int(5.0/DT))):
        for c,npx,npy,nph in [("orange",npc1_x[i],npc1_y[i],npc_arr[i,0,2]),
                                ("purple",npc2_x[i],npc2_y[i],npc_arr[i,1,2])]:
            corners = _vehicle_corners_world(npx,npy,nph,NPC_HW,NPC_FWD,NPC_REV)
            ax.add_patch(MplPolygon(corners,closed=True,fc=c,ec="k",alpha=0.3,lw=0.5))

    skip_e = max(1,N//500)
    ax.plot(ego_wx[::skip_e],ego_wy[::skip_e],"r-",lw=1.8,alpha=0.9,label="Ego")
    ax.plot(ego_wx[0],ego_wy[0],"go",ms=10,label="Start")
    ax.plot(ego_wx[-1],ego_wy[-1],"mo",ms=10,label="End")
    for i in range(0,N,max(1,N//8)):
        ax.arrow(ego_wx[i],ego_wy[i],math.cos(ego_wh[i])*2.5,math.sin(ego_wh[i])*2.5,
                 head_width=1.2,fc="r",ec="r",alpha=0.4)
    ax.legend(loc="upper right",fontsize=7,ncol=2)

    # 子图
    ax2 = fig.add_subplot(2,3,2)
    ax2.plot(ts,ey,"b-",lw=1.5,label="e_y"); ax2.plot(ts,lat_off,"orange",lw=1.0,ls="--",label="target")
    ax2.axhline(0,color="k",ls="--",lw=0.5); ax2.set_title("Lateral Error & Offset"); ax2.legend(fontsize=7); ax2.grid(True,alpha=0.3)

    ax3 = fig.add_subplot(2,3,3)
    ax3.plot(ts,np.degrees(ep),"r-",lw=1.5); ax3.axhline(0,color="k",ls="--",lw=0.5)
    ax3.set_title("Heading Error"); ax3.grid(True,alpha=0.3)

    ax4 = fig.add_subplot(2,3,5)
    d1=np.sqrt((ego_wx-npc1_x)**2+(ego_wy-npc1_y)**2); d2=np.sqrt((ego_wx-npc2_x)**2+(ego_wy-npc2_y)**2)
    ax4.plot(ts,d1,"orange",lw=1.0,alpha=0.7,label="NPC 1"); ax4.plot(ts,d2,"purple",lw=1.0,alpha=0.7,label="NPC 2")
    ax4.axhline(2.5,color="red",ls=":",lw=0.8); ax4.set_title("Ego-NPC Distance"); ax4.legend(fontsize=7); ax4.grid(True,alpha=0.3)

    ax5 = fig.add_subplot(2,3,6)
    ax5.plot(ts,np.degrees(st),"g-",lw=1.5); ax5.set_title("Steering Angle"); ax5.grid(True,alpha=0.3)

    plt.tight_layout()

    skip_w = max(1,N//10)
    ey_rms = float(np.sqrt(np.mean(ey[skip_w:]**2)))
    ep_rms = float(np.sqrt(np.mean(ep[skip_w:]**2)))
    min_d1 = float(np.min(d1[skip_w:])); min_d2 = float(np.min(d2[skip_w:]))
    print(f"  RMS: e_y={ey_rms:.4f}m  e_psi={math.degrees(ep_rms):.2f}deg")
    print(f"  Min NPC dist: NPC1={min_d1:.2f}m  NPC2={min_d2:.2f}m")

    out_png = "output/sim_dynamic_obstacles_lateral.png"
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"  Plot: {out_png}")
    plt.show()


if __name__ == "__main__":
    run()
