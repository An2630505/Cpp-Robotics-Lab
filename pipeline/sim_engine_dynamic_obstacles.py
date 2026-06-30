"""
sim_engine_dynamic_obstacles.py — 引擎版动态障碍物避障

管线: path2.png → map_parser → centerline → circuit
      → HA* (全局) → SafeCorridor → BSpline
      → Engine World (赛道边界 + ego + NPC)
      → MPC 控制 + 横向偏移状态机避障

与 sim_dynamic_obstacles_lateral.py 的区别:
  - 物理仿真由 Engine 负责 (BicycleModel, 碰撞检测, 动量守恒)
  - NPC 由 Engine SimpleModel 驱动 (替代 NpcManager)
  - 碰撞检测由 Engine SAT 负责 (替代手动 OBB SAT + 边界检查)
  - Ego 状态从 Engine 读取, 馈入 KF → MPC (替代 pnc.BicycleModel.step)

用法: python pipeline/sim_engine_dynamic_obstacles.py
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

# C++ 模块
sys.path.insert(0, os.path.join(_project_root, "build", "pnc"))
sys.path.insert(0, os.path.join(_project_root, "build", "engine", "physics"))
import pnc
import engine_physics as ep
from engine import World, Agent, Sensor

# ========================== 参数 ==========================

MASS = 1573.0; IZ = 2873.0; LF = 1.1; LR = 1.58
L_WB = LF + LR; C_AF = 80000.0; C_AR = 80000.0
VX = 10.0; DT_MPC = 0.1; N_HORIZON = 40
DT_ENGINE = 0.01; STEPS_PER_MPC = int(DT_MPC / DT_ENGINE)  # 10
LANE_WIDTH = 3.5; MAX_STEER = math.radians(30.0)

CELL_SIZE = 0.2; GATE_SPACING = 15.0; HA_ARC_LENGTH = 0.6
VEHICLE_HW = 1.0; VEHICLE_FWD = 1.5; VEHICLE_REV = 1.0
SAFETY_MARGIN = VEHICLE_HW + 0.2
COLLISION_MARGIN = VEHICLE_HW + 0.2
CORRIDOR_MARGIN = COLLISION_MARGIN

BSPLINE_DEGREE = 3; BSPLINE_NUM_CTRL = 100; BSPLINE_RESAMPLE = 0.5

NPC_SPEED = 5.0; NPC_HW = 1.0; NPC_FWD = 1.5; NPC_REV = 1.0
NPC_1_START = 0.25; NPC_2_START = 0.40

TRIGGER_DIST = 50.0; RAMP_LENGTH = 30.0
MAX_OFFSET = 2.3; PASSED_DIST = 15.0


# =====================================================================
#  地图 → 边界墙体 (引擎静态实体)
# =====================================================================

def _point_in_poly(x, y, poly):
    inside = False; n = len(poly); j = n - 1
    for i in range(n):
        if ((poly[i, 1] > y) != (poly[j, 1] > y)) and \
           (x < (poly[j, 0] - poly[i, 0]) * (y - poly[i, 1]) /
                 (poly[j, 1] - poly[i, 1]) + poly[i, 0]):
            inside = not inside
        j = i
    return inside


def _edge_wall(p0, p1, thickness, outward_dir):
    """沿边 (p0→p1) 生成厚度为 thickness 的凸四边形墙体"""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return None
    nx, ny = -dy / length, dx / length  # 边右法向
    if outward_dir * (nx * (p0[0] - 500) + ny * (p0[1] - 500)) < 0:
        nx, ny = -nx, -ny
    t = thickness
    return ep.Polygon([
        ep.Vec2d(p0[0], p0[1]),
        ep.Vec2d(p1[0], p1[1]),
        ep.Vec2d(p1[0] + nx * t, p1[1] + ny * t),
        ep.Vec2d(p0[0] + nx * t, p0[1] + ny * t),
    ])


def build_boundary_walls(world: World, outer, holes, thickness=0.5,
                         sample_spacing=1.5):
    """将赛道边界分解为引擎静态墙体 (降采样到 ~sample_spacing 间距)"""

    def _subsample(poly, spacing):
        """对多边形降采样, 保留间距 ≈ spacing 的点"""
        if len(poly) <= 4:
            return poly
        pts = np.asarray(poly)
        d = np.diff(pts, axis=0, append=pts[:1])
        cum = np.concatenate([[0.0], np.cumsum(np.hypot(d[:, 0], d[:, 1]))])
        total = cum[-1]
        if total < spacing * 3:
            return poly
        n_target = max(4, int(total / spacing))
        samples = []
        for i in range(n_target):
            s = total * i / n_target
            idx = np.searchsorted(cum, s)
            idx = min(idx, len(pts) - 1)
            if idx == 0:
                samples.append(tuple(pts[0]))
            else:
                s0, s1 = cum[idx - 1], cum[idx]
                t = (s - s0) / (s1 - s0) if s1 > s0 else 0.0
                t = max(0.0, min(1.0, t))
                x = (1 - t) * float(pts[idx - 1, 0]) + t * float(pts[idx, 0])
                y = (1 - t) * float(pts[idx - 1, 1]) + t * float(pts[idx, 1])
                samples.append((x, y))
        return np.array(samples)

    outer_simple = _subsample(outer, sample_spacing)
    print(f"  边界墙体: {len(outer)}→{len(outer_simple)} 外边界, "
          f"{sum(len(h) for h in holes)}→{sum(len(_subsample(h, sample_spacing)) for h in holes)} 孔洞")

    def _add_walls(poly):
        n = len(poly)
        # 判断多边形方向: 用 shoelace 符号
        area2 = 0.0
        for i in range(n):
            x0, y0 = poly[i]; x1, y1 = poly[(i + 1) % n]
            area2 += x0 * y1 - x1 * y0
        is_ccw = area2 > 0  # CCW → 内部在左, 墙体应在右(外侧)

        for i in range(n):
            p0 = poly[i]; p1 = poly[(i + 1) % n]
            dx, dy = p1[0] - p0[0], p1[1] - p0[1]
            length = math.hypot(dx, dy)
            if length < 1e-6:
                continue
            # 右法向 (对 CCW 多边形 = 外侧; 对 CW 多边形 = 内侧/孔洞外)
            nx, ny = -dy / length, dx / length
            t = thickness
            wall = ep.Polygon([
                ep.Vec2d(p0[0], p0[1]),
                ep.Vec2d(p1[0], p1[1]),
                ep.Vec2d(p1[0] + nx * t, p1[1] + ny * t),
                ep.Vec2d(p0[0] + nx * t, p0[1] + ny * t),
            ])
            s = ep.EntityState()
            s.pose = ep.Pose(0, 0, 0)
            s.geometry = wall
            s.is_static = True
            world.add_entity(s, None)

    _add_walls(outer_simple)
    for hole in holes:
        hole_simple = _subsample(hole, sample_spacing)
        _add_walls(hole_simple)


# =====================================================================
#  占用栅格 + HA* + SafeCorridor + BSpline (复用原管线)
# =====================================================================

def _polygon_scanline_intersections(y, poly):
    xs = []; n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]; x2, y2 = poly[(i + 1) % n]
        if (y1 <= y < y2) or (y2 <= y < y1):
            if abs(y2 - y1) > 1e-12:
                xs.append(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
    xs.sort(); return xs


def build_occupancy_grid(outer, holes, cell_size, safety_margin):
    all_pts = np.vstack([outer] + list(holes))
    x_min, y_min = np.min(all_pts, axis=0)
    x_max, y_max = np.max(all_pts, axis=0)
    pad = safety_margin + 1.0
    x_min -= pad; y_min -= pad; x_max += pad; y_max += pad
    cols = int((x_max - x_min) / cell_size) + 1
    rows = int((y_max - y_min) / cell_size) + 1
    print(f"  Grid: {rows}×{cols} cells")

    outer_list = outer.tolist(); holes_list = [h.tolist() for h in holes]
    grid = [[0] * cols for _ in range(rows)]
    for r in range(rows):
        y = y_min + r * cell_size + cell_size / 2.0
        inters = _polygon_scanline_intersections(y, outer_list)
        for k in range(0, len(inters) - 1, 2):
            xl, xr = inters[k], inters[k + 1]
            cl = max(0, int((xl - x_min) / cell_size))
            cr = min(cols - 1, int((xr - x_min) / cell_size))
            for c in range(cl, cr + 1):
                grid[r][c] = 1
        for hole in holes_list:
            hxs = _polygon_scanline_intersections(y, hole)
            for k in range(0, len(hxs) - 1, 2):
                cl = max(0, int((hxs[k] - x_min) / cell_size))
                cr = min(cols - 1, int((hxs[k + 1] - x_min) / cell_size))
                for c in range(cl, cr + 1):
                    grid[r][c] = 0
        for c in range(cols):
            grid[r][c] = 1 - grid[r][c]

    dilate_radius = int(safety_margin / cell_size)
    if dilate_radius > 0:
        pnc.dilate_grid(grid, dilate_radius)
    return grid, {"x_min": x_min, "y_min": y_min, "x_max": x_max, "y_max": y_max,
                  "cols": cols, "rows": rows, "cell_size": cell_size}


def generate_gates(centerline_pts, spacing_m, lane_width):
    n = len(centerline_pts)
    diffs = np.diff(centerline_pts, axis=0)
    cum_s = np.concatenate([[0.0], np.cumsum(np.sqrt(np.sum(diffs ** 2, axis=1)))])
    total_len = float(cum_s[-1])
    tangents = np.zeros_like(centerline_pts)
    for i in range(n):
        pi, ni = (i - 1) % n, (i + 1) % n
        dx = centerline_pts[ni, 0] - centerline_pts[pi, 0]
        dy = centerline_pts[ni, 1] - centerline_pts[pi, 1]
        norm = np.sqrt(dx * dx + dy * dy)
        if norm > 1e-9:
            tangents[i, 0] = dx / norm; tangents[i, 1] = dy / norm
    gates = []; half_w = lane_width / 2.0
    for i in range(max(2, int(total_len / spacing_m))):
        s = total_len * i / max(2, int(total_len / spacing_m))
        idx = min(np.searchsorted(cum_s, s), n - 1)
        if idx == 0:
            pt = centerline_pts[0]; tx, ty = tangents[0]
        else:
            s0, s1 = cum_s[idx - 1], cum_s[idx]
            t = (s - s0) / (s1 - s0) if s1 > s0 else 0.0
            pt = (1 - t) * centerline_pts[idx - 1] + t * centerline_pts[idx]
            tx = (1 - t) * tangents[idx - 1, 0] + t * tangents[idx, 0]
            ty = (1 - t) * tangents[idx - 1, 1] + t * tangents[idx, 1]
            tn = np.sqrt(tx * tx + ty * ty)
            if tn > 1e-9: tx /= tn; ty /= tn
        nx, ny = -ty, tx
        ga = pnc.Vec2d(); ga.x = pt[0] + nx * half_w; ga.y = pt[1] + ny * half_w
        gb = pnc.Vec2d(); gb.x = pt[0] - nx * half_w; gb.y = pt[1] - ny * half_w
        gates.append((ga, gb))
    return gates


def plan_through_gates(grid, grid_meta, start_pose, gates):
    ha = pnc.HybridAStar(grid)
    ha.set_cell_size(CELL_SIZE); ha.set_wheelbase(L_WB)
    ha.set_max_steer(MAX_STEER); ha.set_num_steer(5)
    ha.set_arc_length(HA_ARC_LENGTH)
    ha.set_goal_xy_tol(1.0); ha.set_goal_th_tol(1.0)
    ha.set_vehicle_dims(COLLISION_MARGIN, VEHICLE_FWD + 0.2, VEHICLE_REV + 0.2)
    ha.set_grid_origin(grid_meta["x_min"], grid_meta["y_min"])
    full_path = []; current = start_pose; cf = 0
    for gi, (ga, gb) in enumerate(gates):
        seg = []
        if gi == 0:
            m = pnc.Pose(); m.x = (ga.x + gb.x) * 0.5; m.y = (ga.y + gb.y) * 0.5; m.theta = 0.0
            try: seg = ha.plan(current, m)
            except: pass
        else:
            try: seg = ha.plan_to_gate(current, ga, gb)
            except: pass
        if not seg:
            if gi > 0:
                ha.set_goal_xy_tol(3.0)
                try: seg = ha.plan_to_gate(current, ga, gb)
                except: pass
                ha.set_goal_xy_tol(1.0)
            if not seg: cf += 1
            if cf >= 3: break
            continue
        cf = 0
        if full_path and seg:
            if math.hypot(full_path[-1][0] - seg[0].x, full_path[-1][1] - seg[0].y) < 0.1:
                seg = seg[1:]
        for p in seg: full_path.append((p.x, p.y, p.theta))
        current = pnc.Pose(); current.x = full_path[-1][0]; current.y = full_path[-1][1]
        current.theta = full_path[-1][2]
        if (gi + 1) % 5 == 0 or gi == len(gates) - 1:
            print(f"  Gate {gi + 1}/{len(gates)}: {len(full_path)} pts")
    print(f"  HA*: {len(full_path)} pts")
    return full_path


def smooth_path(raw_path, grid, grid_meta):
    ref_path = [pnc.Pose() for _ in raw_path]
    for i, (x, y, th) in enumerate(raw_path):
        ref_path[i].x = x; ref_path[i].y = y; ref_path[i].theta = th
    sc = pnc.SafeCorridor()
    sc.set_margin(CORRIDOR_MARGIN); sc.set_sample_interval(2.0)
    sc.set_vehicle_half_width(COLLISION_MARGIN)
    corridors = sc.build(ref_path, grid, grid_meta["x_min"], grid_meta["y_min"],
                          grid_meta["cell_size"], grid_meta["cols"], grid_meta["rows"])
    print(f"  SafeCorridor: {len(corridors)} sections")
    bs = pnc.BSpline(); p = pnc.BSplineParams()
    p.degree = BSPLINE_DEGREE; p.num_control_points = BSPLINE_NUM_CTRL
    p.closed = False; p.resample_spacing = BSPLINE_RESAMPLE
    bs.set_params(p)
    fitted = bs.fit(ref_path, corridors)
    resampled = bs.resample(fitted)
    print(f"  B-Spline: {len(fitted)}→{len(resampled)} pts")
    return [(pp.x, pp.y, pp.theta) for pp in resampled], corridors


# =====================================================================
#  Trajectory (复用)
# =====================================================================

class Trajectory:
    def __init__(self, points: np.ndarray):
        self.points = points; n = len(points)
        d = np.diff(points, axis=0)
        self.cum_s = np.concatenate([[0.0], np.cumsum(np.sqrt(np.sum(d ** 2, axis=1)))])
        self.total_len = float(self.cum_s[-1])
        self.psi = np.zeros(n)
        if n >= 2:
            self.psi[0] = math.atan2(points[1, 1] - points[0, 1], points[1, 0] - points[0, 0])
            self.psi[-1] = math.atan2(points[-1, 1] - points[-2, 1], points[-1, 0] - points[-2, 0])
        for i in range(1, n - 1):
            self.psi[i] = math.atan2(points[i + 1, 1] - points[i - 1, 1],
                                     points[i + 1, 0] - points[i - 1, 0])
        self.kappa = np.zeros(n)
        for i in range(1, n - 1):
            dx = points[i + 1, 0] - points[i - 1, 0]
            dy = points[i + 1, 1] - points[i - 1, 1]
            ddx = points[i + 1, 0] - 2 * points[i, 0] + points[i - 1, 0]
            ddy = points[i + 1, 1] - 2 * points[i, 1] + points[i - 1, 1]
            den = (dx * dx + dy * dy) ** 1.5
            self.kappa[i] = (dx * ddy - dy * ddx) / den if den > 1e-8 else 0.0
        from scipy.ndimage import uniform_filter1d
        ks = min(21, max(3, n - 1))
        if ks % 2 == 0: ks -= 1
        self.kappa = uniform_filter1d(self.kappa, size=ks, mode='nearest')

    def get_state(self, s):
        sm = s % self.total_len if self.total_len > 0 else s
        idx = min(np.searchsorted(self.cum_s, sm), len(self.cum_s) - 1)
        if idx == 0:
            return np.array([self.points[0, 0], self.points[0, 1], self.psi[0], self.kappa[0]])
        s0, s1 = self.cum_s[idx - 1], self.cum_s[idx]
        t = (sm - s0) / (s1 - s0) if s1 > s0 else 0.0
        x = (1 - t) * self.points[idx - 1, 0] + t * self.points[idx, 0]
        y = (1 - t) * self.points[idx - 1, 1] + t * self.points[idx, 1]
        d = self.psi[idx] - self.psi[idx - 1]
        d = (d + math.pi) % (2 * math.pi) - math.pi
        return np.array([x, y, self.psi[idx - 1] + t * d,
                         (1 - t) * self.kappa[idx - 1] + t * self.kappa[idx]])


# =====================================================================
#  辅助函数
# =====================================================================

def build_cont_matrices():
    A = np.array([[0, 1, 0, 0],
                  [0, -(2 * C_AF + 2 * C_AR) / (MASS * VX),
                   (2 * C_AF + 2 * C_AR) / MASS,
                   -(2 * C_AF * LF - 2 * C_AR * LR) / (MASS * VX)],
                  [0, 0, 0, 1],
                  [0, -(2 * C_AF * LF - 2 * C_AR * LR) / (IZ * VX),
                   (2 * C_AF * LF - 2 * C_AR * LR) / IZ,
                   -(2 * C_AF * LF * LF + 2 * C_AR * LR * LR) / (IZ * VX)]])
    B1 = np.array([[0], [2 * C_AF / MASS], [0], [2 * C_AF * LF / IZ]])
    B2 = np.array([[0], [-(2 * C_AF * LF - 2 * C_AR * LR) / (MASS * VX) - VX],
                   [0], [-(2 * C_AF * LF * LF + 2 * C_AR * LR * LR) / (IZ * VX)]])
    return A, B1, B2


def feedforward(kappa):
    L = L_WB
    return L * kappa + (LR / (L * C_AF) - LF / (L * C_AR)) * MASS / 2 * VX * VX * kappa


def find_centerline_s(x, y, pts, cum_s, prev_s=None):
    dx = pts[:, 0] - x; dy = pts[:, 1] - y
    if prev_s is not None:
        total = float(cum_s[-1])
        lo = max(0.0, prev_s - 5.0); hi = min(total, prev_s + 5.0)
        i0 = int(np.searchsorted(cum_s, lo)); i1 = int(np.searchsorted(cum_s, hi)) + 1
        i0 = max(0, i0); i1 = min(len(pts), i1)
        if i1 > i0 + 1:
            local = dx[i0:i1] ** 2 + dy[i0:i1] ** 2
            return float(cum_s[i0 + int(np.argmin(local))])
    return float(cum_s[int(np.argmin(dx * dx + dy * dy))])


def compute_error_from_engine(ego_x, ego_y, ego_theta, ego_vel, ref_traj, local_s):
    """从引擎世界位姿计算横向误差 (e_y, e_psi)"""
    ref = ref_traj.get_state(local_s)
    ref_x, ref_y, ref_psi, kappa = float(ref[0]), float(ref[1]), float(ref[2]), float(ref[3])

    # 横向误差: 世界位姿相对于参考轨迹的横向偏移
    dx = ego_x - ref_x; dy = ego_y - ref_y
    e_y = -dx * math.sin(ref_psi) + dy * math.cos(ref_psi)
    e_psi = ego_theta - ref_psi
    e_psi = (e_psi + math.pi) % (2 * math.pi) - math.pi

    return e_y, e_psi, kappa, ref


def _lateral_clearance(x, y, psi, sign, outer_poly, hole_polys):
    """沿 sign 方向 (+1=左, -1=右) 探测到道路边界或孔洞的距离"""
    nx = -math.sin(psi) * sign; ny = math.cos(psi) * sign
    for d in np.linspace(0.5, 20.0, 40):
        px = x + nx * d; py = y + ny * d
        # 超出外边界
        if not _point_in_poly(px, py, outer_poly):
            return d - 0.5
        # 进入孔洞
        for hole in hole_polys:
            if _point_in_poly(px, py, hole):
                return d - 0.5
    return 20.0


def _vehicle_corners_world(cx, cy, h, hw, hf, hr):
    c = math.cos(h); s = math.sin(h)
    corners = []
    for lx, ly in [(hf, hw), (hf, -hw), (-hr, -hw), (-hr, hw)]:
        corners.append((cx + lx * c - ly * s, cy + lx * s + ly * c))
    return corners


def _check_ego_in_bounds(ego_x, ego_y, ego_h, mpl_outer, mpl_holes):
    """检查 ego 四角: 在外边界内 且 不在任何孔洞(岛屿)内"""
    corners = _vehicle_corners_world(ego_x, ego_y, ego_h,
                                     VEHICLE_HW, VEHICLE_FWD, VEHICLE_REV)
    for cx, cy in corners:
        # 必须在赛道外边界内
        if not mpl_outer.contains_point((cx, cy)):
            return False
        # 不能进入孔洞 (岛屿)
        for hole_path in mpl_holes:
            if hole_path.contains_point((cx, cy)):
                return False
    return True


# =====================================================================
#  NPC: 引擎 SimpleModel + 中心线跟随控制
# =====================================================================

class NpcAgent(Agent):
    """沿中心线匀速行驶的 NPC, 由引擎 SimpleModel 驱动"""

    def __init__(self, entity_id: int, start_ratio: float, speed: float,
                 centerline_pts, cl_cum_s, cl_total_len, name: str = "npc"):
        super().__init__(entity_id)
        self.start_ratio = start_ratio
        self.speed = speed
        self.cl_pts = centerline_pts
        self.cl_cum_s = cl_cum_s
        self.cl_total_len = cl_total_len
        self.name = name
        self._prev_s = None

    def init(self, world: World) -> None:
        pass

    def _get_centerline_s(self, x, y):
        s = find_centerline_s(x, y, self.cl_pts, self.cl_cum_s, self._prev_s)
        self._prev_s = s
        return s

    def tick(self, percepts):
        es = percepts.ego_state
        s = self._get_centerline_s(es.pose.x, es.pose.y)

        # 目标 heading: 中心线前方 2m
        lookahead_s = (s + 2.0) % self.cl_total_len
        n = len(self.cl_pts)
        idx = min(int(np.searchsorted(self.cl_cum_s, lookahead_s)), n - 1)
        i0, i1 = max(idx - 1, 0), min(idx + 1, n - 1)
        target_h = math.atan2(self.cl_pts[i1, 1] - self.cl_pts[i0, 1],
                              self.cl_pts[i1, 0] - self.cl_pts[i0, 0])

        # SimpleModel: steer = 角速度 (heading 误差 P 控制)
        heading_err = target_h - es.pose.theta
        heading_err = (heading_err + math.pi) % (2 * math.pi) - math.pi
        steer = heading_err * 4.0  # P 增益
        steer = max(-1.5, min(1.5, steer))

        # 速度保持
        cur_spd = math.hypot(es.vel.vx, es.vel.vy)
        ax = (self.speed - cur_spd) * 3.0

        return ep.ControlInput(steer, ax)


# =====================================================================
#  Main
# =====================================================================

def run():
    print("=" * 60)
    print("  引擎版动态障碍物避障 — Engine + MPC")
    print("=" * 60)

    from pipeline.map_parser import parse_map
    from pipeline.centerline import extract_centerline_graph
    from pipeline.sim_lane_keeping_real import assemble_go_straight_circuit

    # [1] 地图解析
    img = os.path.join(_project_root, "map", "path2.png")
    print(f"\n[1/7] 解析地图: {img}")
    bounds = parse_map(img, pixels_per_meter=12.8, smoothing_factor=0.0,
                       num_control_points=200, resample_spacing_m=0.1,
                       has_starting_line=True)
    outer = np.array(bounds["outer_boundary"])
    map_holes = [np.array(h) for h in bounds["holes"]]
    print(f"  Outer: {len(outer)} pts, Holes: {len(map_holes)}")

    # [2] 中心线 + 回路
    print("[2/7] 中心线 & 回路 ...")
    graph = extract_centerline_graph(bounds["outer_boundary"], bounds["holes"],
                                     pixels_per_meter=12.8, smoothing_factor=0.02,
                                     starting_line=bounds.get("starting_line"))
    loop_pts = assemble_go_straight_circuit(graph)
    cl_diffs = np.diff(loop_pts, axis=0)
    cl_cum_s = np.concatenate([[0.0], np.cumsum(np.sqrt(np.sum(cl_diffs ** 2, axis=1)))])
    cl_total_len = float(cl_cum_s[-1])
    print(f"  Centerline: {len(loop_pts)} pts, {cl_total_len:.1f} m")

    # [3] HA* + B-Spline → 参考轨迹
    print("[3/7] HA* + B-Spline ...")
    base_grid, grid_meta = build_occupancy_grid(outer, map_holes, CELL_SIZE, SAFETY_MARGIN)
    gates = generate_gates(loop_pts, GATE_SPACING, LANE_WIDTH)
    print(f"  Gates: {len(gates)}"); gates.append(gates[0])
    start_pose = pnc.Pose()
    start_pose.x = (gates[0][0].x + gates[0][1].x) * 0.5
    start_pose.y = (gates[0][0].y + gates[0][1].y) * 0.5
    gx = gates[0][0].x - gates[0][1].x; gy = gates[0][0].y - gates[0][1].y
    start_pose.theta = math.atan2(-gx, gy)
    raw_path = plan_through_gates(base_grid, grid_meta, start_pose, gates)
    if len(raw_path) < 3:
        print("  ❌ HA* 失败, 退回中心线")
        raw_path = [(x, y, 0.0) for x, y in loop_pts]
    smoothed, corridors = smooth_path(raw_path, base_grid, grid_meta)
    pts = np.array([(p[0], p[1]) for p in smoothed])
    traj = Trajectory(pts)
    print(f"  参考轨迹: {traj.total_len:.1f} m, max|kappa|={np.max(np.abs(traj.kappa)):.4f}")

    # [4] 引擎 World 构建
    print("\n[4/7] 引擎 World 构建 ...")
    world = World(dt=DT_ENGINE)

    # 赛道边界 → 静态墙体
    build_boundary_walls(world, outer, map_holes, thickness=0.5)
    print(f"  静态墙体: {world.entity_count} 段")

    # Ego (引擎 BicycleModel 驱动 — 碰撞响应自动处理)
    ego_state = ep.EntityState()
    ego_state.pose = ep.Pose(start_pose.x, start_pose.y, start_pose.theta)
    ego_state.vel = ep.Velocity(VX, 0.0, 0.0)
    ego_state.geometry = ep.Polygon.vehicle(VEHICLE_HW, VEHICLE_FWD, VEHICLE_REV)
    ego_id = world.add_entity(ego_state, ep.BicycleModel(L_WB))
    print(f"  Ego: id={ego_id}, mass={world.get_entity_state(ego_id).mass:.1f}")
    ego_geom = ego_state.geometry  # 碰撞几何引用

    # NPC
    npc_agents = []
    npc_ids = []
    for i, (start_ratio, name) in enumerate([(NPC_1_START, "npc1"), (NPC_2_START, "npc2")]):
        init_s = cl_total_len * start_ratio
        idx = int(np.searchsorted(cl_cum_s, init_s))
        idx = min(idx, len(loop_pts) - 1)
        npx, npy = float(loop_pts[idx, 0]), float(loop_pts[idx, 1])
        h = math.atan2(loop_pts[min(idx + 1, len(loop_pts) - 1), 1] - loop_pts[max(idx - 1, 0), 1],
                       loop_pts[min(idx + 1, len(loop_pts) - 1), 0] - loop_pts[max(idx - 1, 0), 0])

        ns = ep.EntityState()
        ns.pose = ep.Pose(npx, npy, h)
        ns.vel = ep.Velocity(NPC_SPEED * math.cos(h), NPC_SPEED * math.sin(h), 0.0)
        ns.geometry = ep.Polygon.vehicle(NPC_HW, NPC_FWD, NPC_REV)
        nid = world.add_entity(ns, ep.SimpleModel())

        agent = NpcAgent(nid, start_ratio, NPC_SPEED, loop_pts, cl_cum_s, cl_total_len, name)
        npc_agents.append(agent)
        npc_ids.append(nid)
        world.register_agent(agent)
        print(f"  {name}: id={nid}, s={init_s:.0f}m, pos=({npx:.0f},{npy:.0f})")

    # [5] MPC 初始化 (使用 pnc.BicycleModel)
    print("\n[5/7] MPC 初始化 ...")
    A_c, B1_c, B2_c = build_cont_matrices()
    A_d = np.eye(4) + A_c * DT_MPC; B1_d = B1_c * DT_MPC
    C_mat = np.eye(4); D_mat = np.zeros((4, 1))
    model = pnc.BicycleModel(A_c, B1_c, B2_c, C_mat, D_mat)
    init_state = np.array([-0.3, 0.0, 0.05, 0.0])
    P_kf = np.eye(4) * 1.0; Q_kf = np.eye(4) * 0.01
    R_kf = np.diag([0.1, 0.1, 0.025, 0.005])
    model.kf.init(A_d, B1_d, C_mat, P_kf, Q_kf, R_kf, init_state)
    model.init(init_state)
    Qw = np.diag([120.0, 0.5, 15.0, 0.5]); Rw = np.array([[0.1]])
    S_term = np.eye(4) * 1.0
    mpc_ctrl = pnc.MPC(); mpc_ctrl.init(A_d, B1_d, C_mat, Qw, Rw, S_term, N_HORIZON)

    # [6] 仿真循环 (ego + NPC 均由引擎驱动, 碰撞响应自动处理)
    N_STEPS = min(int(cl_total_len / (VX * DT_MPC)) + 200, 5000)
    print(f"\n[6/7] 仿真 (MPC dt={DT_MPC}s, engine dt={DT_ENGINE}s, "
          f"{STEPS_PER_MPC} engine steps/MPC step) ...")

    sensor = Sensor()
    mpl_outer = MplPath(outer)
    mpl_holes = [MplPath(h) for h in map_holes]

    log = []; npc_log = []; offset_log = []
    steer = 0.0; local_s = 0.0
    ego_prev_s = None
    collision_npc = False; collision_wall = False

    # 状态机
    overtake_state = 'IDLE'; overtake_dir = 0
    overtake_npc = None
    overtake_ramp_start_s = 0.0; overtake_ramp_down_start_s = 0.0
    total_overtakes = 0

    # 误差微分
    prev_e_y, prev_e_psi = 0.0, 0.0

    for step in range(N_STEPS):
        t = step * DT_MPC; s_travel = t * VX

        # ---- 从引擎读取 ego 世界位姿 ----
        ego_es = world.get_entity_state(ego_id)
        ego_x, ego_y, ego_h = ego_es.pose.x, ego_es.pose.y, ego_es.pose.theta
        ego_pose = ep.Pose(ego_x, ego_y, ego_h)

        # 计算误差状态 (从引擎世界位姿 → e_y, e_psi)
        ref = traj.get_state(local_s)
        ref_x, ref_y, ref_psi = float(ref[0]), float(ref[1]), float(ref[2])
        kappa_cur = float(ref[3])
        dx = ego_x - ref_x; dy = ego_y - ref_y
        e_y = -dx * math.sin(ref_psi) + dy * math.cos(ref_psi)
        e_psi = ego_h - ref_psi
        e_psi = (e_psi + math.pi) % (2 * math.pi) - math.pi

        ego_s = find_centerline_s(ego_x, ego_y, loop_pts, cl_cum_s, ego_prev_s)
        ego_prev_s = ego_s

        # ---- 状态机: 超车决策 ----
        lat_offset = 0.0
        nearest_npc = None; nearest_lon = float('inf')
        total = float(cl_cum_s[-1])
        for npc_a in npc_agents:
            ns = world.get_entity_state(npc_a.entity_id)
            npc_s = find_centerline_s(ns.pose.x, ns.pose.y, loop_pts, cl_cum_s)
            if npc_s is None: continue
            lon = (npc_s - ego_s) % total
            if lon > total / 2: lon -= total
            if 0 < lon < nearest_lon:
                nearest_lon = lon; nearest_npc = npc_a

        if overtake_state == 'IDLE':
            if nearest_npc is not None and nearest_lon < TRIGGER_DIST:
                room_l = _lateral_clearance(ego_x, ego_y, ego_h, +1, outer, map_holes)
                room_r = _lateral_clearance(ego_x, ego_y, ego_h, -1, outer, map_holes)
                overtake_dir = 1 if room_l >= MAX_OFFSET else -1
                overtake_npc = nearest_npc
                overtake_ramp_start_s = ego_s
                overtake_state = 'RAMPING_UP'
                total_overtakes += 1
                print(f"  [{step:4d}] 超车 #{total_overtakes}: "
                      f"dir={'左' if overtake_dir > 0 else '右'} npc_lon={nearest_lon:.0f}m")

        elif overtake_state == 'RAMPING_UP':
            progress = (ego_s - overtake_ramp_start_s) % total
            if progress > total / 2: progress -= total
            lat_offset = overtake_dir * MAX_OFFSET * min(1.0, progress / RAMP_LENGTH)
            if progress >= RAMP_LENGTH:
                overtake_state = 'HOLDING'
                print(f"  [{step:4d}] 超车 #{total_overtakes} 保持偏移 {overtake_dir * MAX_OFFSET:+.1f}m")

        elif overtake_state == 'HOLDING':
            lat_offset = overtake_dir * MAX_OFFSET
            if overtake_npc is not None:
                ns = world.get_entity_state(overtake_npc.entity_id)
                npc_s = find_centerline_s(ns.pose.x, ns.pose.y, loop_pts, cl_cum_s)
                lon = (npc_s - ego_s) % total
                if lon > total / 2: lon -= total
                if lon < -PASSED_DIST:
                    overtake_ramp_down_start_s = ego_s
                    overtake_state = 'RAMPING_DOWN'
                    print(f"  [{step:4d}] 超车 #{total_overtakes} 归中, npc_lon={lon:.0f}m")

        elif overtake_state == 'RAMPING_DOWN':
            progress = (ego_s - overtake_ramp_down_start_s) % total
            if progress > total / 2: progress -= total
            lat_offset = overtake_dir * MAX_OFFSET * max(0.0, 1.0 - progress / RAMP_LENGTH)
            if progress >= RAMP_LENGTH:
                lat_offset = 0.0; overtake_state = 'IDLE'
                overtake_dir = 0; overtake_npc = None
                print(f"  [{step:4d}] 超车 #{total_overtakes} 完成")

        # 路宽钳位
        if abs(lat_offset) > 0.01:
            room = _lateral_clearance(ego_x, ego_y, ego_h,
                                      1 if lat_offset > 0 else -1, outer, map_holes)
            lat_offset = math.copysign(min(abs(lat_offset), room * 0.8), lat_offset)

        # ---- KF + MPC ----
        de_y = (e_y - prev_e_y) / DT_MPC if step > 0 else 0.0
        de_psi = (e_psi - prev_e_psi) / DT_MPC if step > 0 else 0.0
        prev_e_y, prev_e_psi = e_y, e_psi

        measurement = np.array([e_y, de_y, e_psi, de_psi])
        model.kf.update(measurement, np.array([steer]))

        mpc_target = np.array([float(lat_offset), 0.0, 0.0, 0.0])
        u_fb = mpc_ctrl.predict(mpc_target, model.kf.x_post)
        steer_ff = feedforward(kappa_cur)
        steer = float(u_fb[0] + steer_ff)
        steer = max(-MAX_STEER, min(MAX_STEER, steer))

        # ---- 引擎子步进 (ego + NPC 统一 100Hz, 碰撞响应自动) ----
        for sub_step in range(STEPS_PER_MPC):
            # Ego 控制
            world.apply_control(ego_id, ep.ControlInput(steer, 0.0))
            # NPC 控制
            for npc_a in npc_agents:
                npc_percepts = sensor.get_percepts(world, npc_a.entity_id)
                npc_cmd = npc_a.tick(npc_percepts)
                world.apply_control(npc_a.entity_id, npc_cmd)
            # 物理步进 (碰撞检测 + 弹性响应 由引擎自动完成)
            sub_collisions = world.step()

            # 记录碰撞事件
            for c in sub_collisions:
                ids = {c.entity_a, c.entity_b}
                if ego_id in ids:
                    other_id = c.entity_a if c.entity_b == ego_id else c.entity_b
                    other_state = world.get_entity_state(other_id)
                    is_npc = any(npc_id == other_id for npc_id in npc_ids)
                    t_sub = t + sub_step * DT_ENGINE
                    if is_npc and not collision_npc:
                        print(f"  ❌ [{step:4d}] t={t_sub:.2f}s NPC碰撞! "
                              f"pen={c.result.penetration:.3f}m → 弹性反弹")
                        collision_npc = True
                    elif not is_npc and not collision_wall:
                        print(f"  ❌ [{step:4d}] t={t_sub:.2f}s 撞墙! "
                              f"pen={c.result.penetration:.3f}m → 弹性反弹")
                        collision_wall = True

        # 更新弧长
        local_s += VX * DT_MPC

        # 子步进后重新读取 ego 状态 (碰撞响应可能已修改)
        ego_es = world.get_entity_state(ego_id)
        ego_x, ego_y, ego_h = ego_es.pose.x, ego_es.pose.y, ego_es.pose.theta

        # 记录
        log.append((step, t, e_y, e_psi, steer, ego_x, ego_y, ego_h, lat_offset))
        offset_log.append(lat_offset)

        npc_positions = []
        for npc_a in npc_agents:
            ns = world.get_entity_state(npc_a.entity_id)
            npc_positions.append((ns.pose.x, ns.pose.y, ns.pose.theta))
        npc_log.append(npc_positions)

        if step % 100 == 0:
            min_d = min(
                math.hypot(ego_x - world.get_entity_state(a.entity_id).pose.x,
                           ego_y - world.get_entity_state(a.entity_id).pose.y)
                for a in npc_agents)
            print(f"  {step:4d} t={t:5.1f}s s={s_travel:6.0f}m "
                  f"e_y={e_y:+.3f}m offset={lat_offset:+.1f}m "
                  f"str={math.degrees(steer):+.1f}deg npc_dist={min_d:.0f}m "
                  f"state={overtake_state}")

        if s_travel >= cl_total_len:
            print(f"\n  → 一圈完成! s={s_travel:.1f}m"); break

    # 统计
    final = log[-1]
    print(f"\n[仿真完成]")
    print(f"  步数: {len(log)}, 时间: {final[1]:.1f}s, 距离: {final[1] * VX:.0f}m")
    print(f"  引擎步数: {world.step_count}")
    print(f"  超车次数: {total_overtakes}")
    print(f"  撞墙: {'❌' if collision_wall else '✅'}")
    print(f"  NPC碰撞: {'❌' if collision_npc else '✅'}")
    print(f"  Final: e_y={final[2]:.4f}m e_psi={math.degrees(final[3]):.2f}deg")

    # [7] 保存 & 可视化
    print("\n[7/7] 保存 & 可视化 ...")
    os.makedirs("output", exist_ok=True)
    arr = np.array(log)
    with open("output/sim_engine_dynamic_obstacles.txt", "w") as f:
        f.write(f"# engine_dynamic_obstacles len={cl_total_len:.1f}m dt_mpc={DT_MPC} Vx={VX}\n")
        f.write("Step\ttime\te_y\te_psi\tsteer\tego_x\tego_y\tego_heading\tlat_offset\n")
        for row in log:
            f.write(f"{int(row[0])}\t" + "\t".join(f"{v:.6f}" for v in row[1:]) + "\n")
    np.save("output/sim_engine_dynamic_obstacles_outer.npy", outer)
    np.save("output/sim_engine_dynamic_obstacles_traj.npy", traj.points)
    np.save("output/sim_engine_dynamic_obstacles_log.npy", arr)
    for i, h in enumerate(map_holes):
        np.save(f"output/sim_engine_dynamic_obstacles_hole_{i}.npy", h)
    npc_arr = np.array(npc_log)
    np.save("output/sim_engine_dynamic_obstacles_npc.npy", npc_arr)

    visualize(log, traj, smoothed, outer, map_holes, npc_log, loop_pts,
              corridors, offset_log, collision_npc, collision_wall)


# =====================================================================
#  Visualization (复用原版)
# =====================================================================

def visualize(log, traj, smoothed, outer, holes, npc_log, centerline_pts,
              corridors, offset_log, collision_npc, collision_boundary):
    arr = np.array(log); N = len(arr)
    ts = arr[:, 1]; ey = arr[:, 2]; ep = arr[:, 3]; st = arr[:, 4]
    ego_wx = arr[:, 5]; ego_wy = arr[:, 6]; ego_wh = arr[:, 7]; lat_off = arr[:, 8]
    NPC_HW_V = 1.0; NPC_FWD_V = 1.5; NPC_REV_V = 1.0  # 可视化用
    VEH_HW_V = 1.0; VEH_FWD_V = 1.5; VEH_REV_V = 1.0

    npc_arr = np.array(npc_log)[:N]
    npc1_x, npc1_y = npc_arr[:, 0, 0], npc_arr[:, 0, 1]
    npc2_x, npc2_y = npc_arr[:, 1, 0], npc_arr[:, 1, 1]
    DT_V = 0.1  # MPC 步长

    fig = plt.figure(figsize=(22, 16))
    status = "⚠ COLLISION" if (collision_npc or collision_boundary) else "✅ OK"
    fig.suptitle(f"Dynamic Obstacles — Engine + MPC  [{status}]", fontsize=16)

    ax = fig.add_subplot(2, 3, (1, 4)); ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
    ax.set_title("Track, NPCs & Ego Trajectory")
    ax.plot(outer[:, 0], outer[:, 1], "k-", lw=1.5, alpha=0.5, label="Track")
    for i, h in enumerate(holes):
        ax.fill(h[:, 0], h[:, 1], fc="white", ec="k", lw=0.8, alpha=0.95)

    if corridors:
        clip = PathPatch(MplPath(outer), transform=ax.transData)
        pk = {"clip_path": clip, "clip_on": True}
        lx = [c.left.x for c in corridors]; ly = [c.left.y for c in corridors]
        rx = [c.right.x for c in corridors]; ry = [c.right.y for c in corridors]
        ax.fill(lx + rx[::-1], ly + ry[::-1], fc="cyan", ec="none", alpha=0.1, **pk)

    ax.plot([p[0] for p in smoothed], [p[1] for p in smoothed], "b-", lw=1.5, alpha=0.4, label="Reference")

    skip = max(1, N // 300)
    ax.plot(npc1_x[::skip], npc1_y[::skip], "orange", lw=1.5, alpha=0.8, ls="--", label="NPC 1")
    ax.plot(npc2_x[::skip], npc2_y[::skip], "purple", lw=1.5, alpha=0.8, ls="--", label="NPC 2")
    ax.plot(npc1_x[0], npc1_y[0], "o", color="orange", ms=8)
    ax.plot(npc2_x[0], npc2_y[0], "o", color="purple", ms=8)

    for i in range(0, N, max(1, int(5.0 / DT_V))):
        for c, npx, npy, nph in [("orange", npc1_x[i], npc1_y[i], npc_arr[i, 0, 2]),
                                  ("purple", npc2_x[i], npc2_y[i], npc_arr[i, 1, 2])]:
            corners = _vehicle_corners_world(npx, npy, nph, NPC_HW_V, NPC_FWD_V, NPC_REV_V)
            ax.add_patch(MplPolygon(corners, closed=True, fc=c, ec="k", alpha=0.3, lw=0.5))

    skip_e = max(1, N // 500)
    ax.plot(ego_wx[::skip_e], ego_wy[::skip_e], "r-", lw=1.8, alpha=0.9, label="Ego")
    ax.plot(ego_wx[0], ego_wy[0], "go", ms=10, label="Start")
    ax.plot(ego_wx[-1], ego_wy[-1], "mo", ms=10, label="End")
    for i in range(0, N, max(1, N // 8)):
        ax.arrow(ego_wx[i], ego_wy[i], math.cos(ego_wh[i]) * 2.5, math.sin(ego_wh[i]) * 2.5,
                 head_width=1.2, fc="r", ec="r", alpha=0.4)
    ax.legend(loc="upper right", fontsize=7, ncol=2)

    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(ts, ey, "b-", lw=1.5, label="e_y")
    ax2.plot(ts, lat_off, "orange", lw=1.0, ls="--", label="target")
    ax2.axhline(0, color="k", ls="--", lw=0.5); ax2.set_title("Lateral Error & Offset")
    ax2.legend(fontsize=7); ax2.grid(True, alpha=0.3)

    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(ts, np.degrees(ep), "r-", lw=1.5); ax3.axhline(0, color="k", ls="--", lw=0.5)
    ax3.set_title("Heading Error"); ax3.grid(True, alpha=0.3)

    ax4 = fig.add_subplot(2, 3, 5)
    d1 = np.sqrt((ego_wx - npc1_x) ** 2 + (ego_wy - npc1_y) ** 2)
    d2 = np.sqrt((ego_wx - npc2_x) ** 2 + (ego_wy - npc2_y) ** 2)
    ax4.plot(ts, d1, "orange", lw=1.0, alpha=0.7, label="NPC 1")
    ax4.plot(ts, d2, "purple", lw=1.0, alpha=0.7, label="NPC 2")
    ax4.axhline(2.5, color="red", ls=":", lw=0.8); ax4.set_title("Ego-NPC Distance")
    ax4.legend(fontsize=7); ax4.grid(True, alpha=0.3)

    ax5 = fig.add_subplot(2, 3, 6)
    ax5.plot(ts, np.degrees(st), "g-", lw=1.5); ax5.set_title("Steering Angle")
    ax5.grid(True, alpha=0.3)

    plt.tight_layout()
    skip_w = max(1, N // 10)
    ey_rms = float(np.sqrt(np.mean(ey[skip_w:] ** 2)))
    ep_rms = float(np.sqrt(np.mean(ep[skip_w:] ** 2)))
    min_d1 = float(np.min(d1[skip_w:])); min_d2 = float(np.min(d2[skip_w:]))
    print(f"  RMS: e_y={ey_rms:.4f}m  e_psi={math.degrees(ep_rms):.2f}deg")
    print(f"  Min NPC dist: NPC1={min_d1:.2f}m  NPC2={min_d2:.2f}m")

    out_png = "output/sim_engine_dynamic_obstacles.png"
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"  Plot: {out_png}")
    plt.show()


if __name__ == "__main__":
    run()
