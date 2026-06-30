"""
sim_engine_dynamic_obstacles_live.py — 引擎实时可视化动态避障

引擎边计算边渲染，所有实体均在引擎中统一管理，碰撞响应由引擎自动完成。

用法:
  python pipeline/sim_engine_dynamic_obstacles_live.py
  python pipeline/sim_engine_dynamic_obstacles_live.py --save output/live.gif --time 30
"""

from __future__ import annotations

import os, sys, math, argparse, time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.path import Path as MplPath
from matplotlib.gridspec import GridSpec

_project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in [os.path.join(_project_root, "build", "pnc"),
           os.path.join(_project_root, "build", "engine", "physics"),
           _project_root]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import engine_physics as ep
from engine import World
import pnc

# 复用管线函数
from pipeline.map_parser import parse_map
from pipeline.sim_engine_dynamic_obstacles import (
    build_occupancy_grid, generate_gates, plan_through_gates, smooth_path,
    build_boundary_walls, Trajectory, build_cont_matrices, feedforward,
    find_centerline_s, _point_in_poly, _lateral_clearance,
)

# ========================== 参数 ==========================

MASS = 1573.0; IZ = 2873.0; LF = 1.1; LR = 1.58
L_WB = LF + LR; C_AF = 80000.0; C_AR = 80000.0
VX = 10.0; DT_MPC = 0.1; N_HORIZON = 40
DT_ENGINE = 0.01; STEPS_PER_MPC = int(DT_MPC / DT_ENGINE)
LANE_WIDTH = 3.5; MAX_STEER = math.radians(30.0)

CELL_SIZE = 0.2; GATE_SPACING = 15.0; HA_ARC_LENGTH = 0.6
VEHICLE_HW = 1.0; VEHICLE_FWD = 1.5; VEHICLE_REV = 1.0
SAFETY_MARGIN = VEHICLE_HW + 0.2
COLLISION_MARGIN = VEHICLE_HW + 0.2; CORRIDOR_MARGIN = COLLISION_MARGIN
BSPLINE_DEGREE = 3; BSPLINE_NUM_CTRL = 100; BSPLINE_RESAMPLE = 0.5

NPC_SPEED = 5.0; NPC_HW = 1.0; NPC_FWD = 1.5; NPC_REV = 1.0
NPC_1_START = 0.25; NPC_2_START = 0.40

TRIGGER_DIST = 50.0; RAMP_LENGTH = 30.0
MAX_OFFSET = 2.3; PASSED_DIST = 15.0

RENDER_EVERY = 3       # 每 N 个引擎步渲染一帧 (~30 FPS)
ZOOM = 30.0
TRAIL_LEN = 400        # 轨迹线保留点数


# =====================================================================
#  NPC 控制
# =====================================================================

class NpcAgent:
    def __init__(self, entity_id, speed, cl_pts, cl_cum_s, cl_total_len):
        self.entity_id = entity_id; self.speed = speed
        self.cl_pts = cl_pts; self.cl_cum_s = cl_cum_s
        self.cl_total_len = cl_total_len; self._prev_s = None

    def tick(self, world):
        es = world.get_entity_state(self.entity_id)
        s = find_centerline_s(es.pose.x, es.pose.y, self.cl_pts,
                              self.cl_cum_s, self._prev_s)
        self._prev_s = s
        lookahead = (s + 2.0) % self.cl_total_len
        n = len(self.cl_pts)
        idx = min(int(np.searchsorted(self.cl_cum_s, lookahead)), n - 1)
        i0, i1 = max(idx - 1, 0), min(idx + 1, n - 1)
        target_h = math.atan2(self.cl_pts[i1, 1] - self.cl_pts[i0, 1],
                              self.cl_pts[i1, 0] - self.cl_pts[i0, 0])
        heading_err = target_h - es.pose.theta
        heading_err = (heading_err + math.pi) % (2 * math.pi) - math.pi
        steer = max(-1.5, min(1.5, heading_err * 4.0))
        cur_spd = math.hypot(es.vel.vx, es.vel.vy)
        ax = (self.speed - cur_spd) * 3.0
        return ep.ControlInput(steer, ax)


# =====================================================================
#  辅助: 车辆世界顶点
# =====================================================================

def _vehicle_corners(cx, cy, h, hw, hf, hr):
    c, s = math.cos(h), math.sin(h)
    return [(cx + lx * c - ly * s, cy + lx * s + ly * c)
            for lx, ly in [(hf, hw), (hf, -hw), (-hr, -hw), (-hr, hw)]]


# =====================================================================
#  LiveSim — 实时仿真 + 可视化
# =====================================================================

class LiveSim:
    def __init__(self):
        # 仿真状态
        self.steer = 0.0; self.local_s = 0.0
        self.ego_prev_s = None
        self.overtake_state = 'IDLE'; self.overtake_dir = 0
        self.overtake_npc = None
        self.overtake_ramp_start_s = 0.0; self.overtake_ramp_down_start_s = 0.0
        self.total_overtakes = 0
        self.prev_e_y = 0.0; self.prev_e_psi = 0.0
        self.collision_npc = False; self.collision_wall = False

        # 渲染历史
        self.t_hist = []; self.ey_hist = []; self.ep_hist = []; self.st_hist = []
        self.d1_hist = []; self.d2_hist = []
        self.ego_trail_x = []; self.ego_trail_y = []
        self.npc1_trail_x = []; self.npc1_trail_y = []
        self.npc2_trail_x = []; self.npc2_trail_y = []
        # 帧计数
        self.frame_count = 0; self.engine_step = 0
        self._paused = False

    # ======================== Setup ========================

    def setup(self):
        print("=" * 60)
        print("  引擎实时可视化 — 动态避障")
        print("=" * 60)

        # [1] 地图
        img = os.path.join(_project_root, "map", "path2.png")
        print(f"\n[1] 解析地图: {img}")
        bounds = parse_map(img, pixels_per_meter=12.8, smoothing_factor=0.0,
                           num_control_points=200, resample_spacing_m=0.1,
                           has_starting_line=True)
        self.outer = np.array(bounds["outer_boundary"])
        self.map_holes = [np.array(h) for h in bounds["holes"]]
        print(f"  Outer: {len(self.outer)} pts, Holes: {len(self.map_holes)}")

        # [2] 中心线
        print("[2] 中心线 ...")
        from pipeline.centerline import extract_centerline_graph
        from pipeline.sim_lane_keeping_real import assemble_go_straight_circuit
        graph = extract_centerline_graph(bounds["outer_boundary"], bounds["holes"],
                                         pixels_per_meter=12.8, smoothing_factor=0.02,
                                         starting_line=bounds.get("starting_line"))
        loop_pts = assemble_go_straight_circuit(graph)
        cl_diffs = np.diff(loop_pts, axis=0)
        self.cl_cum_s = np.concatenate([[0.0], np.cumsum(np.sqrt(np.sum(cl_diffs ** 2, axis=1)))])
        self.cl_total_len = float(self.cl_cum_s[-1])
        self.loop_pts = loop_pts
        print(f"  {len(loop_pts)} pts, {self.cl_total_len:.0f} m")

        # [3] HA* → 参考轨迹
        print("[3] HA* + B-Spline ...")
        base_grid, grid_meta = build_occupancy_grid(self.outer, self.map_holes,
                                                     CELL_SIZE, SAFETY_MARGIN)
        gates = generate_gates(loop_pts, GATE_SPACING, LANE_WIDTH)
        gates.append(gates[0])
        self.start_pose = pnc.Pose()
        self.start_pose.x = (gates[0][0].x + gates[0][1].x) * 0.5
        self.start_pose.y = (gates[0][0].y + gates[0][1].y) * 0.5
        gx = gates[0][0].x - gates[0][1].x; gy = gates[0][0].y - gates[0][1].y
        self.start_pose.theta = math.atan2(-gx, gy)
        raw_path = plan_through_gates(base_grid, grid_meta, self.start_pose, gates)
        if len(raw_path) < 3:
            raw_path = [(x, y, 0.0) for x, y in loop_pts]
        smoothed, self.corridors = smooth_path(raw_path, base_grid, grid_meta)
        self.traj = Trajectory(np.array([(p[0], p[1]) for p in smoothed]))
        self.smoothed = smoothed
        print(f"  参考轨迹: {self.traj.total_len:.1f} m")

        # [4] 引擎 World
        print("\n[4] 引擎 World ...")
        self.world = World(dt=DT_ENGINE)
        build_boundary_walls(self.world, self.outer, self.map_holes, thickness=0.5)

        ego_model = ep.BicycleModel(L_WB)
        ego_model.lat_damping = 15.0  # 强阻尼: 碰撞侧滑 ~0.15s 衰减
        ego_st = ep.EntityState()
        ego_st.pose = ep.Pose(self.start_pose.x, self.start_pose.y, self.start_pose.theta)
        ego_st.vel = ep.Velocity(VX, 0.0, 0.0)
        ego_st.geometry = ep.Polygon.vehicle(VEHICLE_HW, VEHICLE_FWD, VEHICLE_REV)
        self.ego_id = self.world.add_entity(ego_st, ego_model)
        print(f"  Ego: id={self.ego_id}")

        self.npc_agents = []; self.npc_ids = []
        for sr, name in [(NPC_1_START, "npc1"), (NPC_2_START, "npc2")]:
            init_s = self.cl_total_len * sr
            idx = min(int(np.searchsorted(self.cl_cum_s, init_s)), len(loop_pts) - 1)
            npx, npy = float(loop_pts[idx, 0]), float(loop_pts[idx, 1])
            h = math.atan2(loop_pts[min(idx + 1, len(loop_pts) - 1), 1] - loop_pts[max(idx - 1, 0), 1],
                           loop_pts[min(idx + 1, len(loop_pts) - 1), 0] - loop_pts[max(idx - 1, 0), 0])
            ns = ep.EntityState()
            ns.pose = ep.Pose(npx, npy, h)
            ns.vel = ep.Velocity(NPC_SPEED * math.cos(h), NPC_SPEED * math.sin(h), 0.0)
            ns.geometry = ep.Polygon.vehicle(NPC_HW, NPC_FWD, NPC_REV)
            nid = self.world.add_entity(ns, ep.SimpleModel())
            a = NpcAgent(nid, NPC_SPEED, loop_pts, self.cl_cum_s, self.cl_total_len)
            self.npc_agents.append(a); self.npc_ids.append(nid)
            print(f"  {name}: id={nid}")

        # [5] MPC
        print("[5] MPC ...")
        A_c, B1_c, B2_c = build_cont_matrices()
        A_d = np.eye(4) + A_c * DT_MPC; B1_d = B1_c * DT_MPC
        C_mat = np.eye(4); D_mat = np.zeros((4, 1))
        self.model = pnc.BicycleModel(A_c, B1_c, B2_c, C_mat, D_mat)
        init_state = np.array([-0.3, 0.0, 0.05, 0.0])
        P_kf = np.eye(4) * 1.0
        Q_kf = np.eye(4) * 0.5      # 增大: 模型不准确, 少信任预测
        R_kf = np.diag([0.02, 0.02, 0.005, 0.001])  # 减小: 多信任引擎实测
        self.model.kf.init(A_d, B1_d, C_mat, P_kf, Q_kf, R_kf, init_state)
        self.model.init(init_state)
        Qw = np.diag([30.0, 0.5, 5.0, 0.5])  # 降低 e_y 权重, 避免过激转向
        Rw = np.array([[0.1]])
        S_term = np.eye(4) * 1.0
        self.mpc = pnc.MPC(); self.mpc.init(A_d, B1_d, C_mat, Qw, Rw, S_term, N_HORIZON)
        self.mpl_outer = MplPath(self.outer)
        self.mpl_holes = [MplPath(h) for h in self.map_holes]

        # [6] 可视化初始化
        print("[6] 初始化画布 ...")
        self._init_plot()
        self._running = True

    # ======================== Plot ========================

    def _init_plot(self):
        self.fig = plt.figure(figsize=(20, 12))
        self.fig.canvas.mpl_connect('close_event', self._on_close)
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        self.fig.suptitle("Engine Live — Dynamic Obstacle Avoidance", fontsize=14)

        gs = GridSpec(4, 2, figure=self.fig, width_ratios=[3, 1],
                      hspace=0.35, wspace=0.25, left=0.04, right=0.96,
                      top=0.93, bottom=0.06)

        # 主地图
        self.ax_map = ax_map = self.fig.add_subplot(gs[:, 0])
        ax_map.set_aspect("equal"); ax_map.grid(True, alpha=0.2)
        ax_map.set_xlabel("X (m)"); ax_map.set_ylabel("Y (m)")
        ax_map.plot(self.outer[:, 0], self.outer[:, 1], "#444", lw=1.8, alpha=0.6)
        for h in self.map_holes:
            ax_map.fill(h[:, 0], h[:, 1], fc="white", ec="#444", lw=0.5, alpha=0.95)
        ax_map.plot(self.traj.points[:, 0], self.traj.points[:, 1],
                    "b--", lw=0.6, alpha=0.35)

        # Artists
        self.trail_ego, = ax_map.plot([], [], "r-", lw=1.2, alpha=0.6)
        self.trail_n1, = ax_map.plot([], [], "orange", lw=1.0, alpha=0.5, ls="--")
        self.trail_n2, = ax_map.plot([], [], "purple", lw=1.0, alpha=0.5, ls="--")
        self.ego_patch = MplPolygon([[0, 0]] * 4, closed=True, fc="#e63946",
                                     ec="#c1121f", lw=2, alpha=0.85, zorder=5)
        self.n1_patch = MplPolygon([[0, 0]] * 4, closed=True, fc="orange",
                                    ec="darkorange", lw=1.5, alpha=0.7, zorder=4)
        self.n2_patch = MplPolygon([[0, 0]] * 4, closed=True, fc="purple",
                                    ec="darkviolet", lw=1.5, alpha=0.7, zorder=4)
        ax_map.add_patch(self.ego_patch)
        ax_map.add_patch(self.n1_patch); ax_map.add_patch(self.n2_patch)

        self.info = ax_map.text(0.015, 0.985, "", transform=ax_map.transAxes,
                                fontsize=9, va="top", family="monospace",
                                bbox=dict(boxstyle="round", fc="lightyellow", alpha=0.85))

        # 右侧子图
        self.ax_dn = self.fig.add_subplot(gs[0, 1]); self.ax_dn.set_title("Ego-NPC Distance")
        self.ax_dn.set_ylabel("m"); self.ax_dn.grid(True, alpha=0.3)
        self.ax_dn.axhline(2.5, color="red", ls=":", lw=0.8, alpha=0.5)
        self.dn1, = self.ax_dn.plot([], [], "orange", lw=1.0)
        self.dn2, = self.ax_dn.plot([], [], "purple", lw=1.0)

        self.ax_ey = self.fig.add_subplot(gs[1, 1], sharex=self.ax_dn)
        self.ax_ey.set_title("Lateral Error e_y"); self.ax_ey.set_ylabel("m")
        self.ax_ey.grid(True, alpha=0.3); self.ax_ey.axhline(0, color="k", ls="--", lw=0.5)
        self.ax_ey.set_ylim(-5, 5)
        self.ey_ln, = self.ax_ey.plot([], [], "b-", lw=1.0)

        self.ax_ep = self.fig.add_subplot(gs[2, 1], sharex=self.ax_dn)
        self.ax_ep.set_title("Heading Error e_psi"); self.ax_ep.set_ylabel("deg")
        self.ax_ep.grid(True, alpha=0.3); self.ax_ep.axhline(0, color="k", ls="--", lw=0.5)
        self.ax_ep.set_ylim(-30, 30)
        self.ep_ln, = self.ax_ep.plot([], [], "r-", lw=1.0)

        self.ax_st = self.fig.add_subplot(gs[3, 1], sharex=self.ax_dn)
        self.ax_st.set_title("Steering Angle"); self.ax_st.set_xlabel("Time (s)")
        self.ax_st.set_ylabel("deg"); self.ax_st.grid(True, alpha=0.3)
        self.ax_st.set_ylim(-35, 35)
        self.st_ln, = self.ax_st.plot([], [], "g-", lw=1.0)

        plt.setp(self.ax_dn.get_xticklabels(), visible=False)
        plt.setp(self.ax_ey.get_xticklabels(), visible=False)
        plt.setp(self.ax_ep.get_xticklabels(), visible=False)

        plt.ion()  # 交互模式
        plt.show(block=False)  # 非阻塞显示窗口

    def _on_close(self, event):
        self._running = False

    def _on_key(self, event):
        if event.key == ' ':
            self._paused = not self._paused
            print(f"  {'⏸ 暂停' if self._paused else '▶ 继续'}")

    # ======================== Main Loop ========================

    def run(self, max_time: float = 0):
        dt_mpc = DT_MPC
        n_steps = min(int(self.cl_total_len / (VX * dt_mpc)) + 200, 5000)
        if max_time > 0:
            n_steps = min(n_steps, int(max_time / dt_mpc))

        print(f"\n[7] 实时仿真 ({n_steps} steps, ~{n_steps * dt_mpc:.0f}s)")
        print("    [空格] 暂停/继续  [关闭窗口] 退出\n")

        t_start = time.time()
        world = self.world; ego_id = self.ego_id
        npc_ids = self.npc_ids; npc_agents = self.npc_agents
        cl_total = float(self.cl_cum_s[-1])

        for step in range(n_steps):
            if not self._running:
                break
            while self._paused:
                self.fig.canvas.flush_events(); time.sleep(0.05)
                if not self._running: break
            if not self._running: break

            t = step * dt_mpc

            # ---- Ego 状态 (从引擎读取) ----
            es = world.get_entity_state(ego_id)
            ego_x, ego_y, ego_h = es.pose.x, es.pose.y, es.pose.theta

            ref = self.traj.get_state(self.local_s)
            ref_x, ref_y, ref_psi = float(ref[0]), float(ref[1]), float(ref[2])
            kappa_cur = float(ref[3])
            dx = ego_x - ref_x; dy = ego_y - ref_y
            e_y = -dx * math.sin(ref_psi) + dy * math.cos(ref_psi)
            e_psi = ego_h - ref_psi
            e_psi = (e_psi + math.pi) % (2 * math.pi) - math.pi
            ego_s = find_centerline_s(ego_x, ego_y, self.loop_pts, self.cl_cum_s,
                                      self.ego_prev_s)
            self.ego_prev_s = ego_s

            # ---- 状态机 ----
            lat_offset = 0.0
            nearest_npc, nearest_lon = None, float('inf')
            for npc_a in npc_agents:
                ns = world.get_entity_state(npc_a.entity_id)
                npc_s = find_centerline_s(ns.pose.x, ns.pose.y, self.loop_pts, self.cl_cum_s)
                lon = (npc_s - ego_s) % cl_total
                if lon > cl_total / 2: lon -= cl_total
                if 0 < lon < nearest_lon:
                    nearest_lon = lon; nearest_npc = npc_a

            if self.overtake_state == 'IDLE':
                if nearest_npc is not None and nearest_lon < TRIGGER_DIST:
                    room_l = _lateral_clearance(ego_x, ego_y, ego_h, +1, self.outer,
                                                self.map_holes)
                    room_r = _lateral_clearance(ego_x, ego_y, ego_h, -1, self.outer,
                                                self.map_holes)
                    self.overtake_dir = 1 if room_l >= MAX_OFFSET else -1
                    self.overtake_npc = nearest_npc
                    self.overtake_ramp_start_s = ego_s
                    self.overtake_state = 'RAMPING_UP'
                    self.total_overtakes += 1
            elif self.overtake_state == 'RAMPING_UP':
                progress = (ego_s - self.overtake_ramp_start_s) % cl_total
                if progress > cl_total / 2: progress -= cl_total
                lat_offset = self.overtake_dir * MAX_OFFSET * min(1.0, progress / RAMP_LENGTH)
                if progress >= RAMP_LENGTH:
                    self.overtake_state = 'HOLDING'
            elif self.overtake_state == 'HOLDING':
                lat_offset = self.overtake_dir * MAX_OFFSET
                if self.overtake_npc is not None:
                    ns = world.get_entity_state(self.overtake_npc.entity_id)
                    npc_s = find_centerline_s(ns.pose.x, ns.pose.y, self.loop_pts, self.cl_cum_s)
                    lon = (npc_s - ego_s) % cl_total
                    if lon > cl_total / 2: lon -= cl_total
                    if lon < -PASSED_DIST:
                        self.overtake_ramp_down_start_s = ego_s
                        self.overtake_state = 'RAMPING_DOWN'
            elif self.overtake_state == 'RAMPING_DOWN':
                progress = (ego_s - self.overtake_ramp_down_start_s) % cl_total
                if progress > cl_total / 2: progress -= cl_total
                lat_offset = self.overtake_dir * MAX_OFFSET * max(0.0, 1.0 - progress / RAMP_LENGTH)
                if progress >= RAMP_LENGTH:
                    lat_offset = 0.0; self.overtake_state = 'IDLE'
                    self.overtake_dir = 0; self.overtake_npc = None

            if abs(lat_offset) > 0.01:
                room = _lateral_clearance(ego_x, ego_y, ego_h,
                                          1 if lat_offset > 0 else -1,
                                          self.outer, self.map_holes)
                lat_offset = math.copysign(min(abs(lat_offset), room * 0.8), lat_offset)

            # ---- KF + MPC ----
            de_y = (e_y - self.prev_e_y) / dt_mpc if step > 0 else 0.0
            de_psi = (e_psi - self.prev_e_psi) / dt_mpc if step > 0 else 0.0
            self.prev_e_y, self.prev_e_psi = e_y, e_psi
            self.model.kf.update(np.array([e_y, de_y, e_psi, de_psi]),
                                 np.array([self.steer]))
            u_fb = self.mpc.predict(np.array([float(lat_offset), 0.0, 0.0, 0.0]),
                                    self.model.kf.x_post)
            steer_ff = feedforward(kappa_cur)
            self.steer = max(-MAX_STEER, min(MAX_STEER, float(u_fb[0] + steer_ff)))

            # ---- 引擎子步进 (ego + NPC 统一 100Hz) ----
            # 速度控制: 维持 VX=10m/s 巡航
            es_v = world.get_entity_state(ego_id)
            cur_spd = math.hypot(es_v.vel.vx, es_v.vel.vy)
            ax_ego = (VX - cur_spd) * 3.0  # 巡航速度 P 控制
            for sub_step in range(STEPS_PER_MPC):
                world.apply_control(ego_id, ep.ControlInput(self.steer, ax_ego))
                for npc_a in npc_agents:
                    world.apply_control(npc_a.entity_id, npc_a.tick(world))
                collisions = world.step()
                self.engine_step += 1

                for c in collisions:
                    if ego_id in (c.entity_a, c.entity_b):
                        other = c.entity_a if c.entity_b == ego_id else c.entity_b
                        if other in npc_ids:
                            if not self.collision_npc:
                                print(f"  ⚡ t={t + sub_step * DT_ENGINE:.2f}s NPC碰撞! "
                                      f"pen={c.result.penetration:.3f}m → 弹性反弹")
                                self.collision_npc = True
                        elif not self.collision_wall:
                            print(f"  ⚡ t={t + sub_step * DT_ENGINE:.2f}s 撞墙! "
                                  f"pen={c.result.penetration:.3f}m → 弹性反弹")
                            self.collision_wall = True
                        # 碰撞后立即打印5帧诊断
                        self._debug_frames = 5

                # 实时渲染
                if self.engine_step % RENDER_EVERY == 0:
                    self._render(t, e_y, self.steer, ego_x, ego_y, ego_h, lat_offset)

            self.local_s += VX * dt_mpc

            # 更新历史
            self.t_hist.append(t); self.ey_hist.append(e_y)
            self.ep_hist.append(np.degrees(e_psi)); self.st_hist.append(np.degrees(self.steer))
            self.ego_trail_x.append(ego_x); self.ego_trail_y.append(ego_y)
            if len(self.ego_trail_x) > TRAIL_LEN:
                self.ego_trail_x.pop(0); self.ego_trail_y.pop(0)
            for idx_n, na in enumerate(npc_agents):
                ns = world.get_entity_state(na.entity_id)
                tx = self.npc1_trail_x if idx_n == 0 else self.npc2_trail_x
                ty = self.npc1_trail_y if idx_n == 0 else self.npc2_trail_y
                tx.append(ns.pose.x); ty.append(ns.pose.y)
                if len(tx) > TRAIL_LEN: tx.pop(0); ty.pop(0)
            n1 = world.get_entity_state(npc_ids[0])
            n2 = world.get_entity_state(npc_ids[1])
            self.d1_hist.append(math.hypot(ego_x - n1.pose.x, ego_y - n1.pose.y))
            self.d2_hist.append(math.hypot(ego_x - n2.pose.x, ego_y - n2.pose.y))

            # 碰撞后诊断
            if hasattr(self, '_debug_frames') and self._debug_frames > 0:
                es2 = world.get_entity_state(ego_id)
                spd = math.hypot(es2.vel.vx, es2.vel.vy)
                v_lat = -es2.vel.vx * math.sin(es2.pose.theta) + es2.vel.vy * math.cos(es2.pose.theta)
                print(f"  [debug] t={t:.2f}s e_y={e_y:+.3f}m str={math.degrees(self.steer):+.1f}deg "
                      f"spd={spd:.1f}m/s v_lat={v_lat:.1f}m/s")
                self._debug_frames -= 1

            if step % 100 == 0 and step > 0:
                _min_d = min(self.d1_hist[-1], self.d2_hist[-1])
                print(f"  step={step} t={t:.1f}s e_y={e_y:+.2f}m "
                      f"str={math.degrees(self.steer):+.1f}deg "
                      f"offset={lat_offset:+.1f}m min_d={_min_d:.1f}m "
                      f"state={self.overtake_state}")
            elif step < 10:
                print(f"  step={step} t={t:.1f}s e_y={e_y:+.3f}m "
                      f"str={math.degrees(self.steer):+.1f}deg kappa={kappa_cur:.4f}")

            if t * VX >= self.cl_total_len:
                print(f"\n  → 一圈完成!"); break

        elapsed = time.time() - t_start
        print(f"\n[完成] steps={step+1} engine_steps={self.engine_step} "
              f"fps_render={self.frame_count / max(elapsed, 0.1):.0f}")
        print(f"  超车={self.total_overtakes} 撞墙={self.collision_wall} "
              f"NPC碰撞={self.collision_npc}")
        if not hasattr(self, '_save_path') or not self._save_path:
            input("按 Enter 关闭...")

    # ======================== Render ========================

    def _render(self, t, e_y, steer, ego_x, ego_y, ego_h, lat_offset):
        self.frame_count += 1

        # 读引擎状态
        es = self.world.get_entity_state(self.ego_id)
        ego_x, ego_y, ego_h = es.pose.x, es.pose.y, es.pose.theta
        ns1 = self.world.get_entity_state(self.npc_ids[0])
        ns2 = self.world.get_entity_state(self.npc_ids[1])

        # 更新艺术家
        self.trail_ego.set_data(self.ego_trail_x, self.ego_trail_y)
        self.ego_patch.set_xy(_vehicle_corners(ego_x, ego_y, ego_h,
                               VEHICLE_HW, VEHICLE_FWD, VEHICLE_REV))
        self.trail_n1.set_data(self.npc1_trail_x, self.npc1_trail_y)
        self.trail_n2.set_data(self.npc2_trail_x, self.npc2_trail_y)
        self.n1_patch.set_xy(_vehicle_corners(ns1.pose.x, ns1.pose.y, ns1.pose.theta,
                               NPC_HW, NPC_FWD, NPC_REV))
        self.n2_patch.set_xy(_vehicle_corners(ns2.pose.x, ns2.pose.y, ns2.pose.theta,
                               NPC_HW, NPC_FWD, NPC_REV))

        # 视角
        self.ax_map.set_xlim(ego_x - ZOOM, ego_x + ZOOM)
        self.ax_map.set_ylim(ego_y - ZOOM, ego_y + ZOOM)
        # 从引擎实时计算 e_y (而非用 MPC 步进前的旧值)
        ref = self.traj.get_state(self.local_s)
        ref_x, ref_y, ref_psi = float(ref[0]), float(ref[1]), float(ref[2])
        dx_r = ego_x - ref_x; dy_r = ego_y - ref_y
        e_y_rt = -dx_r * math.sin(ref_psi) + dy_r * math.cos(ref_psi)
        e_psi_rt = ego_h - ref_psi
        e_psi_rt = (e_psi_rt + math.pi) % (2 * math.pi) - math.pi

        self.info.set_text(
            f"t={self.t_hist[-1] if self.t_hist else 0:.1f}s  "
            f"e_y={e_y_rt:+.2f}m  steer={math.degrees(self.steer):+.0f}deg  "
            f"state={self.overtake_state}\n"
            f"wall={'Y' if self.collision_wall else 'N'}  "
            f"npc={'Y' if self.collision_npc else 'N'}  "
            f"overtakes={self.total_overtakes}")

        # 时间序列
        if len(self.t_hist) > 1:
            self.dn1.set_data(self.t_hist, self.d1_hist)
            self.dn2.set_data(self.t_hist, self.d2_hist)
            self.ey_ln.set_data(self.t_hist, self.ey_hist)
            self.ep_ln.set_data(self.t_hist, self.ep_hist)
            self.st_ln.set_data(self.t_hist, self.st_hist)
            self.ax_dn.set_xlim(max(0, self.t_hist[-1] - 20),
                                max(20, self.t_hist[-1] + 2))

        # 刷新画布
        self.fig.canvas.draw()
        plt.pause(0.001)


# =====================================================================
#  Entry
# =====================================================================

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--time", type=float, default=0)
    args = ap.parse_args()
    sim = LiveSim()
    sim.setup()
    sim.run(max_time=args.time)
