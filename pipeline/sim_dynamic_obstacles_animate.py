"""
sim_dynamic_obstacles_animate.py — 动态障碍物场景仿真动画

读取仿真日志 + NPC 轨迹 + 赛道边界，还原自车与 NPC 的运动过程。

用法:
  python pipeline/sim_dynamic_obstacles_animate.py
  python pipeline/sim_dynamic_obstacles_animate.py --save output/animation.gif
"""

from __future__ import annotations

import os, sys, math, argparse
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.path import Path as MplPath
from matplotlib.patches import PathPatch

_self_dir = os.path.dirname(os.path.abspath(__file__))
_root = os.path.dirname(_self_dir)
if _root not in sys.path:
    sys.path.insert(0, _root)

from pipeline.sim_dynamic_obstacles import Trajectory

# 车辆几何 (与仿真一致)
LF, LR    = 1.1, 1.58
L_WB      = LF + LR
FWD, REV  = 0.8, 0.5       # 后轴到车头/车尾
HW        = 0.45            # 车半宽 (全宽 0.9m ≈ 赛道1/4)
NPC_FWD   = 1.5             # NPC 车辆尺寸
NPC_REV   = 1.0
NPC_HW    = 1.0
WHL_W     = 0.42
WHL_L     = 0.85
ZOOM      = 25.0            # 主视角放大范围 (m)


# ============================================================
#  Data loading
# ============================================================

def load_sim_data(filepath):
    with open(filepath) as f:
        header = f.readline()
    data = np.loadtxt(filepath, delimiter="\t", skiprows=2)
    return data, header


def load_boundaries():
    outer = np.load("output/sim_dynamic_obstacles_outer.npy")
    holes = []
    i = 0
    while os.path.exists(f"output/sim_dynamic_obstacles_hole_{i}.npy"):
        holes.append(np.load(f"output/sim_dynamic_obstacles_hole_{i}.npy"))
        i += 1
    return outer, holes


def load_npc_data():
    path = "output/sim_dynamic_obstacles_npc.npy"
    if os.path.exists(path):
        data = np.load(path)
        # 空数据或无NPC场景返回None
        if data.ndim < 3 or data.shape[1] == 0:
            return None
        return data  # shape: (N_steps, 2, 3) — [step, npc_idx, (x, y, heading)]
    return None


def load_obs_data():
    """加载动态障碍物参数和时间轴。返回 (N_obs, 5) 数组。"""
    obs_path = "output/sim_dynamic_obstacles_obs.npy"
    t_path = "output/sim_dynamic_obstacles_obs_t.npy"
    if os.path.exists(obs_path) and os.path.exists(t_path):
        obs = np.load(obs_path)  # (N_obs, 5): [x_ref, y_ref, h_ref, amp, period]
        t_arr = np.load(t_path)
        if obs.ndim == 1:
            obs = obs.reshape(1, -1)
        return obs, t_arr
    return None, None


def obs_position_at(obs_params, t):
    """计算障碍物在时刻 t 的世界坐标 (x, y)。"""
    x_ref, y_ref, h_ref, amp, period = obs_params[:5]
    phase = obs_params[5] if len(obs_params) > 5 else 0.0
    lat = amp * math.sin(2.0 * math.pi * t / period + phase)
    nx, ny = -math.sin(h_ref), math.cos(h_ref)  # 法向量
    return x_ref + lat * nx, y_ref + lat * ny


# ============================================================
#  Vehicle rendering
# ============================================================

def _vehicle_parts(cx, cy, psi, steer, hw, fwd, rev, body_color, body_edge):
    """返回车辆渲染部件列表 [(verts, fc, ec, lw), ...]"""
    cos_p, sin_p = math.cos(psi), math.sin(psi)
    R = np.array([[cos_p, -sin_p], [sin_p, cos_p]])
    rx, ry = cx - LR * cos_p, cy - LR * sin_p  # 后轴

    fwx = rx + fwd * 0.7 * cos_p
    fwy = ry + fwd * 0.7 * sin_p
    rwx = rx - rev * 0.5 * cos_p
    rwy = ry - rev * 0.5 * sin_p

    parts = []
    # 车身
    body = np.array([[fwd, hw], [fwd, -hw], [-rev, -hw], [-rev, hw]])
    parts.append((body @ R.T + [rx, ry], body_color, body_edge, 1.5))
    # 挡风玻璃
    ww = hw * 0.76
    ws = np.array([[fwd, ww], [fwd - 0.25, ww], [fwd - 0.25, -ww], [fwd, -ww]])
    parts.append((ws @ R.T + [rx, ry], "#a8dadc", "#457b9d", 0.6))
    # 后窗
    rw = np.array([[-rev + 0.2, ww], [-rev, ww], [-rev, -ww], [-rev + 0.2, -ww]])
    parts.append((rw @ R.T + [rx, ry], "#a8dadc", "#457b9d", 0.6))
    # 车轮
    for ax, ay, angle in [(rwx, rwy, psi), (fwx, fwy, psi + steer)]:
        ca, sa = math.cos(angle), math.sin(angle)
        Rw = np.array([[ca, -sa], [sa, ca]])
        wh = np.array([[WHL_L / 2, WHL_W / 2], [WHL_L / 2, -WHL_W / 2],
                       [-WHL_L / 2, -WHL_W / 2], [-WHL_L / 2, WHL_W / 2]])
        parts.append((wh @ Rw.T + [ax, ay], "#1d3557", "#000", 0.8))
    return parts


def _npc_rect(cx, cy, psi, hw, fwd, rev):
    """NPC 简单矩形 (不画车轮, 与仿真可视化一致)"""
    cos_p, sin_p = math.cos(psi), math.sin(psi)
    R = np.array([[cos_p, -sin_p], [sin_p, cos_p]])
    rx, ry = cx - LR * cos_p, cy - LR * sin_p
    rect = np.array([[fwd, hw], [fwd, -hw], [-rev, -hw], [-rev, hw]])
    return rect @ R.T + [rx, ry]


# ============================================================
#  Main
# ============================================================

def run():
    ap = argparse.ArgumentParser(description="动态障碍物场景动画")
    ap.add_argument("--file", default="output/sim_dynamic_obstacles.txt")
    ap.add_argument("--save", default=None, help="保存为 GIF")
    ap.add_argument("--interval", type=int, default=25, help="帧间隔 (ms)")
    ap.add_argument("--skip", type=int, default=2, help="每隔 N 步取一帧")
    ap.add_argument("--speed", type=float, default=1.0, help="播放速度倍率")
    ap.add_argument("--full", action="store_true", help="固定全局视角")
    args = ap.parse_args()

    print("Loading simulation data ...")
    data, header = load_sim_data(args.file)
    N = len(data)
    dt = 0.1; Vx = 10.0
    t_arr  = data[:, 1]
    e_y    = data[:, 2]
    e_psi  = data[:, 3]
    steer  = data[:, 4]
    ego_wx = data[:, 5]
    ego_wy = data[:, 6]
    ego_wh = data[:, 7]

    print("Loading trajectory & boundaries & NPCs & Obstacle ...")
    outer, holes = load_boundaries()
    npc_data = load_npc_data()
    obs_params, obs_t_arr = load_obs_data()
    has_obs = obs_params is not None

    # 加载轨迹
    traj_path = "output/sim_dynamic_obstacles_traj.npy"
    if os.path.exists(traj_path):
        traj_pts = np.load(traj_path)
        traj = Trajectory(traj_pts)
    else:
        traj = None

    skip = max(1, args.skip)
    nf = (N + skip - 1) // skip
    indices = [min(i * skip, N - 1) for i in range(nf)]

    interval_ms = max(10, int(1000 * dt * skip / args.speed))

    # ---- Precompute Ego frames ----
    print(f"Precomputing {nf} frames ...")

    ego_parts_frames = []
    for fi in range(nf):
        idx = indices[fi]
        cx, cy = ego_wx[idx], ego_wy[idx]
        psi = ego_wh[idx]; st = steer[idx]
        parts = _vehicle_parts(cx, cy, psi, st, HW, FWD, REV, "#e63946", "#c1121f")
        ego_parts_frames.append(parts)

    # NPC 位置 (每帧)
    npc_frames = []
    if npc_data is not None:
        for fi in range(nf):
            idx = min(indices[fi], npc_data.shape[0] - 1)
            npc1 = npc_data[idx, 0]  # [x, y, heading]
            npc2 = npc_data[idx, 1]
            npc_frames.append((npc1, npc2))

    # 轨迹线
    trail_x = [ego_wx[:i+1] for i in indices]
    trail_y = [ego_wy[:i+1] for i in indices]

    times  = [t_arr[i]  for i in indices]
    eys    = [e_y[i]    for i in indices]
    eps    = [np.degrees(e_psi[i]) for i in indices]
    strs   = [np.degrees(steer[i]) for i in indices]
    steps  = [i * Vx * dt for i in indices]

    # 总览边界
    ov_x_min, ov_x_max = outer[:, 0].min(), outer[:, 0].max()
    ov_y_min, ov_y_max = outer[:, 1].min(), outer[:, 1].max()
    ov_pad = max(ov_x_max - ov_x_min, ov_y_max - ov_y_min) * 0.06

    print(f"  Done. {nf} frames, interval={interval_ms}ms.")

    # ======================== Figure ========================
    from matplotlib.gridspec import GridSpec

    fig = plt.figure(figsize=(20, 12))
    fig.suptitle("Dynamic Obstacles — Lane Keeping + NPC Tracking", fontsize=14)

    gs = GridSpec(4, 2, figure=fig, width_ratios=[3, 1],
                  hspace=0.35, wspace=0.25,
                  left=0.04, right=0.96, top=0.93, bottom=0.06)

    # ---- 主地图 (左列, 跨 4 行) ----
    ax_map = fig.add_subplot(gs[:, 0])
    ax_map.set_aspect("equal"); ax_map.grid(True, alpha=0.2)
    ax_map.set_xlabel("X (m)"); ax_map.set_ylabel("Y (m)")

    # 赛道边界 + 孔洞
    ax_map.plot(outer[:, 0], outer[:, 1], "#444", lw=1.8, alpha=0.6, label="Track")
    for h in holes:
        ax_map.fill(h[:, 0], h[:, 1], fc="white", ec="#444", lw=0.5, alpha=0.95)

    # 参考轨迹 (如有)
    if traj is not None:
        ax_map.plot(traj.points[:, 0], traj.points[:, 1], "b--", lw=0.6, alpha=0.35, label="Reference")
    ax_map.legend(loc="lower left", fontsize=7)
    dyn_trail, = ax_map.plot([], [], "r-", lw=1.5, alpha=0.55)

    # NPC 轨迹线
    dyn_npc1, = ax_map.plot([], [], "orange", lw=1.2, alpha=0.7, ls="--")
    dyn_npc2, = ax_map.plot([], [], "purple", lw=1.2, alpha=0.7, ls="--")

    # Ego 车身 patches
    ego_patches = []
    for _ in range(6):
        p = MplPolygon([[0,0],[1,0],[1,1],[0,1]], closed=True, fc="red", ec="red", lw=1, zorder=5)
        ax_map.add_patch(p); ego_patches.append(p)

    # NPC 矩形 patches (每个 NPC 一个)
    npc1_patch = MplPolygon([[0,0],[1,0],[1,1],[0,1]], closed=True,
                             fc="orange", ec="darkorange", lw=1, alpha=0.6, zorder=4)
    npc2_patch = MplPolygon([[0,0],[1,0],[1,1],[0,1]], closed=True,
                             fc="purple", ec="darkviolet", lw=1, alpha=0.6, zorder=4)
    ax_map.add_patch(npc1_patch); ax_map.add_patch(npc2_patch)

    # 动态障碍物 (地图上的部分)
    obs_patches = []; dyn_obs_dots = []; ov_obs_dots = []
    obs_colors = ['orange', 'cyan']
    if has_obs:
        n_obs = obs_params.shape[0]
        for oi in range(n_obs):
            x_ref, y_ref, h_ref, amp = obs_params[oi, :4]
            c = obs_colors[oi % len(obs_colors)]
            nx, ny = -math.sin(h_ref), math.cos(h_ref)
            ax_map.plot([x_ref - amp*nx, x_ref + amp*nx],
                        [y_ref - amp*ny, y_ref + amp*ny],
                        color=c, lw=1.5, ls='--', alpha=0.4)
            ax_map.plot(x_ref, y_ref, 'D', color=c, ms=8, alpha=0.6)
            obs_patches.append(MplPolygon([[0,0],[1,0],[1,1],[0,1]], closed=True,
                                fc=c, ec=c, lw=1.5, alpha=0.7, zorder=6))
            ax_map.add_patch(obs_patches[-1])
            d, = ax_map.plot([], [], 'o', color=c, ms=6, zorder=6)
            dyn_obs_dots.append(d)

    info_txt = ax_map.text(0.015, 0.975, "", transform=ax_map.transAxes,
                           fontsize=9, va="top", family="monospace",
                           bbox=dict(boxstyle="round", fc="lightyellow", alpha=0.85))
    ax_map.legend(loc="lower left", fontsize=7)

    # ---- 总览小窗 (右上角) ----
    ax_ov = ax_map.inset_axes([0.62, 0.60, 0.36, 0.38])
    ax_ov.set_aspect("equal"); ax_ov.set_xticks([]); ax_ov.set_yticks([])
    ax_ov.patch.set_facecolor("#f8f8f8"); ax_ov.patch.set_alpha(0.92)
    for spine in ax_ov.spines.values():
        spine.set_color("#333"); spine.set_linewidth(1.5)
    ax_ov.plot(outer[:, 0], outer[:, 1], "#444", lw=1.0, alpha=0.7)
    for h in holes:
        ax_ov.fill(h[:, 0], h[:, 1], fc="white", ec="#444", lw=0.3, alpha=0.95)
    if traj is not None:
        ax_ov.plot(traj.points[:, 0], traj.points[:, 1], "b--", lw=0.4, alpha=0.4)
    ov_trail, = ax_ov.plot([], [], "r-", lw=0.8, alpha=0.6)
    ov_dot,   = ax_ov.plot([], [], "ro", ms=6, mec="#800", mew=0.8)
    ov_npc1,  = ax_ov.plot([], [], "o", color="orange", ms=4, alpha=0.7)
    ov_npc2,  = ax_ov.plot([], [], "o", color="purple", ms=4, alpha=0.7)
    if has_obs:
        for oi in range(obs_params.shape[0]):
            c = obs_colors[oi % len(obs_colors)]
            d, = ax_ov.plot([], [], 's', color=c, ms=5, alpha=0.8)
            ov_obs_dots.append(d)
    ax_ov.set_xlim(ov_x_min - ov_pad, ov_x_max + ov_pad)
    ax_ov.set_ylim(ov_y_min - ov_pad, ov_y_max + ov_pad)

    # ---- 右面板: 4 子图竖排 ----
    # NPC 距离 (行 0, 列 1)
    ax_dn = fig.add_subplot(gs[0, 1])
    ax_dn.set_title("Ego-NPC Distance"); ax_dn.set_ylabel("m")
    ax_dn.grid(True, alpha=0.3); ax_dn.axhline(2.5, color="red", ls=":", lw=0.8, alpha=0.5)
    ax_dn.set_xlim(t_arr[0], t_arr[-1])
    dn_line1, = ax_dn.plot([], [], "orange", lw=1.0, alpha=0.7, label="NPC 1")
    dn_line2, = ax_dn.plot([], [], "purple", lw=1.0, alpha=0.7, label="NPC 2")
    dn_cur1,  = ax_dn.plot([], [], "o", color="orange", ms=4)
    dn_cur2,  = ax_dn.plot([], [], "o", color="purple", ms=4)
    ax_dn.legend(fontsize=7)

    # 横向误差 (行 1, 列 1, sharex with dn)
    ax_ey = fig.add_subplot(gs[1, 1], sharex=ax_dn)
    ax_ey.set_title("Lateral Error e_y"); ax_ey.set_ylabel("m")
    ax_ey.grid(True, alpha=0.3); ax_ey.axhline(0, color="k", ls="--", lw=0.5)
    eymin = min(-1.5, float(np.min(e_y)) - 0.3)
    eymax = max( 1.5, float(np.max(e_y)) + 0.3)
    ax_ey.set_ylim(eymin, eymax)
    ey_line, = ax_ey.plot([], [], "b-", lw=1.2)
    ey_cur,  = ax_ey.plot([], [], "ro", ms=5)

    # 航向误差 (行 2, 列 1)
    ax_ep = fig.add_subplot(gs[2, 1], sharex=ax_dn)
    ax_ep.set_title("Heading Error e_psi"); ax_ep.set_ylabel("deg")
    ax_ep.grid(True, alpha=0.3); ax_ep.axhline(0, color="k", ls="--", lw=0.5)
    ax_ep.set_ylim(-25, 25)
    ep_line, = ax_ep.plot([], [], "r-", lw=1.2)
    ep_cur,  = ax_ep.plot([], [], "ro", ms=5)

    # 转向角 (行 3, 列 1)
    ax_st = fig.add_subplot(gs[3, 1], sharex=ax_dn)
    ax_st.set_title("Steering Angle"); ax_st.set_xlabel("Time (s)")
    ax_st.set_ylabel("deg"); ax_st.grid(True, alpha=0.3)
    smax = max(abs(np.degrees(steer)).max() + 5, 35)
    ax_st.set_ylim(-smax, smax)
    st_line, = ax_st.plot([], [], "g-", lw=1.2)
    st_cur,  = ax_st.plot([], [], "go", ms=5)

    # 隐藏上方子图的 x 轴标签 (除最底)
    plt.setp(ax_dn.get_xticklabels(), visible=False)
    plt.setp(ax_ey.get_xticklabels(), visible=False)
    plt.setp(ax_ep.get_xticklabels(), visible=False)

    # ---- Precompute NPC distance histories ----
    if npc_data is not None:
        npc1_x_all = npc_data[:N, 0, 0]; npc1_y_all = npc_data[:N, 0, 1]
        npc2_x_all = npc_data[:N, 1, 0]; npc2_y_all = npc_data[:N, 1, 1]
        d1_all = np.sqrt((ego_wx - npc1_x_all)**2 + (ego_wy - npc1_y_all)**2)
        d2_all = np.sqrt((ego_wx - npc2_x_all)**2 + (ego_wy - npc2_y_all)**2)
        # NPC 轨迹线
        npc1_trail_x = [npc1_x_all[:i+1] for i in indices]
        npc1_trail_y = [npc1_y_all[:i+1] for i in indices]
        npc2_trail_x = [npc2_x_all[:i+1] for i in indices]
        npc2_trail_y = [npc2_y_all[:i+1] for i in indices]
    else:
        d1_all = np.zeros(N); d2_all = np.zeros(N)
        npc1_trail_x = [[] for _ in indices]
        npc1_trail_y = [[] for _ in indices]
        npc2_trail_x = [[] for _ in indices]
        npc2_trail_y = [[] for _ in indices]

    # ---- Update function ----
    def update(frame):
        fi = frame; idx = indices[fi]

        # Ego 轨迹
        dyn_trail.set_data(trail_x[fi], trail_y[fi])

        # Ego 车身
        parts = ego_parts_frames[fi]
        for j, (verts, fc, ec, lw) in enumerate(parts):
            if j < len(ego_patches):
                ego_patches[j].set_xy(verts)
                ego_patches[j].set_facecolor(fc)
                ego_patches[j].set_edgecolor(ec)
                ego_patches[j].set_linewidth(lw)
        for j in range(len(parts), len(ego_patches)):
            ego_patches[j].set_xy([[0,0],[0,0],[0,0],[0,0]])

        # NPC 轨迹 & 矩形
        if npc_data is not None and len(npc_frames) > fi:
            npc1, npc2 = npc_frames[fi]
            dyn_npc1.set_data(npc1_trail_x[fi], npc1_trail_y[fi])
            dyn_npc2.set_data(npc2_trail_x[fi], npc2_trail_y[fi])
            npc1_patch.set_xy(_npc_rect(npc1[0], npc1[1], npc1[2], NPC_HW, NPC_FWD, NPC_REV))
            npc2_patch.set_xy(_npc_rect(npc2[0], npc2[1], npc2[2], NPC_HW, NPC_FWD, NPC_REV))
            ov_npc1.set_data([npc1[0]], [npc1[1]])
            ov_npc2.set_data([npc2[0]], [npc2[1]])

        # 动态障碍物
        if has_obs:
            t_f = obs_t_arr[min(idx, len(obs_t_arr) - 1)]
            for oi in range(obs_params.shape[0]):
                ox, oy = obs_position_at(obs_params[oi], t_f)
                hw = 0.5
                h_ref = obs_params[oi, 2]
                c, s = math.cos(h_ref), math.sin(h_ref)
                lx = [hw, hw, -hw, -hw]
                ly = [hw, -hw, -hw, hw]
                corners = [(ox + lx[i]*c - ly[i]*s, oy + lx[i]*s + ly[i]*c) for i in range(4)]
                if oi < len(obs_patches):
                    obs_patches[oi].set_xy(corners)
                if oi < len(dyn_obs_dots):
                    dyn_obs_dots[oi].set_data([ox], [oy])
                if oi < len(ov_obs_dots):
                    ov_obs_dots[oi].set_data([ox], [oy])

        # 主视角
        cx, cy = ego_wx[idx], ego_wy[idx]
        if not args.full:
            ax_map.set_xlim(cx - ZOOM, cx + ZOOM)
            ax_map.set_ylim(cy - ZOOM, cy + ZOOM)

        # 信息
        info_txt.set_text(
            f"t={times[fi]:.1f}s  s={steps[fi]:.0f}m\n"
            f"e_y={eys[fi]:+.3f}m  e_psi={eps[fi]:+.1f}deg  steer={strs[fi]:+.1f}deg")

        # 总览
        ov_trail.set_data(ego_wx[:idx+1], ego_wy[:idx+1])
        ov_dot.set_data([cx], [cy])

        # 误差曲线
        n_data = idx + 1
        ey_line.set_data(t_arr[:n_data], e_y[:n_data]); ey_cur.set_data([times[fi]], [eys[fi]])
        ep_line.set_data(t_arr[:n_data], np.degrees(e_psi[:n_data])); ep_cur.set_data([times[fi]], [eps[fi]])
        st_line.set_data(t_arr[:n_data], np.degrees(steer[:n_data])); st_cur.set_data([times[fi]], [strs[fi]])

        # NPC 距离
        dn_line1.set_data(t_arr[:n_data], d1_all[:n_data]); dn_cur1.set_data([times[fi]], [d1_all[idx]])
        dn_line2.set_data(t_arr[:n_data], d2_all[:n_data]); dn_cur2.set_data([times[fi]], [d2_all[idx]])

        ret = [dyn_trail, dyn_npc1, dyn_npc2, info_txt,
                 ey_line, ey_cur, ep_line, ep_cur, st_line, st_cur,
                 dn_line1, dn_cur1, dn_line2, dn_cur2,
                 ov_trail, ov_dot, ov_npc1, ov_npc2, npc1_patch, npc2_patch]
        if has_obs:
            ret.extend(obs_patches + dyn_obs_dots + ov_obs_dots)
        ret.extend(ego_patches)
        return ret

    fps = max(1, int(round(1000 / interval_ms))) if interval_ms > 0 else 5
    ani = FuncAnimation(fig, update, frames=nf, interval=interval_ms, blit=False)

    if args.save:
        print(f"Saving {args.save} ({nf} frames, fps={fps}) ...")
        ani.save(args.save, writer="pillow", fps=fps, dpi=100)
        print("Done!")
    else:
        plt.show()


if __name__ == "__main__":
    run()
