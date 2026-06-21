"""
sim_trajectory_optimization_animate.py — 轨迹优化仿真动画

读取仿真日志 + 轨迹点 + 赛道边界, 还原车辆运动过程。

用法:
  python pipeline/sim_trajectory_optimization_animate.py                # 交互播放
  python pipeline/sim_trajectory_optimization_animate.py --save output/animation.gif
"""

from __future__ import annotations

import os, sys, math, argparse
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon as MplPolygon

_self_dir = os.path.dirname(os.path.abspath(__file__))
_root = os.path.dirname(_self_dir)
if _root not in sys.path:
    sys.path.insert(0, _root)

from pipeline.sim_trajectory_optimization import Trajectory

# Geometry (与碰撞检测一致)
LF, LR = 1.1, 1.58
L_WB = LF + LR
FWD, REV = 1.5, 1.0    # 后轴到车头/车尾 (总长 2.5m)
HW = 1.0                # 车半宽 (全宽 2.0m)
COLLISION_HW = HW + 0.2 # 碰撞半宽 = 车半宽 + 安全边距
WHL_W = 0.42
WHL_L = 0.85
ZOOM = 20.0

# ============================================================
#  Data loading
# ============================================================

def load_sim_data(filepath):
    with open(filepath) as f:
        lines = f.readlines()
    hdr = {}
    if lines[0].startswith("# REF:"):
        for kv in lines[0].replace("# REF:", "").split():
            if "=" in kv:
                k, v = kv.split("=", 1)
                try:    hdr[k] = float(v)
                except: hdr[k] = v
    return hdr, np.loadtxt(filepath, delimiter="\t", skiprows=2)


def load_boundaries():
    """从 pipeline 输出加载赛道边界."""
    outer = np.load("output/sim_trajectory_optimization_outer.npy")
    holes = [np.load(f"output/sim_trajectory_optimization_hole_{i}.npy")
             for i in range(3)]  # path2.png 有 3 个孔洞
    return outer, holes


def load_trajectory():
    pts = np.load("output/sim_trajectory_optimization_traj.npy")
    return Trajectory(pts), pts


# ============================================================
#  Precomputation
# ============================================================

def precompute_car_polys(nf, skip, car_x, car_y, psi_car, steer):
    frames = []
    for fi in range(nf):
        idx = min(fi * skip, len(car_x) - 1)
        cx, cy = car_x[idx], car_y[idx]
        ps, st = psi_car[idx], steer[idx]

        cos_p, sin_p = math.cos(ps), math.sin(ps)
        R = np.array([[cos_p, -sin_p], [sin_p, cos_p]])
        rx, ry = cx - LR*cos_p, cy - LR*sin_p  # 后轴 (碰撞盒原点)
        fwx = rx + FWD * 0.7 * cos_p             # 前轮: 距车头 0.45m
        fwy = ry + FWD * 0.7 * sin_p
        rwx = rx - REV * 0.5 * cos_p             # 后轮: 距车尾 0.5m
        rwy = ry - REV * 0.5 * sin_p

        parts = []
        # 车身 (以后轴为中心: 前 FWD=1.5, 后 REV=1.0, 宽 HW=1.0)
        body = np.array([[FWD, HW], [FWD, -HW], [-REV, -HW], [-REV, HW]])
        parts.append((body @ R.T + [rx, ry], "#e63946", "#c1121f", 1.5))
        # 挡风玻璃
        ww = HW * 0.76
        ws = np.array([[FWD, ww], [FWD-0.2, ww], [FWD-0.2, -ww], [FWD, -ww]])
        parts.append((ws @ R.T + [rx, ry], "#a8dadc", "#457b9d", 0.6))
        # 后窗
        rw = np.array([[-REV+0.15, ww], [-REV, ww], [-REV, -ww], [-REV+0.15, -ww]])
        parts.append((rw @ R.T + [rx, ry], "#a8dadc", "#457b9d", 0.6))
        # 车轮
        for ax, ay, angle in [(rwx, rwy, ps), (fwx, fwy, ps + st)]:
            ca, sa = math.cos(angle), math.sin(angle)
            Rw = np.array([[ca, -sa], [sa, ca]])
            wh = np.array([[WHL_L/2, WHL_W/2], [WHL_L/2, -WHL_W/2],
                           [-WHL_L/2, -WHL_W/2], [-WHL_L/2, WHL_W/2]])
            parts.append((wh @ Rw.T + [ax, ay], "#1d3557", "#000", 0.8))
        frames.append(parts)
    return frames


# ============================================================
#  Main
# ============================================================

def run():
    ap = argparse.ArgumentParser(description="轨迹优化动画")
    ap.add_argument("--file", default="output/sim_trajectory_optimization.txt")
    ap.add_argument("--save", default=None, help="保存为 GIF")
    ap.add_argument("--interval", type=int, default=25, help="帧间隔 (ms)")
    ap.add_argument("--skip", type=int, default=2, help="每隔 N 步取一帧")
    ap.add_argument("--speed", type=float, default=1.0, help="播放速度倍率")
    ap.add_argument("--full", action="store_true", help="固定全局视角")
    args = ap.parse_args()

    print("Loading simulation data ...")
    header, data = load_sim_data(args.file)
    dt = header.get("dt", 0.1)
    Vx = header.get("Vx", 10.0)
    N = len(data)
    t_arr = data[:, 1]
    e_y = data[:, 2]
    e_psi = data[:, 4]
    steer = data[:, 6]

    print("Loading trajectory & boundaries ...")
    traj, waypoints = load_trajectory()
    outer, holes = load_boundaries()

    skip = max(1, args.skip)
    nf = (N + skip - 1) // skip
    indices = [min(i * skip, N - 1) for i in range(nf)]

    interval = max(10, int(1000 * dt * skip / args.speed))
    sim_s_per_frame = skip * dt
    wall_s_per_frame = interval / 1000.0
    speedup = sim_s_per_frame / wall_s_per_frame if wall_s_per_frame > 0 else 1.0

    # ---- Precompute all frame data ----
    print(f"Precomputing {nf} frames ...")

    x_ref = np.zeros(N); y_ref = np.zeros(N); psi_ref = np.zeros(N)
    for i in range(N):
        rs = traj.get_state(i * Vx * dt)
        x_ref[i], y_ref[i], psi_ref[i] = rs[0], rs[1], rs[2]

    car_x = x_ref - e_y * np.sin(psi_ref)
    car_y = y_ref + e_y * np.cos(psi_ref)
    psi_car = psi_ref + e_psi
    psi_car = np.array([(p + math.pi) % (2*math.pi) - math.pi for p in psi_car])

    # 碰撞边界线 (参考轨迹 ± 碰撞半宽)
    bl_lx = x_ref - COLLISION_HW * np.sin(psi_ref)
    bl_ly = y_ref + COLLISION_HW * np.cos(psi_ref)
    bl_rx = x_ref + COLLISION_HW * np.sin(psi_ref)
    bl_ry = y_ref - COLLISION_HW * np.cos(psi_ref)

    ep_deg = np.degrees(e_psi)
    st_deg = np.degrees(steer)

    car_frames = precompute_car_polys(nf, skip, car_x, car_y, psi_car, steer)

    trail_x_frames = [car_x[:i+1] for i in indices]
    trail_y_frames = [car_y[:i+1] for i in indices]
    ref_x_frames  = [x_ref[:i+1]  for i in indices]
    ref_y_frames  = [y_ref[:i+1]  for i in indices]
    blx_frames = [bl_lx[:i+1] for i in indices]
    bly_frames = [bl_ly[:i+1] for i in indices]
    brx_frames = [bl_rx[:i+1] for i in indices]
    bry_frames = [bl_ry[:i+1] for i in indices]

    times = [t_arr[i] for i in indices]
    eys   = [e_y[i]    for i in indices]
    eps   = [ep_deg[i] for i in indices]
    strs  = [st_deg[i] for i in indices]
    steps = [i * Vx * dt for i in indices]

    # Overview bounding box
    ov_x_min, ov_x_max = outer[:,0].min(), outer[:,0].max()
    ov_y_min, ov_y_max = outer[:,1].min(), outer[:,1].max()
    ov_pad = max(ov_x_max - ov_x_min, ov_y_max - ov_y_min) * 0.06

    print(f"  Done. {nf} frames, track={traj.total_len:.0f}m, "
          f"interval={interval}ms (speed={speedup:.1f}x).")

    # ======================== Figure ========================
    fig = plt.figure(figsize=(18, 10))
    fig.suptitle("Trajectory Optimization — Hybrid A* + B-Spline + MPC",
                 fontsize=14)

    # ---- Main map ----
    ax_map = fig.add_subplot(1, 2, 1)
    ax_map.set_aspect("equal")
    ax_map.grid(True, alpha=0.2)
    ax_map.set_xlabel("X (m)"); ax_map.set_ylabel("Y (m)")

    # Static backdrop: 赛道
    ax_map.plot(outer[:,0], outer[:,1], "#444", lw=1.8, alpha=0.6)
    for h in holes:
        ax_map.fill(h[:,0], h[:,1], fc="white", ec="#444", lw=0.5, alpha=0.95)
    # 优化轨迹 (灰虚线背景)
    ax_map.plot(waypoints[:,0], waypoints[:,1], "b--", lw=0.6, alpha=0.35)

    # Dynamic lines
    dyn_ref,   = ax_map.plot([], [], "b-", lw=1.8, alpha=0.55)
    dyn_bl,    = ax_map.plot([], [], "gray", lw=0.5, ls=":", alpha=0.45)
    dyn_br,    = ax_map.plot([], [], "gray", lw=0.5, ls=":", alpha=0.45)
    dyn_trail, = ax_map.plot([], [], "r-", lw=1.5, alpha=0.55)

    # Car patches
    car_patches: list[MplPolygon] = []
    for _ in range(6):
        p = MplPolygon([[0,0],[1,0],[1,1],[0,1]], closed=True,
                       fc="red", ec="red", lw=1, zorder=5)
        ax_map.add_patch(p)
        car_patches.append(p)

    info_txt = ax_map.text(0.015, 0.975, "", transform=ax_map.transAxes,
                           fontsize=9, va="top", family="monospace",
                           bbox=dict(boxstyle="round", fc="lightyellow", alpha=0.85))

    # ---- Overview inset (top-right) ----
    ax_ov = ax_map.inset_axes([0.60, 0.60, 0.38, 0.38])
    ax_ov.set_aspect("equal")
    ax_ov.set_xticks([]); ax_ov.set_yticks([])
    ax_ov.patch.set_facecolor("#f8f8f8")
    ax_ov.patch.set_alpha(0.92)
    for spine in ax_ov.spines.values():
        spine.set_color("#333"); spine.set_linewidth(1.5)

    ax_ov.plot(outer[:,0], outer[:,1], "#444", lw=1.0, alpha=0.7)
    for h in holes:
        ax_ov.fill(h[:,0], h[:,1], fc="white", ec="#444", lw=0.3, alpha=0.95)
    ax_ov.plot(waypoints[:,0], waypoints[:,1], "b--", lw=0.4, alpha=0.4)
    ax_ov.set_xlim(ov_x_min - ov_pad, ov_x_max + ov_pad)
    ax_ov.set_ylim(ov_y_min - ov_pad, ov_y_max + ov_pad)

    ov_trail, = ax_ov.plot([], [], "r-", lw=0.8, alpha=0.6)
    ov_dot,   = ax_ov.plot([], [], "ro", ms=6, mec="#800", mew=0.8)
    ov_label  = ax_ov.text(0.5, 0.97, "OVERVIEW", transform=ax_ov.transAxes,
                           fontsize=7, ha="center", va="top", weight="bold",
                           color="#555")

    # ---- Right panel: error plots ----
    ax_ey = fig.add_subplot(3, 2, 2)
    ax_ey.set_title("Lateral Error e_y"); ax_ey.set_ylabel("m")
    ax_ey.grid(True, alpha=0.3); ax_ey.axhline(0, color="k", ls="--", lw=0.5)
    ax_ey.set_xlim(t_arr[0], t_arr[-1]); ax_ey.set_ylim(-1.2, 1.2)
    ey_line, = ax_ey.plot([], [], "b-", lw=1.2)
    ey_cur,  = ax_ey.plot([], [], "ro", ms=5)

    ax_ep = fig.add_subplot(3, 2, 4)
    ax_ep.set_title("Heading Error e_psi"); ax_ep.set_ylabel("deg")
    ax_ep.grid(True, alpha=0.3); ax_ep.axhline(0, color="k", ls="--", lw=0.5)
    ax_ep.set_xlim(t_arr[0], t_arr[-1]); ax_ep.set_ylim(-25, 25)
    ep_line, = ax_ep.plot([], [], "r-", lw=1.2)
    ep_cur,  = ax_ep.plot([], [], "ro", ms=5)

    ax_st = fig.add_subplot(3, 2, 6)
    ax_st.set_title("Steering Angle"); ax_st.set_xlabel("Time (s)")
    ax_st.set_ylabel("deg"); ax_st.grid(True, alpha=0.3)
    ax_st.set_xlim(t_arr[0], t_arr[-1])
    smax = max(abs(st_deg)) + 5
    ax_st.set_ylim(-smax, smax)
    st_line, = ax_st.plot([], [], "g-", lw=1.2)
    st_cur,  = ax_st.plot([], [], "go", ms=5)

    plt.tight_layout(rect=[0, 0, 1, 0.94])

    # ---- Update ----
    def update(frame):
        fi = frame

        dyn_ref.set_data(ref_x_frames[fi], ref_y_frames[fi])
        dyn_bl.set_data(blx_frames[fi], bly_frames[fi])
        dyn_br.set_data(brx_frames[fi], bry_frames[fi])
        dyn_trail.set_data(trail_x_frames[fi], trail_y_frames[fi])

        parts = car_frames[fi]
        for j, (verts, fc, ec, lw) in enumerate(parts):
            if j < len(car_patches):
                car_patches[j].set_xy(verts)
                car_patches[j].set_facecolor(fc)
                car_patches[j].set_edgecolor(ec)
                car_patches[j].set_linewidth(lw)
        for j in range(len(parts), len(car_patches)):
            car_patches[j].set_xy([[0,0],[0,0],[0,0],[0,0]])

        cx, cy = car_x[indices[fi]], car_y[indices[fi]]
        if not args.full:
            ax_map.set_xlim(cx - ZOOM, cx + ZOOM)
            ax_map.set_ylim(cy - ZOOM, cy + ZOOM)

        info_txt.set_text(
            f"t={times[fi]:.1f}s  s={steps[fi]:.0f}/{traj.total_len:.0f}m\n"
            f"e_y={eys[fi]:+.3f}m  e_psi={eps[fi]:+.1f}deg  steer={strs[fi]:+.1f}deg")

        # Overview
        ov_trail.set_data(car_x[:indices[fi]+1], car_y[:indices[fi]+1])
        ov_dot.set_data([cx], [cy])

        # Error plots
        n_data = indices[fi] + 1
        ey_line.set_data(t_arr[:n_data], e_y[:n_data])
        ey_cur.set_data([times[fi]], [eys[fi]])
        ep_line.set_data(t_arr[:n_data], ep_deg[:n_data])
        ep_cur.set_data([times[fi]], [eps[fi]])
        st_line.set_data(t_arr[:n_data], st_deg[:n_data])
        st_cur.set_data([times[fi]], [strs[fi]])

        return [dyn_ref, dyn_bl, dyn_br, dyn_trail, info_txt,
                ey_line, ey_cur, ep_line, ep_cur, st_line, st_cur,
                ov_trail, ov_dot] + car_patches

    fps = max(1, int(round(1000 / interval))) if interval > 0 else 5
    ani = FuncAnimation(fig, update, frames=nf, interval=interval, blit=False)
    plt.tight_layout(rect=[0, 0, 1, 0.94])

    if args.save:
        print(f"Saving {args.save} ({nf} frames, fps={fps}) ...")
        ani.save(args.save, writer="pillow", fps=fps, dpi=100)
        print("Done!")
    else:
        plt.show()


if __name__ == "__main__":
    run()
