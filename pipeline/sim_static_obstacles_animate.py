"""
sim_static_obstacles_animate.py — 静态障碍物场景仿真动画

读取仿真日志 + 轨迹点 + 赛道边界 + 障碍物 + 安全走廊, 还原车辆运动过程。

用法:
  python pipeline/sim_static_obstacles_animate.py
  python pipeline/sim_static_obstacles_animate.py --save output/animation.gif
"""

from __future__ import annotations

import os, sys, math, json, argparse
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

from pipeline.sim_static_obstacles import Trajectory

# 车辆几何
LF, LR = 1.1, 1.58
L_WB = LF + LR
CAR_W = 1.8
WHL_W = 0.42
WHL_L = 0.85
LANE_W = 3.5
ZOOM = 20.0

# ============================================================
#  Data loading
# ============================================================

def load_sim_data(filepath):
    with open(filepath) as f:
        line = f.readline()
    return np.loadtxt(filepath, delimiter="\t", skiprows=2)


def load_boundaries():
    outer = np.load("output/sim_static_obstacles_outer.npy")
    holes = []
    i = 0
    while os.path.exists(f"output/sim_static_obstacles_hole_{i}.npy"):
        holes.append(np.load(f"output/sim_static_obstacles_hole_{i}.npy"))
        i += 1
    return outer, holes


def load_trajectory():
    pts = np.load("output/sim_static_obstacles_traj.npy")
    return Trajectory(pts), pts


def load_obstacles():
    json_path = os.path.join(_root, "config", "obstacles.json")
    if os.path.exists(json_path):
        with open(json_path) as f:
            cfg = json.load(f)
        from pipeline.static_obstacles import ObstacleLayer
        layer = ObstacleLayer()
        layer.add_from_json(json_path)
        return layer, layer.to_polygons()
    return None, []


def load_corridors():
    path = "output/sim_static_obstacles_corridors.npy"
    if os.path.exists(path):
        return np.load(path)  # shape (N, 3, 2): [left, center, right]
    return None


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
        rx, ry = cx - LR*cos_p, cy - LR*sin_p
        fx, fy = cx + LF*cos_p, cy + LF*sin_p

        parts = []
        # 车身
        body = np.array([[LF+.2, CAR_W/2], [LF+.2, -CAR_W/2],
                         [-LR-.2, -CAR_W/2], [-LR-.2, CAR_W/2]])
        parts.append((body @ R.T + [cx, cy], "#e63946", "#c1121f", 1.5))
        # 挡风玻璃
        ws = np.array([[LF+.2, CAR_W*.38], [LF-.3, CAR_W*.38],
                       [LF-.3, -CAR_W*.38], [LF+.2, -CAR_W*.38]])
        parts.append((ws @ R.T + [cx, cy], "#a8dadc", "#457b9d", 0.6))
        # 后窗
        rw = np.array([[-LR+.3, CAR_W*.38], [-LR-.1, CAR_W*.38],
                       [-LR-.1, -CAR_W*.38], [-LR+.3, -CAR_W*.38]])
        parts.append((rw @ R.T + [cx, cy], "#a8dadc", "#457b9d", 0.6))
        # 车轮
        for ax, ay, angle in [(rx, ry, ps), (fx, fy, ps + st)]:
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
    ap = argparse.ArgumentParser(description="静态障碍物场景动画")
    ap.add_argument("--file", default="output/sim_static_obstacles.txt")
    ap.add_argument("--save", default=None, help="保存为 GIF")
    ap.add_argument("--interval", type=int, default=25, help="帧间隔 (ms)")
    ap.add_argument("--skip", type=int, default=2, help="每隔 N 步取一帧")
    ap.add_argument("--speed", type=float, default=1.0, help="播放速度倍率")
    ap.add_argument("--full", action="store_true", help="固定全局视角")
    args = ap.parse_args()

    print("Loading simulation data ...")
    data = load_sim_data(args.file)
    N = len(data)
    dt = 0.1
    Vx = 10.0
    t_arr = data[:, 1]
    e_y = data[:, 2]
    e_psi = data[:, 4]
    steer = data[:, 6]

    print("Loading trajectory & boundaries & obstacles & corridors ...")
    traj, waypoints = load_trajectory()
    outer, holes = load_boundaries()
    obs_layer, obs_polys = load_obstacles()
    corridors = load_corridors()

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

    lane_lx = x_ref - LANE_W/2 * np.sin(psi_ref)
    lane_ly = y_ref + LANE_W/2 * np.cos(psi_ref)
    lane_rx = x_ref + LANE_W/2 * np.sin(psi_ref)
    lane_ry = y_ref - LANE_W/2 * np.cos(psi_ref)

    ep_deg = np.degrees(e_psi)
    st_deg = np.degrees(steer)

    car_frames = precompute_car_polys(nf, skip, car_x, car_y, psi_car, steer)

    trail_x_frames = [car_x[:i+1] for i in indices]
    trail_y_frames = [car_y[:i+1] for i in indices]
    ref_x_frames  = [x_ref[:i+1]  for i in indices]
    ref_y_frames  = [y_ref[:i+1]  for i in indices]
    llx_frames = [lane_lx[:i+1] for i in indices]
    lly_frames = [lane_ly[:i+1] for i in indices]
    rlx_frames = [lane_rx[:i+1] for i in indices]
    rly_frames = [lane_ry[:i+1] for i in indices]

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
    fig.suptitle("Static Obstacles — HA* + B-Spline + MPC", fontsize=14)

    # ---- Main map ----
    ax_map = fig.add_subplot(1, 2, 1)
    ax_map.set_aspect("equal")
    ax_map.grid(True, alpha=0.2)
    ax_map.set_xlabel("X (m)"); ax_map.set_ylabel("Y (m)")

    # Static backdrop: 赛道边界
    ax_map.plot(outer[:,0], outer[:,1], "#444", lw=1.8, alpha=0.6)

    # 安全走廊 (clip 到 outer 内)
    clip_patch = PathPatch(MplPath(outer), transform=ax_map.transData)
    patch_kw = {"clip_path": clip_patch, "clip_on": True}
    if corridors is not None:
        lx = corridors[:, 0, 0]; ly = corridors[:, 0, 1]
        rx = corridors[:, 2, 0]; ry = corridors[:, 2, 1]
        ax_map.fill(np.concatenate([lx, rx[::-1]]),
                     np.concatenate([ly, ry[::-1]]),
                     fc="cyan", ec="none", alpha=0.12,
                     label="Safe Corridor", **patch_kw)

    # 孔洞
    for h in holes:
        ax_map.fill(h[:,0], h[:,1], fc="white", ec="#444", lw=0.5, alpha=0.95)

    # 静态障碍物
    if obs_layer is not None and len(obs_layer) > 0:
        for i, poly in enumerate(obs_polys):
            ax_map.fill(poly[:,0], poly[:,1], fc="red", ec="darkred",
                         lw=1.5, alpha=0.45)

    # 优化轨迹 (灰虚线背景)
    ax_map.plot(waypoints[:,0], waypoints[:,1], "b--", lw=0.6, alpha=0.35)

    # Dynamic lines
    dyn_ref,   = ax_map.plot([], [], "b-", lw=1.8, alpha=0.55)
    dyn_ll,    = ax_map.plot([], [], "gray", lw=0.5, ls=":", alpha=0.45)
    dyn_rl,    = ax_map.plot([], [], "gray", lw=0.5, ls=":", alpha=0.45)
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

    # ---- Overview inset ----
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
    for poly in obs_polys:
        ax_ov.fill(poly[:,0], poly[:,1], fc="red", ec="darkred", lw=0.3, alpha=0.4)
    ax_ov.plot(waypoints[:,0], waypoints[:,1], "b--", lw=0.4, alpha=0.4)
    ax_ov.set_xlim(ov_x_min - ov_pad, ov_x_max + ov_pad)
    ax_ov.set_ylim(ov_y_min - ov_pad, ov_y_max + ov_pad)

    ov_trail, = ax_ov.plot([], [], "r-", lw=0.8, alpha=0.6)
    ov_dot,   = ax_ov.plot([], [], "ro", ms=6, mec="#800", mew=0.8)

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
        dyn_ll.set_data(llx_frames[fi], lly_frames[fi])
        dyn_rl.set_data(rlx_frames[fi], rly_frames[fi])
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

        return [dyn_ref, dyn_ll, dyn_rl, dyn_trail, info_txt,
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
