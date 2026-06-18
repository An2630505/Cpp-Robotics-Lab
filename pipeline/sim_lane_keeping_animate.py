"""
sim_lane_keeping_animate.py — 车道保持仿真动画可视化

读取 output/sim_lane_keeping.txt，生成带自行车模型的道路动画。

用法:
  python pipeline/sim_lane_keeping_animate.py                    # 交互播放
  python pipeline/sim_lane_keeping_animate.py --save output.gif  # 保存为 GIF
"""

import sys, os, argparse, math
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ===== 车辆参数 =====
LF, LR = 1.1, 1.58
L_WB = LF + LR
LANE_WIDTH = 3.5


def load_data(filepath):
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"找不到 {filepath}")
    ref_params = {}
    with open(filepath) as f:
        first = f.readline().strip()
    if first.startswith("# REF:"):
        for kv in first.replace("# REF:", "").split():
            if "=" not in kv:
                ref_params["segments"] = kv.replace("segments", "", 1).lstrip("|")
                continue
            k, v = kv.split("=", 1)
            try:
                ref_params[k] = float(v)
            except ValueError:
                ref_params[k] = v
    data = np.loadtxt(filepath, delimiter="\t", skiprows=2)
    return {"step": data[:, 0].astype(int), "time": data[:, 1], "e_y": data[:, 2],
            "de_y": data[:, 3], "e_psi": data[:, 4], "de_psi": data[:, 5],
            "steer": data[:, 6]}, ref_params


def rebuild_reference_path(n_steps, ref_params):
    """从 # REF 头重建参考路径."""
    dt = ref_params.get("dt", 0.1)
    Vx = ref_params.get("Vx", 10.0)
    seg_str = ref_params.get("segments", "")
    segs = []
    x, y, psi = 0.0, 0.0, 0.0
    cum_s = 0.0
    for sseg in seg_str.split("|"):
        if not sseg: continue
        fields = sseg.split(":")
        t = fields[0]
        seg = {"type": t, "start_s": cum_s, "sx": x, "sy": y, "sp": psi}
        if t == "S":
            L = float(fields[1]); seg["L"] = L
            x += L * math.cos(psi); y += L * math.sin(psi); cum_s += L
        elif t == "A":
            L, R = float(fields[1]), float(fields[2]); seg["L"], seg["R"] = L, R
            da = L / R
            x += R * (math.sin(psi + da) - math.sin(psi)); y -= R * (math.cos(psi + da) - math.cos(psi))
            psi += da; cum_s += L
        elif t == "Z":
            L, A, omega = float(fields[1]), float(fields[2]), float(fields[3])
            seg["L"], seg["A"], seg["omega"] = L, A, omega
            yl = A * math.sin(omega * L); dy = A * omega * math.cos(omega * L)
            x += L * math.cos(psi) - yl * math.sin(psi); y += L * math.sin(psi) + yl * math.cos(psi)
            psi += math.atan2(dy, 1.0); cum_s += L
        segs.append(seg)
    total_len = cum_s
    x_ref, y_ref, psi_ref = np.zeros(n_steps), np.zeros(n_steps), np.zeros(n_steps)
    for i in range(n_steps):
        s = i * Vx * dt
        if s > total_len: s = total_len
        idx = 0
        while idx + 1 < len(segs) and segs[idx + 1]["start_s"] <= s: idx += 1
        seg = segs[idx]; sl = s - seg["start_s"]
        sx, sy, sp = seg["sx"], seg["sy"], seg["sp"]
        if seg["type"] == "S":
            x_ref[i] = sx + sl * math.cos(sp); y_ref[i] = sy + sl * math.sin(sp); psi_ref[i] = sp
        elif seg["type"] == "A":
            R = seg["R"]; da = sl / R; ps = sp + da
            x_ref[i] = sx + R * (math.sin(ps) - math.sin(sp)); y_ref[i] = sy - R * (math.cos(ps) - math.cos(sp))
            psi_ref[i] = ps
        elif seg["type"] == "Z":
            A, omega = seg["A"], seg["omega"]
            yl = A * math.sin(omega * sl); dy = A * omega * math.cos(omega * sl)
            ps = sp + math.atan2(dy, 1.0)
            x_ref[i] = sx + sl * math.cos(sp) - yl * math.sin(sp)
            y_ref[i] = sy + sl * math.sin(sp) + yl * math.cos(sp); psi_ref[i] = ps
    return x_ref, y_ref, psi_ref, total_len


def wheel_poly(axle_x, axle_y, angle, width=0.5, length=1.5):
    c, s = math.cos(angle), math.sin(angle)
    hl, hw = length / 2, width / 2
    corners = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
    R = np.array([[c, -s], [s, c]])
    verts = corners @ R.T + np.array([axle_x, axle_y])
    return verts[:, 0], verts[:, 1]


def main():
    ap = argparse.ArgumentParser(description="车道保持动画")
    ap.add_argument("--file", default="output/sim_lane_keeping.txt")
    ap.add_argument("--save", default=None, help="保存为 GIF (需 pillow)")
    ap.add_argument("--interval", type=int, default=40, help="帧间隔 (ms)")
    ap.add_argument("--skip", type=int, default=3, help="每隔 N 步取一帧")
    args = ap.parse_args()

    d, ref = load_data(args.file)
    N = len(d["time"])
    t = d["time"]
    skip = max(1, args.skip)

    x_ref, y_ref, psi_ref, total_len = rebuild_reference_path(N, ref)
    print(f"  路径总长: {total_len:.1f}m, 步数: {N}, 帧间隔: {skip}")

    # 车辆绝对位姿
    car_x = x_ref - d["e_y"] * np.sin(psi_ref)
    car_y = y_ref + d["e_y"] * np.cos(psi_ref)
    psi_car = psi_ref + d["e_psi"]
    rear_x = car_x - LR * np.cos(psi_car)
    rear_y = car_y - LR * np.sin(psi_car)
    front_x = car_x + LF * np.cos(psi_car)
    front_y = car_y + LF * np.sin(psi_car)

    lane_lx = x_ref - LANE_WIDTH / 2 * np.sin(psi_ref)
    lane_ly = y_ref + LANE_WIDTH / 2 * np.cos(psi_ref)
    lane_rx = x_ref + LANE_WIDTH / 2 * np.sin(psi_ref)
    lane_ry = y_ref - LANE_WIDTH / 2 * np.cos(psi_ref)

    half_win = 15.0

    # ===== 图 =====
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle("Bicycle Model — Lane Keeping Animation", fontsize=16)

    ax_main = plt.subplot(2, 2, (1, 3))
    ax_main.set_aspect("equal")
    ax_main.set_xlabel("X (m)"); ax_main.set_ylabel("Y (m)")
    ax_main.grid(True, alpha=0.3)

    ref_line, = ax_main.plot([], [], "b--", lw=1.5, alpha=0.6, label="Reference")
    left_lane, = ax_main.plot([], [], "gray", lw=1.0, ls=":", alpha=0.5)
    right_lane, = ax_main.plot([], [], "gray", lw=1.0, ls=":", alpha=0.5, label="Lane")
    car_body, = ax_main.plot([], [], "r-", lw=3.0, label="Vehicle")
    rear_w = ax_main.fill([], [], "k", alpha=0.8, lw=1, ec="white")[0]
    front_w = ax_main.fill([], [], "g", alpha=0.8, lw=1, ec="white")[0]
    trail, = ax_main.plot([], [], "r-", lw=1.0, alpha=0.4)
    time_txt = ax_main.text(0.02, 0.95, "", transform=ax_main.transAxes, fontsize=10,
                            va="top", bbox=dict(boxstyle="round", fc="wheat", alpha=0.8))
    info_txt = ax_main.text(0.02, 0.75, "", transform=ax_main.transAxes, fontsize=9,
                            va="top", bbox=dict(boxstyle="round", fc="lightcyan", alpha=0.8))
    ax_main.legend(loc="lower right", fontsize=8)

    ax_ey = plt.subplot(2, 2, 2)
    ax_ey.set_ylabel("e_y (m)"); ax_ey.set_title("Lateral Error"); ax_ey.grid(True, alpha=0.3)
    ey_line, = ax_ey.plot([], [], "b-", lw=1.5)
    ax_ey.axhline(0, color="k", ls="--", lw=0.5)
    ax_ey.set_xlim(t[0], t[-1]); ax_ey.set_ylim(-1.5, 1.5)

    ax_ep = plt.subplot(2, 2, 4)
    ax_ep.set_xlabel("Time (s)"); ax_ep.set_ylabel("e_psi (deg)"); ax_ep.set_title("Heading Error")
    ax_ep.grid(True, alpha=0.3)
    ep_line, = ax_ep.plot([], [], "r-", lw=1.5)
    ax_ep.axhline(0, color="k", ls="--", lw=0.5)
    ax_ep.set_xlim(t[0], t[-1]); ax_ep.set_ylim(-30, 30)

    def update(frame):
        i = min(frame * skip, N - 1); n = i + 1
        ref_line.set_data(x_ref[:n], y_ref[:n])
        left_lane.set_data(lane_lx[:n], lane_ly[:n])
        right_lane.set_data(lane_rx[:n], lane_ry[:n])
        trail.set_data(car_x[:n], car_y[:n])
        car_body.set_data([rear_x[i], front_x[i]], [rear_y[i], front_y[i]])
        rwx, rwy = wheel_poly(rear_x[i], rear_y[i], psi_car[i])
        rear_w.set_xy(np.column_stack([rwx, rwy]))
        fwx, fwy = wheel_poly(front_x[i], front_y[i], psi_car[i] + d["steer"][i])
        front_w.set_xy(np.column_stack([fwx, fwy]))
        cx, cy = car_x[i], car_y[i]
        ax_main.set_xlim(cx - half_win, cx + half_win); ax_main.set_ylim(cy - half_win, cy + half_win)
        time_txt.set_text(f"Time: {t[i]:.1f}s")
        info_txt.set_text(f"e_y = {d['e_y'][i]:.3f}m\ne_psi = {np.degrees(d['e_psi'][i]):.1f}°"
                          f"\nsteer = {np.degrees(d['steer'][i]):.1f}°")
        ey_line.set_data(t[:n], d["e_y"][:n])
        ep_line.set_data(t[:n], np.degrees(d["e_psi"][:n]))
        return (ref_line, left_lane, right_lane, car_body, rear_w, front_w, trail,
                time_txt, info_txt, ey_line, ep_line)

    nf = (N + skip - 1) // skip
    ani = FuncAnimation(fig, update, frames=nf, interval=args.interval, blit=False)

    if args.save:
        print(f"  保存动画至 {args.save} ...")
        ani.save(args.save, writer="pillow", fps=1000 // args.interval)
        print("  完成!")
    else:
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
