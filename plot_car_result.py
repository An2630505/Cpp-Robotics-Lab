#!/usr/bin/env python3
"""
plot_car_result.py - 自行车模型 LDW 可视化

读取 7 列输出，内部重建 Object 参考轨迹，显示全局俯视图:
  参考路径 + 自行车模型(车身/前后轮) + 车道边界 + 误差/转角曲线。
"""

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import argparse, os, math

# ===== 车辆参数 =====
lf = 1.1
lr = 1.58
L = lf + lr
LANE_WIDTH = 3.5

# ===== Object 参数 (从文件首行读取) =====
REF_PARAMS = None


def load_data(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"找不到 {path}")

    global REF_PARAMS
    with open(path) as f:
        first = f.readline().strip()
    if first.startswith("# REF:"):
        parts = first.replace("# REF:", "").split()
        p = {}
        for kv in parts:
            if "=" not in kv:
                p["segments"] = kv.replace("segments", "", 1).lstrip("|")
                continue
            k, v = kv.split("=", 1)
            try:
                p[k] = float(v)
            except ValueError:
                p[k] = v  # 保留字符串（如 type=slalom）
        REF_PARAMS = p
    else:
        REF_PARAMS = {"type": "circle", "w": 0.5, "dt": 0.1, "Vx": 15,
                      "x0": 0, "y0": 1, "vx0": 15, "vy0": 0}

    data = np.loadtxt(path, delimiter="\t", skiprows=2)
    return {
        "step": data[:, 0].astype(int),
        "time": data[:, 1],
        "e_y": data[:, 2],
        "de_y": data[:, 3],
        "e_psi": data[:, 4],
        "de_psi": data[:, 5],
        "steer": data[:, 6],
    }


def generate_reference(N, p):
    """生成参考轨迹，仅 complex 多段组合路径，circle 为旧格式回退"""
    ref_type = p.get("type", "circle")
    dt = p["dt"]
    Vx = p["Vx"]

    if ref_type == "complex":
        seg_str = p["segments"]
        seg_parts = seg_str.split("|")
        segs = []
        x, y, psi = 0.0, 0.0, 0.0
        cum_s = 0.0

        for sseg in seg_parts:
            if not sseg:
                continue
            fields = sseg.split(":")
            t = fields[0]
            seg = {"type": t, "start_s": cum_s, "sx": x, "sy": y, "sp": psi}

            if t == "S":  # straight
                L = float(fields[1])
                seg["L"] = L
                x += L * math.cos(psi)
                y += L * math.sin(psi)
                cum_s += L
            elif t == "A":  # arc (circle or curve)
                L, R = float(fields[1]), float(fields[2])
                seg["L"], seg["R"] = L, R
                da = L / R
                x += R * (math.sin(psi + da) - math.sin(psi))
                y -= R * (math.cos(psi + da) - math.cos(psi))
                psi += da
                cum_s += L
            elif t == "Z":  # slalom
                L, A, omega = float(fields[1]), float(fields[2]), float(fields[3])
                seg["L"], seg["A"], seg["omega"] = L, A, omega
                yle = A * math.sin(omega * L)
                dye = A * omega * math.cos(omega * L)
                x += L * math.cos(psi) - yle * math.sin(psi)
                y += L * math.sin(psi) + yle * math.cos(psi)
                psi += math.atan2(dye, 1.0)
                cum_s += L

            segs.append(seg)

        total_len = cum_s
        xa = np.zeros(N)
        ya = np.zeros(N)
        vxa = np.zeros(N)
        vya = np.zeros(N)

        for i in range(N):
            s = i * Vx * dt
            if s > total_len:
                s = total_len

            idx = 0
            while idx + 1 < len(segs) and segs[idx + 1]["start_s"] <= s:
                idx += 1

            seg = segs[idx]
            sl = s - seg["start_s"]
            sx, sy, sp = seg["sx"], seg["sy"], seg["sp"]

            if seg["type"] == "S":
                xa[i] = sx + sl * math.cos(sp)
                ya[i] = sy + sl * math.sin(sp)
                vxa[i] = Vx * math.cos(sp)
                vya[i] = Vx * math.sin(sp)
            elif seg["type"] == "A":
                R = seg["R"]
                da = sl / R
                ps = sp + da
                xa[i] = sx + R * (math.sin(ps) - math.sin(sp))
                ya[i] = sy - R * (math.cos(ps) - math.cos(sp))
                vxa[i] = Vx * math.cos(ps)
                vya[i] = Vx * math.sin(ps)
            elif seg["type"] == "Z":
                A, omega = seg["A"], seg["omega"]
                yl = A * math.sin(omega * sl)
                dy = A * omega * math.cos(omega * sl)
                ps = sp + math.atan2(dy, 1.0)
                xa[i] = sx + sl * math.cos(sp) - yl * math.sin(sp)
                ya[i] = sy + sl * math.sin(sp) + yl * math.cos(sp)
                vxa[i] = Vx * math.cos(ps)
                vya[i] = Vx * math.sin(ps)

        return xa, ya, vxa, vya
    else:
        # 回退：圆周运动
        x, y, vx, vy = p["x0"], p["y0"], p["vx0"], p["vy0"]
        xa, ya, vxa, vya = np.zeros(N), np.zeros(N), np.zeros(N), np.zeros(N)
        c, s = math.cos(p["w"] * dt), math.sin(p["w"] * dt)
        for i in range(N):
            xa[i], ya[i], vxa[i], vya[i] = x, y, vx, vy
            x += dt * vx
            y += dt * vy
            vx, vy = vx * c - vy * s, vx * s + vy * c
        return xa, ya, vxa, vya


def wheel_poly(axle_x, axle_y, angle, width=0.5, length=1.5):
    """返回矩形车轮的四个顶点（沿 rolling 方向）"""
    c, s = math.cos(angle), math.sin(angle)
    # 半长沿 rolling 方向，半宽垂直 rolling 方向
    hl, hw = length / 2, width / 2
    corners = np.array([
        [ hl,  hw],
        [ hl, -hw],
        [-hl, -hw],
        [-hl,  hw],
    ])
    R = np.array([[c, -s], [s,  c]])
    verts = corners @ R.T + np.array([axle_x, axle_y])
    return verts[:, 0], verts[:, 1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", default="output/output.txt")
    ap.add_argument("--save", default=None)
    ap.add_argument("--interval", type=int, default=50)
    ap.add_argument("--skip", type=int, default=2)
    args = ap.parse_args()

    d = load_data(args.file)
    N = len(d["time"])
    skip = max(1, args.skip)

    # 生成参考轨迹 (从文件首行读取的参数)
    x_ref, y_ref, dx_ref, dy_ref = generate_reference(N, REF_PARAMS)

    # 绝对位姿
    spd = np.hypot(dx_ref, dy_ref)
    spd = np.where(spd < 1e-12, 1e-12, spd)
    psi_ref = np.arctan2(dy_ref, dx_ref)

    car_x = x_ref - d["e_y"] * np.sin(psi_ref)
    car_y = y_ref + d["e_y"] * np.cos(psi_ref)
    psi_car = psi_ref + d["e_psi"]

    rear_x = car_x - lr * np.cos(psi_car)
    rear_y = car_y - lr * np.sin(psi_car)
    front_x = car_x + lf * np.cos(psi_car)
    front_y = car_y + lf * np.sin(psi_car)

    lane_lx = x_ref - LANE_WIDTH / 2 * np.sin(psi_ref)
    lane_ly = y_ref + LANE_WIDTH / 2 * np.cos(psi_ref)
    lane_rx = x_ref + LANE_WIDTH / 2 * np.sin(psi_ref)
    lane_ry = y_ref - LANE_WIDTH / 2 * np.cos(psi_ref)

    t = d["time"]

    # 固定子图范围
    ey_lim = max(np.abs(d["e_y"]).max() * 1.4, 0.5)
    ep_lim = max(np.abs(d["e_psi"]).max() * 1.4, 0.5)

    # ===== 建图 =====
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle("Bicycle Model — Lane Keeping Visualization", fontsize=16)

    # --- 主图: 俯视 ---
    ax_main = plt.subplot(2, 2, (1, 3))
    ax_main.set_aspect("equal")
    ax_main.set_xlabel("X (m)")
    ax_main.set_ylabel("Y (m)")
    ax_main.grid(True, alpha=0.3)

    ref_line, = ax_main.plot([], [], "b--", lw=1.5, alpha=0.6, label="Reference Path")
    left_lane, = ax_main.plot([], [], "gray", lw=1.0, ls=":", alpha=0.5, label="Lane Boundary")
    right_lane, = ax_main.plot([], [], "gray", lw=1.0, ls=":", alpha=0.5)

    car_body, = ax_main.plot([], [], "r-", lw=3.0, label="Car Body")
    rear_wheel = ax_main.fill([], [], "k", alpha=0.8, lw=1, edgecolor="white")[0]
    front_wheel = ax_main.fill([], [], "g", alpha=0.8, lw=1, edgecolor="white")[0]
    front_center, = ax_main.plot([], [], "go", ms=4)
    rear_center, = ax_main.plot([], [], "ko", ms=4)
    car_trail, = ax_main.plot([], [], "r-", lw=1.0, alpha=0.4)

    time_text = ax_main.text(0.02, 0.95, "", transform=ax_main.transAxes, fontsize=10,
                             va="top", bbox=dict(boxstyle="round", fc="wheat", alpha=0.8))
    info_text = ax_main.text(0.02, 0.75, "", transform=ax_main.transAxes, fontsize=9,
                             va="top", bbox=dict(boxstyle="round", fc="lightcyan", alpha=0.8))

    ax_main.legend(loc="lower right", fontsize=8)

    # 动态视野（动画中跟随车辆更新）
    half_win = 15.0  # 半窗口大小

    # --- 子图: e_y ---
    ax_ey = plt.subplot(2, 2, 2)
    ax_ey.set_xlabel("Time (s)"); ax_ey.set_ylabel("e_y (m)")
    ax_ey.set_title("Lateral Error"); ax_ey.grid(True, alpha=0.3)
    ey_line, = ax_ey.plot([], [], "b-", lw=1.5)
    ax_ey.axhline(y=0, color="k", ls="--", lw=0.5)
    ax_ey.set_xlim(t[0], t[-1]); ax_ey.set_ylim(-ey_lim, ey_lim)

    # --- 子图: e_psi ---
    ax_ep = plt.subplot(2, 2, 4)
    ax_ep.set_xlabel("Time (s)"); ax_ep.set_ylabel("e_psi (rad)")
    ax_ep.set_title("Heading Error"); ax_ep.grid(True, alpha=0.3)
    ep_line, = ax_ep.plot([], [], "r-", lw=1.5)
    ax_ep.axhline(y=0, color="k", ls="--", lw=0.5)
    ax_ep.set_xlim(t[0], t[-1]); ax_ep.set_ylim(-ep_lim, ep_lim)

    # ===== 小地图 (inset in top-right corner of main view) =====
    ax_minimap = ax_main.inset_axes([0.55, 0.55, 0.45, 0.45])
    ax_minimap.set_aspect("equal")

    # 完整参考轨迹（静态）
    ax_minimap.plot(x_ref, y_ref, "b--", lw=1.0, alpha=0.6)

    # 车辆行驶轨迹（静态）
    ax_minimap.plot(car_x, car_y, "r-", lw=0.8, alpha=0.4)

    # 起点
    ax_minimap.plot(x_ref[0], y_ref[0], "go", ms=4, alpha=0.8)

    # 当前位置（动态更新）
    minimap_dot, = ax_minimap.plot([], [], "ro", ms=5, zorder=5)

    # 固定显示完整路径范围
    all_x = np.concatenate([x_ref, car_x])
    all_y = np.concatenate([y_ref, car_y])
    x_margin = max(np.ptp(all_x) * 0.05, 5.0)
    y_margin = max(np.ptp(all_y) * 0.05, 5.0)
    ax_minimap.set_xlim(all_x.min() - x_margin, all_x.max() + x_margin)
    ax_minimap.set_ylim(all_y.min() - y_margin, all_y.max() + y_margin)
    ax_minimap.set_xticks([])
    ax_minimap.set_yticks([])

    # ===== 动画 =====
    def update(frame):
        i = min(frame * skip, N - 1)
        n = i + 1

        ref_line.set_data(x_ref[:n], y_ref[:n])
        left_lane.set_data(lane_lx[:n], lane_ly[:n])
        right_lane.set_data(lane_rx[:n], lane_ry[:n])
        car_trail.set_data(car_x[:n], car_y[:n])
        minimap_dot.set_data([car_x[i]], [car_y[i]])

        car_body.set_data([rear_x[i], front_x[i]], [rear_y[i], front_y[i]])

        # 后轮：沿车身方向 (psi_car) 滚动
        r_angle = psi_car[i]
        rwx, rwy = wheel_poly(rear_x[i], rear_y[i], r_angle)
        rear_wheel.set_xy(np.column_stack([rwx, rwy]))
        rear_center.set_data([rear_x[i]], [rear_y[i]])

        # 前轮：沿 psi_car + steer 方向滚动
        f_angle = psi_car[i] + d["steer"][i]
        fwx, fwy = wheel_poly(front_x[i], front_y[i], f_angle)
        front_wheel.set_xy(np.column_stack([fwx, fwy]))
        front_center.set_data([front_x[i]], [front_y[i]])

        # 动态视角：跟随车辆
        cx, cy = car_x[i], car_y[i]
        ax_main.set_xlim(cx - half_win, cx + half_win)
        ax_main.set_ylim(cy - half_win, cy + half_win)

        time_text.set_text(f"Time: {t[i]:.1f} s")
        info_text.set_text(
            f"e_y = {d['e_y'][i]:.3f} m\n"
            f"e_psi = {d['e_psi'][i]:.3f} rad\n"
            f"steer = {np.degrees(d['steer'][i]):.1f}°"
        )

        ey_line.set_data(t[:n], d["e_y"][:n])
        ep_line.set_data(t[:n], d["e_psi"][:n])

        return (ref_line, left_lane, right_lane, car_body,
                rear_wheel, front_wheel, front_center, rear_center,
                car_trail, time_text, info_text, ey_line, ep_line,
                minimap_dot)

    nf = (N + skip - 1) // skip
    ani = FuncAnimation(fig, update, frames=nf, interval=args.interval, blit=False)

    if args.save:
        print(f"保存至 {args.save} ...")
        ani.save(args.save, writer="pillow", fps=1000 // args.interval)
        print("完成!")
    else:
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
