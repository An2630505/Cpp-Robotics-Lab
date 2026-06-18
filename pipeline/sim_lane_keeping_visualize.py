"""
sim_lane_keeping_visualize.py — 车道保持仿真静态可视化

读取 output/sim_lane_keeping.txt，生成四幅子图：
  1. 全局俯视 — 参考路径 + 车辆轨迹 + 车道边界
  2. 横向误差 e_y 随时间
  3. 航向误差 e_psi 随时间
  4. 方向盘转角 steer 随时间

用法: python pipeline/sim_lane_keeping_visualize.py [--file output/sim_lane_keeping.txt]
"""

import sys, os, argparse, math, re
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

# ===== 车辆参数 =====
LF, LR = 1.1, 1.58
L_WB = LF + LR
LANE_WIDTH = 3.5


def load_data(filepath):
    """读取输出文件，返回 data dict 和参考参数."""
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"找不到 {filepath}")

    # 解析 # REF 头
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

    # 加载数据
    data = np.loadtxt(filepath, delimiter="\t", skiprows=2)
    return {
        "step": data[:, 0].astype(int),
        "time": data[:, 1],
        "e_y": data[:, 2],
        "de_y": data[:, 3],
        "e_psi": data[:, 4],
        "de_psi": data[:, 5],
        "steer": data[:, 6],
    }, ref_params


def rebuild_reference_path(n_steps, ref_params):
    """从 # REF 头重建参考路径 (与 sim_lane_keeping.py 中 build_path 对应)."""
    dt = ref_params.get("dt", 0.1)
    Vx = ref_params.get("Vx", 10.0)

    seg_str = ref_params.get("segments", "")
    segs = []
    x, y, psi = 0.0, 0.0, 0.0
    cum_s = 0.0

    for sseg in seg_str.split("|"):
        if not sseg:
            continue
        fields = sseg.split(":")
        t = fields[0]
        seg = {"type": t, "start_s": cum_s, "sx": x, "sy": y, "sp": psi}

        if t == "S":
            L_len = float(fields[1])
            seg["L"] = L_len
            x += L_len * math.cos(psi)
            y += L_len * math.sin(psi)
            cum_s += L_len
        elif t == "A":
            L_len, R = float(fields[1]), float(fields[2])
            seg["L"], seg["R"] = L_len, R
            da = L_len / R
            x += R * (math.sin(psi + da) - math.sin(psi))
            y -= R * (math.cos(psi + da) - math.cos(psi))
            psi += da
            cum_s += L_len
        elif t == "Z":
            L_len, A, omega = float(fields[1]), float(fields[2]), float(fields[3])
            seg["L"], seg["A"], seg["omega"] = L_len, A, omega
            yl = A * math.sin(omega * L_len)
            dy = A * omega * math.cos(omega * L_len)
            x += L_len * math.cos(psi) - yl * math.sin(psi)
            y += L_len * math.sin(psi) + yl * math.cos(psi)
            psi += math.atan2(dy, 1.0)
            cum_s += L_len

        segs.append(seg)

    total_len = cum_s
    x_ref = np.zeros(n_steps)
    y_ref = np.zeros(n_steps)
    psi_ref = np.zeros(n_steps)
    kappa_ref = np.zeros(n_steps)

    for i in range(n_steps):
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
            x_ref[i] = sx + sl * math.cos(sp)
            y_ref[i] = sy + sl * math.sin(sp)
            psi_ref[i] = sp
            kappa_ref[i] = 0.0
        elif seg["type"] == "A":
            R = seg["R"]
            da = sl / R
            ps = sp + da
            x_ref[i] = sx + R * (math.sin(ps) - math.sin(sp))
            y_ref[i] = sy - R * (math.cos(ps) - math.cos(sp))
            psi_ref[i] = ps
            kappa_ref[i] = 1.0 / R
        elif seg["type"] == "Z":
            A, omega = seg["A"], seg["omega"]
            yl = A * math.sin(omega * sl)
            dy = A * omega * math.cos(omega * sl)
            ps = sp + math.atan2(dy, 1.0)
            x_ref[i] = sx + sl * math.cos(sp) - yl * math.sin(sp)
            y_ref[i] = sy + sl * math.sin(sp) + yl * math.cos(sp)
            psi_ref[i] = ps
            ddy = -A * omega * omega * math.sin(omega * sl)
            denom = (1.0 + dy * dy) ** 1.5
            kappa_ref[i] = (ddy / denom) if denom > 1e-6 else 0.0

    return x_ref, y_ref, psi_ref, kappa_ref, total_len


def main():
    ap = argparse.ArgumentParser(description="车道保持仿真可视化")
    ap.add_argument("--file", default="output/sim_lane_keeping.txt",
                    help="仿真输出文件")
    ap.add_argument("--save", default=None, help="保存图片路径")
    args = ap.parse_args()

    d, ref = load_data(args.file)
    N = len(d["time"])
    t = d["time"]

    # 重建参考路径
    x_ref, y_ref, psi_ref, kappa_ref, total_len = rebuild_reference_path(N, ref)
    print(f"  路径总长: {total_len:.1f}m, 数据步数: {N}")

    # 车辆绝对坐标
    car_x = x_ref - d["e_y"] * np.sin(psi_ref)
    car_y = y_ref + d["e_y"] * np.cos(psi_ref)

    # 车道边界
    lane_lx = x_ref - LANE_WIDTH / 2 * np.sin(psi_ref)
    lane_ly = y_ref + LANE_WIDTH / 2 * np.cos(psi_ref)
    lane_rx = x_ref + LANE_WIDTH / 2 * np.sin(psi_ref)
    lane_ry = y_ref - LANE_WIDTH / 2 * np.cos(psi_ref)

    ey_deg = np.rad2deg(d["e_psi"])
    steer_deg = np.rad2deg(d["steer"])

    # ===== 建图 =====
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle("MPC Lane Keeping — Visualization", fontsize=16)

    # ---- 1. 全局俯视图 ----
    ax_map = plt.subplot(2, 2, (1, 3))
    ax_map.set_aspect("equal")
    ax_map.set_xlabel("X (m)")
    ax_map.set_ylabel("Y (m)")
    ax_map.set_title("Global Path & Vehicle Trajectory")
    ax_map.grid(True, alpha=0.3)

    # 参考路径与车道线
    ax_map.plot(x_ref, y_ref, "b--", lw=1.2, alpha=0.6, label="Reference Path")
    ax_map.plot(lane_lx, lane_ly, "gray", lw=0.8, ls=":", alpha=0.4, label="Lane Boundary")
    ax_map.plot(lane_rx, lane_ry, "gray", lw=0.8, ls=":", alpha=0.4)

    # 车辆轨迹（抽稀）
    step_skip = max(1, N // 500)
    ax_map.plot(car_x[::step_skip], car_y[::step_skip], "r-", lw=1.5, alpha=0.7, label="Vehicle Trajectory")

    # 起点 / 终点
    ax_map.plot(x_ref[0], y_ref[0], "go", ms=8, label="Start")
    ax_map.plot(x_ref[-1], y_ref[-1], "mo", ms=8, label="End")

    # 等间距标记车辆朝向
    for i in range(0, N, N // 8):
        ax_map.plot(car_x[i], car_y[i], "r.", ms=6)
        dx, dy = math.cos(psi_ref[i] + d["e_psi"][i]), math.sin(psi_ref[i] + d["e_psi"][i])
        ax_map.arrow(car_x[i], car_y[i], dx * 3, dy * 3, head_width=1.5, fc="r", ec="r", alpha=0.5)

    ax_map.legend(loc="lower right", fontsize=8)

    # ---- 2. 横向误差 e_y ----
    ax_ey = plt.subplot(2, 2, 2)
    ax_ey.plot(t, d["e_y"], "b-", lw=1.5)
    ax_ey.axhline(0, color="k", ls="--", lw=0.5)
    ax_ey.set_ylabel("e_y (m)")
    ax_ey.set_title("Lateral Error")
    ax_ey.grid(True, alpha=0.3)
    ax_ey.set_xlim(t[0], t[-1])

    # ---- 3. 航向误差 e_psi ----
    ax_ep = plt.subplot(2, 2, 4)
    ax_ep.plot(t, ey_deg, "r-", lw=1.5)
    ax_ep.axhline(0, color="k", ls="--", lw=0.5)
    ax_ep.set_xlabel("Time (s)")
    ax_ep.set_ylabel("e_psi (deg)")
    ax_ep.set_title("Heading Error")
    ax_ep.grid(True, alpha=0.3)
    ax_ep.set_xlim(t[0], t[-1])

    # ---- (附加窗口) 方向盘转角 ----
    # 在右侧添加一个独立的 steer 子图
    # 和 e_psi 共用 x 轴更方便

    plt.tight_layout()

    # 单独再画一张 steer + kappa 的图
    fig2, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, steer_deg, "g-", lw=1.5, label="Steer")
    ax.plot(t, np.rad2deg(np.arctan(L_WB * kappa_ref)), "k--", lw=1.0, alpha=0.5, label="κ feedforward ref")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Steer (deg)")
    ax.set_title("Steering Angle")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim(t[0], t[-1])
    fig2.tight_layout()

    # 统计
    steady = d["time"] > 5.0
    ey_rms = np.sqrt(np.mean(d["e_y"][steady] ** 2))
    ep_rms = np.sqrt(np.mean(d["e_psi"][steady] ** 2))
    print(f"\n  Steady-state (t > 5s):")
    print(f"    e_y  RMS: {ey_rms:.4f} m")
    print(f"    e_psi RMS: {np.rad2deg(ep_rms):.4f}°")

    if args.save:
        fig.savefig(args.save, dpi=150, bbox_inches="tight")
        steer_path = args.save.replace(".png", "_steer.png")
        fig2.savefig(steer_path, dpi=150, bbox_inches="tight")
        print(f"  图片已保存: {args.save}, {steer_path}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
