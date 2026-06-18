"""
sim_navigation_visualize.py — 端到端导航仿真可视化

用法:
  python pipeline/sim_navigation_visualize.py
  python pipeline/sim_navigation_visualize.py --save output/nav_result.png
"""

import sys, os, argparse, math
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt


def load_traj(filepath="output/sim_navigation.txt"):
    data = np.loadtxt(filepath, delimiter="\t", skiprows=4, dtype=float)
    return {
        "time": data[:, 0],
        "x": data[:, 1], "y": data[:, 2],
        "theta": data[:, 3],
        "v": data[:, 4], "steer": data[:, 5],
    }


def load_ref(filepath="output/nav_ref_path.txt"):
    path = []
    with open(filepath) as f:
        for line in f:
            if line.startswith("#"): continue
            parts = line.strip().split()
            if len(parts) >= 3:
                path.append((float(parts[0]), float(parts[1]), float(parts[2])))
    return path


def main():
    ap = argparse.ArgumentParser(description="导航可视化")
    ap.add_argument("--save", default=None, help="保存图片路径")
    ap.add_argument("--file", default="output/sim_navigation.txt")
    args = ap.parse_args()

    if not os.path.exists(args.file):
        print(f"文件不存在: {args.file}")
        print("请先运行: python pipeline/sim_navigation.py")
        return

    d = load_traj(args.file)
    ref = load_ref()
    print(f"  参考路径: {len(ref)} 点, 轨迹: {len(d['time'])} 步")

    t = d["time"]
    steer_deg = np.rad2deg(d["steer"])

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("End-to-End PNC Navigation", fontsize=14)

    # 左上: 全局
    ax = axes[0, 0]
    ax.plot([p[0] for p in ref], [p[1] for p in ref],
            "b--", lw=1.2, alpha=0.6, label="Reference Path")
    ax.plot(d["x"], d["y"], "g-", lw=1.5, alpha=0.9, label="Tracked Trajectory")
    ax.plot(ref[0][0], ref[0][1], "bo", ms=8, label="Start")
    ax.plot(ref[-1][0], ref[-1][1], "ro", ms=8, label="Goal")

    # 画几个朝向箭头
    step = max(1, len(d["x"]) // 20)
    for i in range(0, len(d["x"]), step):
        ax.arrow(d["x"][i], d["y"][i],
                 math.cos(d["theta"][i]) * 1.0,
                 math.sin(d["theta"][i]) * 1.0,
                 head_width=0.3, fc="green", ec="green", alpha=0.5)

    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title("Path Overview")
    ax.set_aspect("equal"); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # 右上: 速度
    ax = axes[0, 1]
    ax.plot(t, d["v"], "b-", lw=1.5)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Speed (m/s)")
    ax.set_title("Speed Profile")
    ax.grid(True, alpha=0.3)

    # 左下: 转向角
    ax = axes[1, 0]
    ax.plot(t, steer_deg, "r-", lw=1.5)
    ax.axhline(0, color="gray", ls=":", lw=0.5)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Steer (deg)")
    ax.set_title("Steering Angle")
    ax.grid(True, alpha=0.3)

    # 右下: 曲率
    ax = axes[1, 1]
    ds = np.hypot(np.diff(d["x"]), np.diff(d["y"]))
    ds = np.where(ds < 1e-6, 1e-6, ds)
    dth = np.diff(d["theta"])
    dth = np.where(dth > math.pi, dth - 2*math.pi, dth)
    dth = np.where(dth < -math.pi, dth + 2*math.pi, dth)
    kappa = dth / ds
    ax.plot(t[1:], kappa, "b-", lw=1.5)
    ax.axhline(0, color="gray", ls=":", lw=0.5)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Curvature (1/m)")
    ax.set_title("Path Curvature")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if args.save:
        plt.savefig(args.save, dpi=150, bbox_inches="tight")
        print(f"已保存: {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
