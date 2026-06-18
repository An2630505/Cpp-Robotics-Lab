"""
sim_path_planning_visualize.py — 路径规划结果可视化

用法:
  python pipeline/sim_path_planning_visualize.py                    # 默认读 astar_path.txt
  python pipeline/sim_path_planning_visualize.py --file output/astar_path.txt
  python pipeline/sim_path_planning_visualize.py --save output/planning.png
"""

import sys, os, argparse, math
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt


def load_astar_path(filepath):
    """读取 A* 路径文件 (row col)."""
    path = []
    with open(filepath) as f:
        for line in f:
            if line.startswith("#"):
                continue
            parts = line.strip().split()
            if len(parts) >= 2:
                path.append((int(parts[0]), int(parts[1])))
    return path


def load_hybrid_path(filepath):
    """读取 Hybrid A* 路径文件 (x y theta)."""
    path = []
    with open(filepath) as f:
        for line in f:
            if line.startswith("#"):
                continue
            parts = line.strip().split()
            if len(parts) >= 3:
                path.append((float(parts[0]), float(parts[1]), float(parts[2])))
    return path


def main():
    ap = argparse.ArgumentParser(description="路径规划可视化")
    ap.add_argument("--file", default="output/astar_path.txt",
                    help="路径文件")
    ap.add_argument("--grid", default=None, help="网格文件 (可选)")
    ap.add_argument("--save", default=None, help="保存图片路径")
    args = ap.parse_args()

    if not os.path.exists(args.file):
        print(f"文件不存在: {args.file}")
        print("请先运行: python pipeline/sim_path_planning.py")
        return

    is_hybrid = "hybrid" in args.file
    if is_hybrid:
        path = load_hybrid_path(args.file)
        print(f"加载 Hybrid A* 路径: {len(path)} 点")
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        xlabel, ylabel = "X (m)", "Y (m)"
    else:
        path = load_astar_path(args.file)
        print(f"加载 A* 路径: {len(path)} 点")
        # row→y, col→x
        xs = [p[1] for p in path]
        ys = [p[0] for p in path]
        xlabel, ylabel = "Col", "Row"

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Path Planning Result", fontsize=14)

    # 左: 路径图
    ax = axes[0]
    ax.plot(xs, ys, "g-", lw=2, label="Path")
    ax.plot(xs[0], ys[0], "bo", ms=8, label="Start")
    ax.plot(xs[-1], ys[-1], "ro", ms=8, label="Goal")

    if is_hybrid:
        # 画朝向箭头
        for i in range(0, len(path), max(1, len(path) // 40)):
            x, y, th = path[i]
            dx, dy = math.cos(th) * 0.3, math.sin(th) * 0.3
            ax.arrow(x, y, dx, dy, head_width=0.1, fc="red", ec="red", alpha=0.6)

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title("Path Overview")
    ax.set_aspect("equal")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.invert_yaxis()

    # 右: 曲率分析 (仅 Hybrid A*)
    if is_hybrid:
        thetas = np.array([p[2] for p in path])
        ds = np.hypot(np.diff(xs), np.diff(ys))
        ds = np.where(ds < 1e-6, 1e-6, ds)
        dtheta = np.diff(thetas)
        dtheta = np.where(dtheta > math.pi, dtheta - 2 * math.pi, dtheta)
        dtheta = np.where(dtheta < -math.pi, dtheta + 2 * math.pi, dtheta)
        kappa = dtheta / ds

        ax2 = axes[1]
        ax2.plot(range(len(kappa)), kappa, "b-", lw=1)
        ax2.axhline(0, color="gray", ls=":", lw=0.5)
        ax2.set_xlabel("Step")
        ax2.set_ylabel("Curvature (1/m)")
        ax2.set_title("Path Curvature")
        ax2.grid(True, alpha=0.3)
    else:
        # 步长分析
        seg_lens = [math.hypot(xs[i] - xs[i-1], ys[i] - ys[i-1])
                    for i in range(1, len(xs))]
        ax2 = axes[1]
        ax2.plot(range(1, len(xs)), seg_lens, "b.-", ms=3)
        ax2.axhline(1.0, color="gray", ls="--", lw=0.5, label="orthogonal")
        ax2.axhline(math.sqrt(2), color="orange", ls="--", lw=0.5, label="diagonal")
        ax2.set_xlabel("Step")
        ax2.set_ylabel("Segment Length")
        ax2.set_title("Step Lengths")
        ax2.legend(fontsize=8)
        ax2.grid(True, alpha=0.3)

    plt.tight_layout()

    if args.save:
        plt.savefig(args.save, dpi=150, bbox_inches="tight")
        print(f"已保存: {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
