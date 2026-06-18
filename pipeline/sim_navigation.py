"""
sim_navigation.py — 端到端 PNC 导航仿真

管线: 地图 → A*/Hybrid A* 路径规划 → 纯追踪轨迹生成 → 自行车模型跟踪

用法:
  python pipeline/sim_navigation.py                 # A* + 纯追踪
  python pipeline/sim_navigation.py --planner hybrid # Hybrid A* + 纯追踪
"""

import sys, os, math, argparse
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "build2", "pnc"))
import numpy as np
import pnc

# ========================== 参数 ==========================
GRID_RES   = 0.5
WHEELBASE  = 2.68
MAX_STEER  = 0.6
DT         = 0.2
V_DES      = 3.0


def generate_grid(size=64, ratio=0.3):
    np.random.seed(42)
    g = (np.random.rand(size, size) < ratio).astype(int).tolist()
    for r in range(size):
        for c in range(size):
            if r < 4 and c < 4:            g[r][c] = 0
            if r >= size-4 and c >= size-4: g[r][c] = 0
    return g


def astar_plan(grid, sr, sc, gr, gc):
    """A* → (row,col) 列表."""
    astar = pnc.AStar(grid, sr, sc, gr, gc)
    raw = astar.find_path()
    if not raw: return None
    return [(p.row, p.col) for p in raw]


def cells_to_ref_poses(cells, cell_size=GRID_RES):
    """(row,col) → Pose 序列 (x,y,theta)."""
    ps = []
    for i, (r, c) in enumerate(cells):
        p = pnc.Pose()
        p.x = c * cell_size + cell_size / 2
        p.y = r * cell_size + cell_size / 2
        if i + 1 < len(cells):
            p.theta = math.atan2(cells[i+1][0] - r, cells[i+1][1] - c)
        elif ps:
            p.theta = ps[-1].theta
        else:
            p.theta = 0
        ps.append(p)
    return ps


def hybrid_plan(grid, sr, sc, gr, gc, cell_size=GRID_RES):
    """Hybrid A* 规划."""
    # 膨胀
    ex = [row.copy() for row in grid]
    n = len(grid)
    for _ in range(2):
        tmp = [row.copy() for row in ex]
        for r in range(1, n-1):
            for c in range(1, n-1):
                if ex[r][c] == 0:
                    for dr in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            tmp[r+dr][c+dc] = 0
        ex = tmp

    start = pnc.Pose()
    start.x = sc * cell_size + cell_size / 2
    start.y = sr * cell_size + cell_size / 2
    start.theta = 0
    goal = pnc.Pose()
    goal.x = gc * cell_size + cell_size / 2
    goal.y = gr * cell_size + cell_size / 2
    goal.theta = 0

    planner = pnc.HybridAStar(ex)
    planner.set_cell_size(cell_size)
    planner.set_goal_xy_tol(1.5)
    planner.set_goal_th_tol(0.8)
    path = planner.plan(start, goal)
    if not path: return None
    return [(p.x, p.y, p.theta) for p in path]


def run():
    ap = argparse.ArgumentParser(description="端到端 PNC 导航仿真")
    ap.add_argument("--planner", choices=["astar", "hybrid"], default="astar")
    ap.add_argument("--size", type=int, default=64)
    args = ap.parse_args()

    print("=" * 55)
    print("  端到端 PNC 导航仿真")
    print(f"  管线: 地图 → {args.planner.upper()} → 纯追踪")
    print("=" * 55)

    # ---- 1. 地图 ----
    grid = generate_grid(args.size, 0.3)
    n = len(grid)
    free = sum(1 for r in range(n) for c in range(n) if grid[r][c] == 0)
    print(f"\n[1] 地图: {n}×{n}, free={free}, obs={n*n-free}")

    # ---- 2. 路径规划 ----
    sr, sc, gr, gc = 1, 1, n - 2, n - 2
    print(f"[2] 路径规划 ({args.planner}): ({sr},{sc})→({gr},{gc})")

    if args.planner == "hybrid":
        path_pts = hybrid_plan(grid, sr, sc, gr, gc, GRID_RES)
        if not path_pts:
            print("  ❌ Hybrid A* 未找到路径!")
            return
        # 转为 Pose 引用
        ref_poses = []
        for x, y, th in path_pts:
            p = pnc.Pose(); p.x = x; p.y = y; p.theta = th
            ref_poses.append(p)
        print(f"  ✅ {len(path_pts)} 步连续路径")
    else:
        cells = astar_plan(grid, sr, sc, gr, gc)
        if not cells:
            print("  ❌ A* 未找到路径!")
            return
        ref_poses = cells_to_ref_poses(cells, GRID_RES)
        path_len = sum(math.hypot(ref_poses[i].x - ref_poses[i-1].x,
                                  ref_poses[i].y - ref_poses[i-1].y)
                       for i in range(1, len(ref_poses)))
        print(f"  ✅ {len(cells)} 步, {path_len:.1f}m")

    # 路径数据记录
    ref_pts = [(p.x, p.y, p.theta) for p in ref_poses]
    ref_len = sum(math.hypot(ref_pts[i][0] - ref_pts[i-1][0],
                             ref_pts[i][1] - ref_pts[i-1][1])
                  for i in range(1, len(ref_pts)))
    print(f"[3] 参考路径: {len(ref_pts)} 点, {ref_len:.1f}m")

    # ---- 3. 纯追踪跟踪 ----
    total_N = max(30, int(ref_len / (V_DES * DT) * 1.5) + 10)
    print(f"[4] 纯追踪跟踪: N={total_N}, dt={DT}s, V={V_DES}m/s")

    tracker = pnc.MPCTrajectoryPlanner(grid)
    tracker.set_dt(DT)
    tracker.set_horizon(total_N)
    tracker.set_desired_speed(V_DES)
    traj, vels, steers = tracker.plan(ref_poses)

    # 控制序列
    exec_poses = [(traj[k].x, traj[k].y, traj[k].theta) for k in range(len(traj))]
    exec_steers = [steers[k] for k in range(len(steers))]
    exec_vels   = [vels[k] for k in range(len(vels))]

    print(f"  ✅ 轨迹: {len(exec_poses)} 步, "
          f"max steer={max(abs(s) for s in exec_steers)*180/math.pi:.1f}°")

    # ---- 4. 输出 ----
    os.makedirs("output", exist_ok=True)

    with open("output/nav_ref_path.txt", "w") as f:
        f.write("# Reference Path\n# x y theta\n")
        for x, y, th in ref_pts:
            f.write(f"{x:.4f} {y:.4f} {th:.4f}\n")

    outpath = "output/sim_navigation.txt"
    with open(outpath, "w") as f:
        f.write("# PNC Navigation E2E\n")
        f.write(f"# Planner: {args.planner}, ref: {len(ref_pts)} pts, "
                f"{ref_len:.1f}m\n")
        f.write(f"# Trajectory: {len(exec_poses)} steps\n")
        f.write("time\tx\ty\ttheta\tv\tsteer\n")
        for k in range(len(exec_poses)):
            t = k * DT
            x, y, th = exec_poses[k]
            v = exec_vels[k] if k < len(exec_vels) else 0.0
            s = exec_steers[k] if k < len(exec_steers) else 0.0
            f.write(f"{t:.3f}\t{x:.4f}\t{y:.4f}\t{th:.4f}\t{v:.4f}\t{s:.4f}\n")

    # 统计
    final_dist = math.hypot(exec_poses[-1][0] - ref_pts[-1][0],
                            exec_poses[-1][1] - ref_pts[-1][1])
    print(f"\n[5] 仿真结果:")
    print(f"  轨迹时长: {len(exec_poses)*DT:.1f}s")
    print(f"  终点偏差: {final_dist:.3f}m")
    print(f"  输出: {outpath}")
    print("✅ pipeline/sim_navigation.py 跑通")


if __name__ == "__main__":
    run()
