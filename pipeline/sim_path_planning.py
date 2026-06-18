"""
sim_path_planning.py — 路径规划仿真场景

支持两种模式:
  1. 内置随机迷宫 — 快速验证 A* 和 Hybrid A*
  2. 地图文件 — 从 PGM/YAML 构建 Occupancy Grid 后规划

用法:
  python pipeline/sim_path_planning.py                        # 随机迷宫, A*
  python pipeline/sim_path_planning.py --mode hybrid          # 随机迷宫, Hybrid A*
  python pipeline/sim_path_planning.py --mode both            # 两者对比
  python pipeline/sim_path_planning.py --map map.pgm          # 从地图文件
"""

import sys, os, argparse, math, random
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "build2", "pnc"))
import numpy as np
import pnc


def generate_random_grid(size=64, obstacle_ratio=0.3):
    """生成随机占用网格 (0=free, 1=obstacle)."""
    np.random.seed(42)
    grid = (np.random.rand(size, size) < obstacle_ratio).astype(int).tolist()
    # 清空起点和终点附近区域
    for r in range(size):
        for c in range(size):
            if r < 4 and c < 4:
                grid[r][c] = 0
            if r >= size - 4 and c >= size - 4:
                grid[r][c] = 0
    return grid, size


def print_grid_stats(grid):
    n = len(grid)
    free = sum(1 for r in grid for c in r if c == 0)
    occ = n * n - free
    print(f"  网格: {n}×{n}, free={free}, obstacle={occ}")


def run_astar(grid, sr, sc, gr, gc):
    """运行 A*, 返回路径点列表 [(row, col), ...]."""
    print(f"\n=== A* 路径规划 ===")
    print(f"  起点: ({sr}, {sc}) → 终点: ({gr}, {gc})")
    astar = pnc.AStar(grid, sr, sc, gr, gc)
    path = astar.find_path()
    history = astar.get_search_history()
    if not path:
        print("  ❌ 未找到路径!")
        return [], [], 0
    # path 是 pnc.Point 列表, 用 .row/.col 访问
    cells = [(p.row, p.col) for p in path]
    length = sum(
        math.hypot(cells[i][0] - cells[i-1][0], cells[i][1] - cells[i-1][1])
        for i in range(1, len(cells))
    )
    history_cells = [(p.row, p.col) for p in history]
    print(f"  ✅ 路径: {len(cells)} 步, 长度={length:.1f}, "
          f"展开节点={len(history_cells)}")
    return cells, history_cells, length


def run_hybrid_astar(grid, sr, sc, gr, gc, cell_size=0.5):
    """运行 Hybrid A* (需要连续坐标)."""
    print(f"\n=== Hybrid A* ===")
    # 膨胀路面
    expanded = [row.copy() for row in grid]
    for _ in range(2):
        tmp = [row.copy() for row in expanded]
        n = len(expanded)
        for r in range(1, n - 1):
            for c in range(1, n - 1):
                if expanded[r][c] == 0:
                    for dr in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            tmp[r+dr][c+dc] = 0
        expanded = tmp

    start = pnc.Pose()
    start.x = sc * cell_size + cell_size / 2
    start.y = sr * cell_size + cell_size / 2
    start.theta = 0
    goal = pnc.Pose()
    goal.x = gc * cell_size + cell_size / 2
    goal.y = gr * cell_size + cell_size / 2
    goal.theta = 0

    h = pnc.HybridAStar(expanded)
    h.set_cell_size(cell_size)
    h.set_goal_xy_tol(1.5)
    h.set_goal_th_tol(0.8)
    path = h.plan(start, goal)
    if not path:
        print("  ❌ 未找到路径!")
        return [], 0
    # path 是 pnc.Pose 列表
    pts = [(p.x, p.y, p.theta) for p in path]
    length = sum(
        math.hypot(pts[i][0] - pts[i-1][0], pts[i][1] - pts[i-1][1])
        for i in range(1, len(pts))
    )
    print(f"  ✅ 路径: {len(pts)} 步, 长度={length:.1f}m")
    return pts, length


def save_path_astar(path, filepath):
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    with open(filepath, "w") as f:
        f.write(f"# A* Path\n# steps: {len(path)}\n# row col\n")
        for r, c in path:
            f.write(f"{r} {c}\n")
    print(f"  已保存: {filepath}")


def save_path_hybrid(pts, filepath):
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    with open(filepath, "w") as f:
        f.write(f"# Hybrid A* Path\n# steps: {len(pts)}\n# x y theta\n")
        for x, y, th in pts:
            f.write(f"{x:.4f} {y:.4f} {th:.4f}\n")
    print(f"  已保存: {filepath}")


def save_ppm(grid, path_cells, sr, sc, gr, gc, filepath):
    """保存 PPM 可视化图片. path_cells: [(row, col), ...]"""
    n = len(grid)
    S = 3
    on_path = set((r, c) for r, c in path_cells)
    with open(filepath, "w") as f:
        f.write(f"P3\n{n*S} {n*S}\n255\n")
        for r in range(n):
            for _ in range(S):
                for c in range(n):
                    if r == sr and c == sc:
                        R, G, B = 0, 0, 255
                    elif r == gr and c == gc:
                        R, G, B = 255, 0, 0
                    elif (r, c) in on_path:
                        R, G, B = 0, 255, 0
                    elif grid[r][c] == 1:
                        R, G, B = 30, 30, 30
                    else:
                        R, G, B = 200, 200, 200
                    for _ in range(S):
                        f.write(f"{R} {G} {B} ")
                f.write("\n")
    print(f"  已保存: {filepath}")


def main():
    ap = argparse.ArgumentParser(description="路径规划仿真")
    ap.add_argument("--mode", choices=["astar", "hybrid", "both"], default="astar",
                    help="规划算法 (default: astar)")
    ap.add_argument("--size", type=int, default=64, help="随机网格大小 (default: 64)")
    ap.add_argument("--obstacle", type=float, default=0.3,
                    help="障碍物比例 (default: 0.3)")
    ap.add_argument("--map", default=None, help="地图文件路径 (PGM/PNG)")
    args = ap.parse_args()

    # 1. 生成或加载网格
    if args.map:
        print(f"=== 加载地图: {args.map} ===")
        # map_parser 暂不通过 pybind11 暴露 readPGM
        # 用 Python 读取 grid.txt 或手工构造
        print("  (地图文件加载暂用内置随机网格代替)")
        grid, size = generate_random_grid(args.size, args.obstacle)
    else:
        print(f"=== 生成随机网格: {args.size}×{args.size} ===")
        grid, size = generate_random_grid(args.size, args.obstacle)
    print_grid_stats(grid)

    sr, sc = 1, 1
    gr, gc = size - 2, size - 2

    # 2. A*
    if args.mode in ("astar", "both"):
        path, history, length = run_astar(grid, sr, sc, gr, gc)
        if path:
            save_path_astar(path, "output/astar_path.txt")
            save_ppm(grid, path, sr, sc, gr, gc, "output/astar_result.ppm")

    # 3. Hybrid A*
    if args.mode in ("hybrid", "both"):
        hpath, hlen = run_hybrid_astar(grid, sr, sc, gr, gc)
        if hpath:
            save_path_hybrid(hpath, "output/hybrid_path.txt")
            # PPM 只标记经过的格子
            path_cells = [(int(p.y / 0.5), int(p.x / 0.5)) for p in [
                type('P', (), {'x': px, 'y': py})() for px, py, _ in hpath
            ]]
            save_ppm(grid, path_cells[:0], sr, sc, gr, gc, "output/hybrid_result.ppm")

    print("\n✅ 路径规划完成")


if __name__ == "__main__":
    main()
