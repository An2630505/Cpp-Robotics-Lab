import matplotlib.pyplot as plt
import numpy as np

plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = True

# ========================== 读取数据 ==========================
def read_grid(filepath):
    with open(filepath) as f:
        lines = f.readlines()

    idx = 0
    while lines[idx].startswith('#'):
        idx += 1

    size = int(lines[idx].strip())
    idx += 1
    start = tuple(map(int, lines[idx].split()))
    idx += 1
    goal = tuple(map(int, lines[idx].split()))
    idx += 1

    grid = np.zeros((size, size), dtype=np.int8)
    for r in range(size):
        grid[r] = list(map(int, lines[idx + r].split()))

    return grid, start, goal

def read_path(filepath):
    path = []
    with open(filepath) as f:
        for line in f:
            if line.startswith('#'):
                continue
            r, c = map(int, line.split())
            path.append((r, c))
    return path

# ========================== 可视化 ==========================
grid, start, goal = read_grid('output/grid.txt')
path = read_path('output/path.txt')

print(f"网格: {grid.shape[0]}x{grid.shape[1]}")
print(f"起点: {start}, 终点: {goal}")
print(f"路径步数: {len(path)}")

fig, ax = plt.subplots(figsize=(10, 10))

# 网格
cmap = plt.get_cmap('gray_r', 2)
ax.imshow(grid, cmap=cmap, origin='upper', interpolation='none')

# 路径 (绿线 + 绿色点)
if path:
    pr = [p[0] for p in path]
    pc = [p[1] for p in path]
    ax.plot(pc, pr, '-', color='lime', linewidth=2.5, label=f'Path ({len(path)} steps)')
    ax.plot(pc[1:-1], pr[1:-1], 'o', color='green', markersize=1.5, alpha=0.7)

# 起点 (蓝)
ax.plot(start[1], start[0], 'o', color='blue', markersize=12,
        markeredgecolor='white', markeredgewidth=2, label='Start')
# 终点 (红)
ax.plot(goal[1], goal[0], 's', color='red', markersize=12,
        markeredgecolor='white', markeredgewidth=2, label='Goal')

# 统计
straight = np.hypot(goal[0] - start[0], goal[1] - start[1])
path_len = sum(np.hypot(path[i][0] - path[i-1][0], path[i][1] - path[i-1][1])
               for i in range(1, len(path)))
efficiency = straight / path_len * 100

ax.set_title(f'A* Pathfinding ({grid.shape[0]}x{grid.shape[1]})\n'
             f'Start: {start}  →  Goal: {goal}\n'
             f'Efficiency: {efficiency:.1f}%  |  Length: {path_len:.1f}  |  Steps: {len(path)}',
             fontsize=13)
ax.set_xlabel('Column')
ax.set_ylabel('Row')
ax.legend(loc='upper right', fontsize=11)
ax.set_xlim(-0.5, grid.shape[1] - 0.5)
ax.set_ylim(grid.shape[0] - 0.5, -0.5)

plt.tight_layout()
plt.savefig('output/path_result.png', dpi=200, bbox_inches='tight')
print("图片已保存至: output/path_result.png")
plt.show()
