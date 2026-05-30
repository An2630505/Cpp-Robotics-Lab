import matplotlib.pyplot as plt
import numpy as np
import os
import sys

# 设置字体
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial Unicode MS', 'Liberation Sans']
plt.rcParams['axes.unicode_minus'] = True

# ========================== 参数配置 ==========================
GRID_SIZE = 256                # 网格大小 256x256
OBSTACLE_RATIO = 0.2           # 障碍物占比 (0.0 ~ 1.0)
OUTPUT_DIR = 'output'          # 输出目录
OUTPUT_GRID_FILE = 'grid.txt'  # 网格数据文件名

# ========================== 随机种子 (可选固定) ==========================
# 取消下面一行注释可固定随机种子，便于复现
# np.random.seed(42)


def generate_grid(size, obstacle_ratio):
    """生成 size x size 的 Occupancy Grid Map (0=空闲, 1=障碍)。
       使用矩形块 + 墙壁，让地图结构更自然。"""
    grid = np.zeros((size, size), dtype=np.int8)
    target_obs = int(size * size * obstacle_ratio)
    placed = 0

    # 随机放置矩形块
    while placed < target_obs:
        w = np.random.randint(4, 31)
        h = np.random.randint(4, 31)
        r = np.random.randint(0, size)
        c = np.random.randint(0, size)

        r_end = min(r + h - 1, size - 1)
        c_end = min(c + w - 1, size - 1)

        # 跳过重叠超过 30% 的
        area = (r_end - r + 1) * (c_end - c + 1)
        already = int(np.sum(grid[r:r_end+1, c:c_end+1]))
        if already > area * 0.3:
            continue

        grid[r:r_end+1, c:c_end+1] = 1
        placed += area - already

    # 额外添加细长墙壁（横向 + 纵向）
    walls = size // 16
    for i in range(walls):
        length = np.random.randint(20, 81)
        r = np.random.randint(5, size - 5)
        c = np.random.randint(5, size - 5)

        if i % 2 == 0:
            # 横向墙 1~2 格厚
            thick = np.random.randint(1, 3)
            r_end = min(r + thick - 1, size - 1)
            c_end = min(c + length, size - 1)
            grid[r:r_end+1, c:c_end+1] = 1
        else:
            # 纵向墙 1~2 格厚
            thick = np.random.randint(1, 3)
            r_end = min(r + length, size - 1)
            c_end = min(c + thick - 1, size - 1)
            grid[r:r_end+1, c:c_end+1] = 1

    return grid


def generate_start_goal(grid, margin=10):
    """在空闲区域随机选取起点和终点，保证两者不重叠且有一定间距。"""
    size = grid.shape[0]
    free_rc = np.argwhere(grid == 0)

    # 过滤边界区域
    mask = ((free_rc[:, 0] >= margin) & (free_rc[:, 0] < size - margin) &
            (free_rc[:, 1] >= margin) & (free_rc[:, 1] < size - margin))
    valid = free_rc[mask]

    if len(valid) < 2:
        print("错误: 空闲格子不足，无法放置起点和终点!")
        sys.exit(1)

    min_dist = size * 0.3
    while True:
        idx = np.random.choice(len(valid), 2, replace=False)
        start = tuple(valid[idx[0]])
        goal  = tuple(valid[idx[1]])
        if np.linalg.norm(np.array(start) - np.array(goal)) >= min_dist:
            return start, goal


def save_grid(grid, start, goal, filepath):
    """将网格、起点、终点保存为文本文件，供 C++ A* 读取。"""
    size = grid.shape[0]
    with open(filepath, 'w') as f:
        f.write(f"# A* Grid Map\n")
        f.write(f"# size: {size}x{size}\n")
        f.write(f"# start: ({start[0]}, {start[1]})\n")
        f.write(f"# goal:  ({goal[0]}, {goal[1]})\n")
        f.write(f"# 0=free, 1=obstacle\n")
        f.write(f"{size}\n")
        f.write(f"{start[0]} {start[1]}\n")
        f.write(f"{goal[0]} {goal[1]}\n")
        for row in grid:
            f.write(' '.join(str(c) for c in row) + '\n')
    print(f"网格数据已保存至: {filepath}")


def visualize_grid(grid, start, goal, save_path=None):
    """可视化网格，起点蓝色、终点红色。"""
    fig, ax = plt.subplots(figsize=(8, 8))

    cmap = plt.get_cmap('gray_r', 2)
    ax.imshow(grid, cmap=cmap, origin='upper', interpolation='none')

    # 小网格显示格线
    if GRID_SIZE <= 64:
        for x in range(grid.shape[1] + 1):
            ax.axvline(x - 0.5, color='black', linewidth=0.3, alpha=0.3)
        for y in range(grid.shape[0] + 1):
            ax.axhline(y - 0.5, color='black', linewidth=0.3, alpha=0.3)

    # 起点 (蓝色圆)
    ax.plot(start[1], start[0], 'o', color='blue', markersize=10,
            markeredgecolor='darkblue', markeredgewidth=1.5, label='Start')
    # 终点 (红色方)
    ax.plot(goal[1], goal[0], 's', color='red', markersize=10,
            markeredgecolor='darkred', markeredgewidth=1.5, label='Goal')

    ax.set_title(f'A* Grid Map ({grid.shape[0]}x{grid.shape[1]})\n'
                 f'Start: ({start[0]}, {start[1]})  Goal: ({goal[0]}, {goal[1]})',
                 fontsize=14)
    ax.set_xlabel('Column')
    ax.set_ylabel('Row')
    ax.legend(loc='upper right')
    ax.set_xlim(-0.5, grid.shape[1] - 0.5)
    ax.set_ylim(grid.shape[0] - 0.5, -0.5)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"图片已保存至: {save_path}")

    plt.show()


def main():
    print(f"=== A* 网格地图生成 (Python) ===")
    print(f"网格大小: {GRID_SIZE}x{GRID_SIZE}")
    print(f"障碍物占比: {OBSTACLE_RATIO*100:.0f}%")

    # 1. 生成网格
    grid = generate_grid(GRID_SIZE, OBSTACLE_RATIO)
    print(f"空闲格子: {np.sum(grid == 0)}, 障碍物: {np.sum(grid == 1)}")

    # 2. 生成起点和终点
    start, goal = generate_start_goal(grid)
    print(f"起点 (Start):  ({start[0]}, {start[1]})")
    print(f"终点 (Goal):   ({goal[0]}, {goal[1]})")
    print(f"欧几里得距离: {np.linalg.norm(np.array(start) - np.array(goal)):.2f}")

    # 3. 保存数据
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    save_grid(grid, start, goal, os.path.join(OUTPUT_DIR, OUTPUT_GRID_FILE))

    # 4. 可视化
    visualize_grid(grid, start, goal, save_path=os.path.join(OUTPUT_DIR, 'grid_map.png'))


if __name__ == '__main__':
    main()
