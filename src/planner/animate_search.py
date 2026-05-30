"""
A* 搜索过程动画
读取 grid.txt / path.txt / search_history.txt，生成搜索过程动画。
用法:
  python3 src/planner/animate_search.py          # 交互式预览
  python3 src/planner/animate_search.py --save   # 保存 GIF/MP4
"""

import matplotlib
matplotlib.use('TkAgg')  # 交互式后端，支持播放
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from matplotlib.colors import ListedColormap
import numpy as np
import os
import sys

# ========================== 参数 ==========================
FRAMES = 200                    # 总帧数
FPS = 15                        # GIF 帧率
OUTPUT_GIF = 'output/astar_animation.gif'
OUTPUT_MP4 = 'output/astar_animation.mp4'

SAVE_MODE = '--save' in sys.argv

# ========================== 颜色映射 ==========================
# 0=unexplored  1=obstacle  2=expanded  3=frontier  4=path  5=start  6=goal
COLORS = ['white',          # 0: 未探索
          (0.15, 0.15, 0.15),  # 1: 障碍 (深灰)
          (0.65, 0.85, 1.0),  # 2: 已展开 (淡蓝)
          (1.0, 0.55, 0.2),   # 3: 前沿 (橙色)
          (0.2, 0.9, 0.2),    # 4: 路径 (绿)
          (0.0, 0.3, 0.9),    # 5: 起点 (蓝)
          (0.9, 0.15, 0.1)]   # 6: 终点 (红)

cmap = ListedColormap(COLORS)

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
            parts = line.split()
            if len(parts) == 2:
                path.append((int(parts[0]), int(parts[1])))
    return path

def read_search_history(filepath):
    history = []
    with open(filepath) as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) == 2:
                history.append((int(parts[0]), int(parts[1])))
    return history

# ========================== 前沿计算 ==========================
def compute_frontier(expanded_mask, grid):
    """
    计算当前搜索前沿：与已展开区域相邻的未探索空闲格子。
    """
    size = grid.shape[0]
    frontier = np.zeros((size, size), dtype=bool)
    # 对每个已展开格子检查 8 个邻居
    expanded_rc = np.argwhere(expanded_mask)
    if len(expanded_rc) == 0:
        return frontier

    dirs = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    for r, c in expanded_rc:
        for dr, dc in dirs:
            nr, nc = r + dr, c + dc
            if 0 <= nr < size and 0 <= nc < size:
                if grid[nr, nc] == 0 and not expanded_mask[nr, nc]:
                    frontier[nr, nc] = True
    return frontier

# ========================== 主流程 ==========================
print("读取数据...")
grid, start, goal = read_grid('output/grid.txt')
path = read_path('output/path.txt')
history = read_search_history('output/search_history.txt')

size = grid.shape[0]
print(f"网格: {size}x{size}, 展开: {len(history)}, 路径: {len(path)} 步")

# ---- 找到目标被展开的时刻 ----
goal_expanded_idx = None
for i, (r, c) in enumerate(history):
    if (r, c) == goal:
        goal_expanded_idx = i
        break

if goal_expanded_idx is None:
    print("警告: 搜索历史中未找到目标节点!")
    goal_expanded_idx = len(history) - 1

print(f"目标在第 {goal_expanded_idx + 1} 次展开时被找到 "
      f"({goal_expanded_idx / len(history) * 100:.1f}%)")

# ---- 分配帧数 ----
# 搜索阶段: 从起点展开到目标 (占总帧数 80%)
# 回溯阶段: 从目标沿 parent 链回到起点 (占总帧数 20%)
SEARCH_FRAMES = int(FRAMES * 0.8)
BACKTRACK_FRAMES = FRAMES - SEARCH_FRAMES

# 搜索阶段每帧展开的节点数 (精确覆盖到 goal_expanded_idx)
search_stride = max(1, (goal_expanded_idx + 1) // SEARCH_FRAMES)

# 构建可视化数组
viz = np.zeros((size, size), dtype=np.int8)
viz[grid == 1] = 1             # 障碍
viz[start[0], start[1]] = 5    # 起点
viz[goal[0], goal[1]] = 6      # 终点

# 路径查找表 (排除起点/终点)
path_mask = np.zeros((size, size), dtype=bool)
for r, c in path:
    path_mask[r, c] = True
path_mask[start[0], start[1]] = False
path_mask[goal[0], goal[1]] = False

# A* 回溯是从终点沿 parent 链倒退到起点: path[0]=start, path[-1]=goal
reversed_path = list(reversed(path))  # [goal, ..., start]

print(f"搜索阶段: {SEARCH_FRAMES} 帧, 回溯阶段: {BACKTRACK_FRAMES} 帧, FPS: {FPS}")

# ---- 创建画布 ----
fig, (ax_img, ax_bar) = plt.subplots(2, 1, figsize=(8, 9.5),
    gridspec_kw={'height_ratios': [8, 0.3]})

ax_img.set_title('A* Search', fontsize=14)
im = ax_img.imshow(viz, cmap=cmap, vmin=0, vmax=6,
                   origin='upper', interpolation='none',
                   aspect='equal')
ax_img.set_xlim(-0.5, size - 0.5)
ax_img.set_ylim(size - 0.5, -0.5)

from matplotlib.lines import Line2D
legend_elements = [
    Line2D([0], [0], marker='o', color='w', markerfacecolor=COLORS[5],
           markersize=10, label='Start'),
    Line2D([0], [0], marker='s', color='w', markerfacecolor=COLORS[6],
           markersize=10, label='Goal'),
    Line2D([0], [0], marker='s', color='w', markerfacecolor=COLORS[2],
           markersize=10, label='Expanded'),
    Line2D([0], [0], marker='s', color='w', markerfacecolor=COLORS[3],
           markersize=10, label='Frontier'),
    Line2D([0], [0], marker='s', color='w', markerfacecolor=COLORS[4],
           markersize=10, label='Path'),
]
ax_img.legend(handles=legend_elements, loc='upper right',
              fontsize=9, framealpha=0.9)

progress_bar = ax_bar.barh(0, 0, height=1, color=COLORS[2])
ax_bar.set_xlim(0, 100)
ax_bar.set_ylim(-0.5, 0.5)
ax_bar.set_yticks([])
ax_bar.set_xlabel('Progress')
ax_bar.set_title('0%', fontsize=11)

text_info = ax_img.text(0.02, 0.02, '', transform=ax_img.transAxes,
                        fontsize=10, color='black',
                        verticalalignment='bottom',
                        bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

plt.tight_layout()

# ---- 预计算：搜索阶段每帧状态 ----
print("预计算搜索阶段帧状态...")
search_expanded_masks = []  # 第 k 帧已展开的节点 (k=0..SEARCH_FRAMES-1)
for k in range(SEARCH_FRAMES):
    end_idx = min((k + 1) * search_stride, goal_expanded_idx + 1)
    mask = np.zeros((size, size), dtype=bool)
    for i in range(0, end_idx):
        r, c = history[i]
        mask[r, c] = True
    search_expanded_masks.append(mask)

search_frontier_masks = [compute_frontier(m, grid) for m in search_expanded_masks]
print(f"搜索阶段预计算完成: {SEARCH_FRAMES} 帧")

# ---- 动画更新函数 ----
def update(frame):
    if frame < SEARCH_FRAMES:
        # ===== 阶段一：搜索 =====
        expanded_mask = search_expanded_masks[frame]
        frontier = search_frontier_masks[frame]
        end_idx = min((frame + 1) * search_stride, goal_expanded_idx + 1)

        viz_copy = viz.copy()
        viz_copy[expanded_mask] = 2
        viz_copy[frontier] = 3
        viz_copy[start[0], start[1]] = 5
        viz_copy[goal[0], goal[1]] = 6

        # 目标被找到的那一帧，闪烁提示
        if frame == SEARCH_FRAMES - 1:
            text_info.set_text(f'Goal reached! ({len(path)} steps path found)')
        else:
            text_info.set_text(f'Searching... {end_idx}/{goal_expanded_idx + 1} nodes')

        pct = (frame + 1) / FRAMES * 100
        bar_color = COLORS[2]

    else:
        # ===== 阶段二：回溯 (goal → start) =====
        back_frame = frame - SEARCH_FRAMES
        reveal_frac = (back_frame + 1) / BACKTRACK_FRAMES
        reveal_steps = int(len(reversed_path) * reveal_frac)

        # 搜索完成的最终状态
        viz_copy = viz.copy()
        viz_copy[search_expanded_masks[-1]] = 2
        viz_copy[search_frontier_masks[-1]] = 3

        # 路径从终点向起点回溯
        for r, c in reversed_path[:reveal_steps]:
            if (r, c) != start and (r, c) != goal:
                viz_copy[r, c] = 4
        # 当前回溯位置高亮
        if reveal_steps > 0 and reveal_steps < len(reversed_path):
            br, bc = reversed_path[reveal_steps - 1]
            if (br, bc) != start and (br, bc) != goal:
                viz_copy[br, bc] = 0  # 白色闪烁表示回溯头

        viz_copy[start[0], start[1]] = 5
        viz_copy[goal[0], goal[1]] = 6

        text_info.set_text(f'Backtracking goal→start... {reveal_steps}/{len(path)}')
        pct = (frame + 1) / FRAMES * 100
        bar_color = COLORS[4]
        end_idx = goal_expanded_idx + 1
        frontier = search_frontier_masks[-1]

    im.set_array(viz_copy)

    progress_bar[0].set_width(pct)
    progress_bar[0].set_color(bar_color)
    ax_bar.set_title(f'{pct:.0f}%  |  {"Search" if frame < SEARCH_FRAMES else "Backtrack"}  |  '
                     f'Nodes: {end_idx}',
                     fontsize=10)

    return [im, progress_bar[0], text_info]

# ---- 生成/播放动画 ----
print("生成动画帧...")
ani = anim.FuncAnimation(fig, update, frames=FRAMES,
                         interval=1000//FPS, blit=False)

if SAVE_MODE:
    # 保存 GIF
    print(f"保存 GIF ({OUTPUT_GIF})...")
    ani.save(OUTPUT_GIF, writer='pillow', fps=FPS, dpi=100)
    print(f"GIF 已保存: {OUTPUT_GIF}")

    # 保存 MP4
    try:
        print(f"保存 MP4 ({OUTPUT_MP4})...")
        ani.save(OUTPUT_MP4, writer='ffmpeg', fps=FPS, dpi=150)
        print(f"MP4 已保存: {OUTPUT_MP4}")
    except Exception:
        print("MP4 保存失败 (可能缺少 ffmpeg)，GIF 已可用")
    plt.close()
else:
    # 交互式预览播放
    print("播放动画 (关闭窗口退出)...")
    plt.show()
