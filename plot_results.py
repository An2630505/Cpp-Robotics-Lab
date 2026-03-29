import matplotlib.pyplot as plt
import numpy as np

# 设置字体（使用 DejaVu Sans，支持英文和符号）
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial Unicode MS', 'Liberation Sans']
plt.rcParams['axes.unicode_minus'] = True  # 正常显示负号

# 读取数据文件
data = []
with open('output/output.txt', 'r') as f:
    for line in f:
        # 跳过注释行、标题行和空行
        if line.startswith('#') or line.startswith('Step') or not line.strip():
            continue
        
        # 解析数据行：step, y[0], y[1], u[0], u[1]
        parts = line.strip().split(', ')
        if len(parts) == 5:
            step = int(parts[0])
            y0 = float(parts[1])
            y1 = float(parts[2])
            u0 = float(parts[3])
            u1 = float(parts[4])
            data.append([step, y0, y1, u0, u1])

# 转换为 numpy 数组
data = np.array(data)
steps = data[:, 0]  # 时间步（step）
y0 = data[:, 1]     # y[0]
y1 = data[:, 2]     # y[1]
u0 = data[:, 3]     # u[0]
u1 = data[:, 4]     # u[1]

# 假设时间步长为 0.1s（根据 main.cpp 中的 dt）
dt = 0.1
time = steps * dt

# 创建 3 个子图
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
fig.suptitle('result', fontsize=16)

# ===== 子图 1: y[0] vs y[1] 相平面图 =====
ax1.plot(y0, y1, 'b-', linewidth=2, marker='o', markersize=3, alpha=0.6)
ax1.set_xlabel('y[0] - pos x', fontsize=12)
ax1.set_ylabel('y[1] - pos y', fontsize=12)
ax1.set_title('traj: y[0] vs y[1]', fontsize=14)
ax1.grid(True, alpha=0.3)
ax1.axis('equal')  # 保持 x、y 轴比例一致

# 标注起点和终点
ax1.plot(y0[0], y1[0], 'go', markersize=10, label='start')
ax1.plot(y0[-1], y1[-1], 'ro', markersize=10, label='end')
ax1.legend()

# ===== 子图 2: y[0] 和 y[1] 随时间变化曲线 =====
ax2.plot(time, y0, 'b-', linewidth=2, label='y[0]', alpha=0.7)
ax2.plot(time, y1, 'r-', linewidth=2, label='y[1]', alpha=0.7)
ax2.set_xlabel('time (s)', fontsize=12)
ax2.set_ylabel('output', fontsize=12)
ax2.set_title('output and time', fontsize=14)
ax2.grid(True, alpha=0.3)
ax2.legend(loc='best')
ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.3)

# ===== 子图 3: u[0] 和 u[1] 随时间变化曲线（控制输入） =====
ax3.plot(time, u0, 'g-', linewidth=2, label='u[0]', alpha=0.7)
ax3.plot(time, u1, 'm-', linewidth=2, label='u[1]', alpha=0.7)
ax3.set_xlabel('time (s)', fontsize=12)
ax3.set_ylabel('input', fontsize=12)
ax3.set_title('input and time', fontsize=14)
ax3.grid(True, alpha=0.3)
ax3.legend(loc='best')
ax3.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.3)

# 调整布局
plt.tight_layout()

# 保存图片
plt.savefig('output/simulation_results.png', dpi=300, bbox_inches='tight')
print('图表已保存为 simulation_results.png')

# 显示图表
plt.show()

# 打印统计信息
print('\n=== 数据统计 ===')
print(f'总步数：{len(steps)}')
print(f'时间范围：{time[0]:.2f}s ~ {time[-1]:.2f}s')
print(f'\ny[0] 统计:')
print(f'  最小值：{y0.min():.4f}')
print(f'  最大值：{y0.max():.4f}')
print(f'  平均值：{y0.mean():.4f}')
print(f'\ny[1] 统计:')
print(f'  最小值：{y1.min():.4f}')
print(f'  最大值：{y1.max():.4f}')
print(f'  平均值：{y1.mean():.4f}')