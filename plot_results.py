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
        
        # 解析数据行：step, y_meas[0], y_meas[1], y_filt[0], y_filt[1], u[0], u[1]
        parts = line.strip().split(', ')
        if len(parts) == 7:
            step = int(parts[0])
            y_meas_0 = float(parts[1])
            y_meas_1 = float(parts[2])
            y_filt_0 = float(parts[3])
            y_filt_1 = float(parts[4])
            u0 = float(parts[5])
            u1 = float(parts[6])
            data.append([step, y_meas_0, y_meas_1, y_filt_0, y_filt_1, u0, u1])

# 转换为 numpy 数组
data = np.array(data)
steps = data[:, 0]  # 时间步（step）
y_meas_0 = data[:, 1]     # 滤波前测量值 y[0]
y_meas_1 = data[:, 2]     # 滤波前测量值 y[1]
y_filt_0 = data[:, 3]     # 滤波后估计值 y[0]
y_filt_1 = data[:, 4]     # 滤波后估计值 y[1]
u0 = data[:, 5]     # u[0]
u1 = data[:, 6]     # u[1]

# 假设时间步长为 0.1s（根据 main.cpp 中的 dt）
dt = 0.1
time = steps * dt

# 打印统计信息
print('\n=== 数据统计 ===')
print(f'总步数：{len(steps)}')
print(f'时间范围：{time[0]:.2f}s ~ {time[-1]:.2f}s')
print(f'\ny[0] (滤波前 - 测量值) 统计:')
print(f'  最小值：{y_meas_0.min():.4f}')
print(f'  最大值：{y_meas_0.max():.4f}')
print(f'  平均值：{y_meas_0.mean():.4f}')
print(f'  标准差：{y_meas_0.std():.4f}')
print(f'\ny[0] (滤波后 - 估计值) 统计:')
print(f'  最小值：{y_filt_0.min():.4f}')
print(f'  最大值：{y_filt_0.max():.4f}')
print(f'  平均值：{y_filt_0.mean():.4f}')
print(f'  标准差：{y_filt_0.std():.4f}')
print(f'\ny[1] (滤波前 - 测量값) 统计:')
print(f'  最小值：{y_meas_1.min():.4f}')
print(f'  最大值：{y_meas_1.max():.4f}')
print(f'  平均值：{y_meas_1.mean():.4f}')
print(f'  标准差：{y_meas_1.std():.4f}')
print(f'\ny[1] (滤波后 - 估计值) 统计:')
print(f'  最小值：{y_filt_1.min():.4f}')
print(f'  最大值：{y_filt_1.max():.4f}')
print(f'  平均值：{y_filt_1.mean():.4f}')
print(f'  标准差：{y_filt_1.std():.4f}')

# 创建 3 个子图
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
fig.suptitle('result', fontsize=16)

# ===== 子图 1: y[0] vs y[1] 相平面图 =====
ax1.plot(y_meas_0, y_meas_1, 'r-', linewidth=1.5, marker='o', markersize=2, alpha=0.4, label='before KF (noisy)')
ax1.plot(y_filt_0, y_filt_1, 'b-', linewidth=2, marker='s', markersize=3, alpha=0.7, label='after KF (filtered)')
ax1.set_xlabel('y[0] - pos x', fontsize=12)
ax1.set_ylabel('y[1] - pos y', fontsize=12)
ax1.set_title('traj: y[0] vs y[1] (KF comparison)', fontsize=14)
ax1.grid(True, alpha=0.3)
ax1.axis('equal')  # 保持 x、y 轴比例一致

# 标注起点和终点
ax1.plot(y_meas_0[0], y_meas_1[0], 'go', markersize=10, label='start meas')
ax1.plot(y_meas_0[-1], y_meas_1[-1], 'ro', markersize=10, label='end meas')
ax1.plot(y_filt_0[0], y_filt_1[0], 'bo', markersize=8, label='start filt')
ax1.plot(y_filt_0[-1], y_filt_1[-1], 'mo', markersize=8, label='end filt')
ax1.legend()

# ===== 子图 2: y[0] 和 y[1] 随时间变化曲线 =====
ax2.plot(time, y_meas_0, 'r--', linewidth=1.5, label='y[0] before KF (noisy)', alpha=0.6)
ax2.plot(time, y_filt_0, 'b-', linewidth=2, label='y[0] after KF (filtered)', alpha=0.8)
ax2.plot(time, y_meas_1, 'm--', linewidth=1.5, label='y[1] before KF (noisy)', alpha=0.6)
ax2.plot(time, y_filt_1, 'g-', linewidth=2, label='y[1] after KF (filtered)', alpha=0.8)
ax2.set_xlabel('time (s)', fontsize=12)
ax2.set_ylabel('output', fontsize=12)
ax2.set_title('output and time (KF comparison)', fontsize=14)
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