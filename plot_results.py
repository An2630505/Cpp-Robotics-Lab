import matplotlib.pyplot as plt
import numpy as np

# 设置交互式后端（必须在导入 pyplot 之后立即设置）
import matplotlib
matplotlib.use('TkAgg')  # 使用 TkAgg 后端支持交互操作

# 设置字体（使用 DejaVu Sans,支持英文和符号）
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial Unicode MS', 'Liberation Sans']
plt.rcParams['axes.unicode_minus'] = True  # 正常显示负号

# 读取数据文件
data = []
with open('output/output.txt', 'r') as f:
    for line in f:
        # 跳过注释行、标题行和空行
        if line.startswith('#') or line.startswith('Step') or not line.strip():
            continue
        
        # 解析数据行：
        # main.cpp 输出格式: 
        # Step, plant.kf.y_post[0], plant.kf.y_post[1], plant.kf.y_post[2], plant.kf.y_post[3], 
        #       object.kf.y_post[0], object.kf.y_post[1], object.kf.y_post[2], object.kf.y_post[3]
        parts = line.strip().split(', ')
        if len(parts) == 9:
            step = int(parts[0])
            # Plant 卡尔曼滤波后验估计 (Plant KF Posterior Estimate)
            plant_y_0 = float(parts[1]) # x position
            plant_y_1 = float(parts[2]) # y position
            plant_y_2 = float(parts[3]) # x velocity
            plant_y_3 = float(parts[4]) # y velocity
            # Object 卡尔曼滤波后验估计 (Object KF Posterior Estimate)
            obj_y_0 = float(parts[5])   # x position
            obj_y_1 = float(parts[6])   # y position
            obj_y_2 = float(parts[7])   # x velocity
            obj_y_3 = float(parts[8])   # y velocity
            
            data.append([step, plant_y_0, plant_y_1, plant_y_2, plant_y_3, 
                        obj_y_0, obj_y_1, obj_y_2, obj_y_3])

# 转换为 numpy 数组
data = np.array(data)

if len(data) == 0:
    print("错误: 数据文件为空或格式不正确!")
    print("请检查 output/output.txt 文件格式")
    exit(1)

steps = data[:, 0]  # 时间步（step）
# Plant 状态估计 (Plant State Estimates via KF)
plant_y_0 = data[:, 1]     # x position
plant_y_1 = data[:, 2]     # y position
plant_y_2 = data[:, 3]     # x velocity
plant_y_3 = data[:, 4]     # y velocity
# Object 状态估计 (Object State Estimates via KF)
obj_y_0 = data[:, 5]       # x position
obj_y_1 = data[:, 6]       # y position
obj_y_2 = data[:, 7]       # x velocity
obj_y_3 = data[:, 8]       # y velocity

# 假设时间步长为 0.1s（根据 main.cpp 中的 dt）
dt = 0.1
time = steps * dt

# 打印统计信息
print('\n=== 数据统计 ===')
print(f'总步数：{len(steps)}')
print(f'时间范围：{time[0]:.2f}s ~ {time[-1]:.2f}s')
print(f'\nPlant 位置 X (Plant Pos X) 统计:')
print(f'  最小值：{plant_y_0.min():.4f}, 最大值：{plant_y_0.max():.4f}')
print(f'\nObject 位置 X (Object Pos X) 统计:')
print(f'  最小值：{obj_y_0.min():.4f}, 最大值：{obj_y_0.max():.4f}')

# 创建 3 个子图
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
fig.suptitle('Control Algorithm Results: Plant vs Object (KF Estimates)', fontsize=16)

# ===== 子图 1: 相平面图 (Position Trajectory) =====
ax1.plot(plant_y_0, plant_y_1, 'b-', linewidth=2, label='Plant Position (KF Estimated)', alpha=0.8)
ax1.plot(obj_y_0, obj_y_1, 'r--', linewidth=2, label='Object Position (KF Estimated)', alpha=0.8)
ax1.set_xlabel('X Position', fontsize=12)
ax1.set_ylabel('Y Position', fontsize=12)
ax1.set_title('Trajectory: Plant vs Object', fontsize=14)
ax1.grid(True, alpha=0.3)
ax1.axis('equal')  # 保持 x、y 轴比例一致
ax1.legend()

# 标注起点
ax1.plot(plant_y_0[0], plant_y_1[0], 'bo', markersize=8, label='Plant Start')
ax1.plot(obj_y_0[0], obj_y_1[0], 'rs', markersize=8, label='Object Start')

# ===== 子图 2: 位置随时间变化 (Position vs Time) =====
ax2.plot(time, plant_y_0, 'b-', linewidth=1.5, label='Plant X', alpha=0.8)
ax2.plot(time, obj_y_0, 'r--', linewidth=1.5, label='Object X', alpha=0.8)
ax2.plot(time, plant_y_1, 'g-', linewidth=1.5, label='Plant Y', alpha=0.8)
ax2.plot(time, obj_y_1, 'm--', linewidth=1.5, label='Object Y', alpha=0.8)
ax2.set_xlabel('Time (s)', fontsize=12)
ax2.set_ylabel('Position', fontsize=12)
ax2.set_title('Position Components vs Time', fontsize=14)
ax2.grid(True, alpha=0.3)
ax2.legend(loc='best')
ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.3)

# ===== 子图 3: 速度随时间变化 (Velocity vs Time) =====
ax3.plot(time, plant_y_2, 'b-', linewidth=1.5, label='Plant Vel X', alpha=0.8)
ax3.plot(time, obj_y_2, 'r--', linewidth=1.5, label='Object Vel X', alpha=0.8)
ax3.plot(time, plant_y_3, 'g-', linewidth=1.5, label='Plant Vel Y', alpha=0.8)
ax3.plot(time, obj_y_3, 'm--', linewidth=1.5, label='Object Vel Y', alpha=0.8)
ax3.set_xlabel('Time (s)', fontsize=12)
ax3.set_ylabel('Velocity', fontsize=12)
ax3.set_title('Velocity Components vs Time', fontsize=14)
ax3.grid(True, alpha=0.3)
ax3.legend(loc='best')
ax3.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.3)

# 调整布局
plt.tight_layout()

# 保存图片
plt.savefig('output/simulation_results.png', dpi=300, bbox_inches='tight')
print('图表已保存为 simulation_results.png')

# 显示交互式图表（支持缩放、平移等操作）
print('\n提示：在弹出的窗口中可以使用以下交互功能：')
print('  - 鼠标滚轮：缩放')
print('  - 拖拽：平移')
print('  - 工具栏按钮：重置视图、前进/后退、保存等')
plt.show()
