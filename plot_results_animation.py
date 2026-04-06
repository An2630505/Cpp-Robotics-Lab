import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# 设置交互式后端
import matplotlib
matplotlib.use('TkAgg')

# 设置字体
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial Unicode MS', 'Liberation Sans']
plt.rcParams['axes.unicode_minus'] = True

# 读取数据文件
data = []
with open('output/output.txt', 'r') as f:
    for line in f:
        if line.startswith('#') or line.startswith('Step') or not line.strip():
            continue
        
        parts = line.strip().split(', ')
        if len(parts) == 9:
            step = int(parts[0])
            # Plant KF 后验估计
            plant_y_0 = float(parts[1])
            plant_y_1 = float(parts[2])
            plant_y_2 = float(parts[3])
            plant_y_3 = float(parts[4])
            # Object KF 后验估计
            obj_y_0 = float(parts[5])
            obj_y_1 = float(parts[6])
            obj_y_2 = float(parts[7])
            obj_y_3 = float(parts[8])
            
            data.append([step, plant_y_0, plant_y_1, plant_y_2, plant_y_3, 
                        obj_y_0, obj_y_1, obj_y_2, obj_y_3])

data = np.array(data)

if len(data) == 0:
    print("错误: 数据文件为空或格式不正确!")
    exit(1)

steps = data[:, 0]
plant_y_0 = data[:, 1]
plant_y_1 = data[:, 2]
obj_y_0 = data[:, 5]
obj_y_1 = data[:, 6]

dt = 0.1
time = steps * dt

# 创建图形
fig, ax = plt.subplots(figsize=(10, 8))
fig.suptitle('Control Algorithm Results: Trajectory Animation (Plant vs Object)', fontsize=16)

# 初始化线条
line_plant, = ax.plot([], [], 'b-', linewidth=2, label='Plant Position (KF Estimated)')
line_obj, = ax.plot([], [], 'r--', linewidth=2, label='Object Position (KF Estimated)')
point_plant, = ax.plot([], [], 'bo', markersize=8)
point_obj, = ax.plot([], [], 'rs', markersize=8)

# 设置坐标轴范围 (可以根据实际数据动态调整，这里先预设一个大致范围或稍后自动调整)
# 为了动画流畅，通常先设定好固定的 axis limit，或者在 init 中设置
min_x = min(plant_y_0.min(), obj_y_0.min()) - 1
max_x = max(plant_y_0.max(), obj_y_0.max()) + 1
min_y = min(plant_y_1.min(), obj_y_1.min()) - 1
max_y = max(plant_y_1.max(), obj_y_1.max()) + 1

ax.set_xlim(min_x, max_x)
ax.set_ylim(min_y, max_y)
ax.set_xlabel('X Position', fontsize=12)
ax.set_ylabel('Y Position', fontsize=12)
ax.set_title('Trajectory: Plant vs Object', fontsize=14)
ax.grid(True, alpha=0.3)
ax.axis('equal')
ax.legend()

# 添加时间文本
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=12, verticalalignment='top')

def init():
    line_plant.set_data([], [])
    line_obj.set_data([], [])
    point_plant.set_data([], [])
    point_obj.set_data([], [])
    time_text.set_text('')
    return line_plant, line_obj, point_plant, point_obj, time_text

def update(frame):
    # frame 是当前帧的索引
    # 绘制从 0 到 frame 的数据
    current_plant_x = plant_y_0[:frame+1]
    current_plant_y = plant_y_1[:frame+1]
    current_obj_x = obj_y_0[:frame+1]
    current_obj_y = obj_y_1[:frame+1]
    
    line_plant.set_data(current_plant_x, current_plant_y)
    line_obj.set_data(current_obj_x, current_obj_y)
    
    # 更新当前点的位置
    if frame < len(plant_y_0):
        point_plant.set_data([plant_y_0[frame]], [plant_y_1[frame]])
        point_obj.set_data([obj_y_0[frame]], [obj_y_1[frame]])
    
    # 更新时间文本
    time_text.set_text(f'Time: {time[frame]:.2f} s')
    
    return line_plant, line_obj, point_plant, point_obj, time_text

# 创建动画
# interval: 帧间隔 (ms), blit: 是否只重绘变化的部分
ani = animation.FuncAnimation(fig, update, frames=len(steps), init_func=init,
                              interval=50, blit=True)

# 保存动画 (可选，需要 ffmpeg 或 imagemagick)
# ani.save('output/trajectory_animation.gif', writer='pillow', fps=20)
# ani.save('output/trajectory_animation.mp4', writer='ffmpeg', fps=20)

print('正在播放动画... 关闭窗口以退出。')
plt.show()
