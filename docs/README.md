# Cpp-Robotics-Lab

C++ 自动驾驶算法库 + Python 仿真管线。

## 架构

```
┌─────────────────────────────────────────────────────────────────┐
│                     Python Pipeline (pipeline/)                 │
│  map_parser → centerline → HA* → SafeCorridor → BSpline → MPC  │
│                         │               │            │         │
│                    调用 C++ 算法    调用 C++ 算法   调用 C++    │
└─────────────────────────┼───────────────┼────────────┼─────────┘
                          │               │            │
┌─────────────────────────┼───────────────┼────────────┼─────────┐
│                     C++ 算法库 (pnc/)                           │
│  astar  hybrid_astar  safe_corridor  bspline                   │
│  path   bicycle_model  mpc_planner    map_parser               │
│  ───────────────────────────────────────────                    │
│  control:  mpc  kf  lqr  pid                                   │
└─────────────────────────────────────────────────────────────────┘
```

## 仿真管线

| 步骤 | 模块 | 说明 |
|------|------|------|
| 1 | map_parser | 解析 path2.png → 提取外边界多边形 + 孔洞 |
| 2 | centerline | 边界骨架化 → 提取拓扑图 → 组装闭环中心线 |
| 3 | static_obstacles | 加载障碍物配置 (圆/矩形/多边形) → 生成障碍物多边形 |
| 4 | build_occupancy_grid | 扫描线填充 → 障碍物标记 → 膨胀 → 占用栅格 (0=自由, 1=障碍物) |
| 5 | Hybrid A\* | 逐 Gate 规划粗略轨迹, 栅格碰撞检测保证无碰 |
| 6 | SafeCorridor | 在 HA* 采样点上扩张矩形, 逐 cell 检查栅格 → 安全走廊截面 |
| 7 | BSpline | 以安全走廊为约束, 拟合平滑轨迹 → 等弧长重采样 |
| 8 | MPC + KF | 模型预测控制 + 卡尔曼滤波 + 前馈 → 仿真跟踪 |

## SafeCorridor — 安全走廊构建

在每个 HA* 轨迹采样点上向左右法向**扩张矩形**, 检查矩形内全部 cell：

```
         n_left (法向)
          ↑
 step 3:  ┌──────────────────┐  depth = 3×cell
 step 2:  │  ┌──────────────┐│  
 step 1:  │  │  ┌──────────┐││  
          │  │  │  车辆    │││  ← 矩形宽 = 2×COLLISION_MARGIN
          │  │  └──────────┘││
          │  └──────────────┘│
          └──────────────────┘
          
   逐层检查: 计算矩形包围盒 → 遍历每个 cell → 点积判定是否在矩形内
   任一 occupied → 返回上一层距离 → 减 CORRIDOR_MARGIN → 记录截面
```

### 安全边距拆分

同一安全约束拆为三个独立参数, 不再互相抵消：

| 参数 | 用途 | 值 |
|------|------|-----|
| SAFETY_MARGIN | 栅格膨胀 (障碍物向外扩) | VEHICLE_HW + 0.2 |
| COLLISION_MARGIN | HA\* 碰撞盒半宽 = 走廊矩形半宽 | VEHICLE_HW + 0.2 |
| CORRIDOR_MARGIN | 走廊边界缩进 (膨胀后的路再往里收) | COLLISION_MARGIN |

```
真实障碍物 ─── SAFETY_MARGIN(膨胀) ─── dilated cell
                    ← COLLISION_MARGIN →  HA* 车盒 + 走廊矩形
                    ← CORRIDOR_MARGIN →  走廊边界
```

每个截面 `CorridorSection` 记录 `center`(采样点)、`left`(左边界)、`right`(右边界)。截面连成安全走廊管道, 作为 B 样条拟合的**硬约束**。

## C++ 算法模块 (pnc/)

| 模块 | 文件 | 功能 |
|------|------|------|
| A\* | `motion/astar/` | A\* 栅格路径规划 |
| Hybrid A\* | `motion/hybrid_astar/` | 车体运动学约束的 A\* |
| SafeCorridor | `motion/safe_corridor/` | 矩形扩张构建安全走廊 |
| BSpline | `motion/bspline/` | B 样条拟合与重采样 |
| BicycleModel | `motion/bicycle_model/` | 自行车动力学模型 |
| MapParser | `motion/map_parser/` | 地图解析与栅格构建 |
| MPC | `control/mpc/` | 模型预测控制 |
| KF | `control/kf/` | 卡尔曼滤波器 |
| LQR | `control/lqr/` | 线性二次调节器 |
| PID | `control/pid/` | PID 控制器 |

## Python Pipeline (pipeline/)

| 脚本 | 功能 |
|------|------|
| `sim_trajectory_optimization.py` | 轨迹优化: HA\* + SafeCorridor + BSpline + MPC |
| `sim_static_obstacles.py` | 静态障碍物场景: 同上 + 障碍物避让 |
| `sim_lane_keeping_real.py` | 车道保持: 闭环回路 + MPC 循迹 |
| `sim_navigation.py` | A\*/HA\* 路径规划导航 |
| `sim_mpc_basic.py` | 基础 MPC 仿真 |
| `map_parser/` | 地图解析 (PNG → 多边形边界) |
| `centerline/` | 中心线提取 (骨架化 → 拓扑图 → 回路) |
| `static_obstacles/` | 障碍物定义与加载 |

## 编译与运行

```bash
# 编译 C++ 模块
cd build && cmake .. && cmake --build .

# 运行仿真 (需 conda 环境 CRL, Python 3.11)
conda activate CRL
python pipeline/sim_trajectory_optimization.py
python pipeline/sim_static_obstacles.py

# 运行 C++ 单元测试
./build/pnc/test_safe_corridor
./build/pnc/test_bspline
./build/pnc/test_hybrid_astar
```

## 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| CELL_SIZE | 0.2m | 栅格分辨率 |
| VEHICLE_HW | 1.0m | 车辆半宽 (全宽 2.0m) |
| SAFETY_MARGIN | VEHICLE_HW+0.2=1.2m | 栅格膨胀距离 |
| COLLISION_MARGIN | VEHICLE_HW+0.2=1.2m | HA\* 碰撞盒半宽 / 走廊矩形半宽 |
| CORRIDOR_MARGIN | COLLISION_MARGIN=1.2m | 走廊边界缩进 |
| GATE_SPACING | 15m | Gate 间距 |
| SAMPLE_INTERVAL | 2m | 走廊采样间距 |
| BSPLINE_DEGREE | 3 | B 样条阶数 |
| VX | 10 m/s | 恒定巡航速度 |
| DT | 0.1s | 仿真步长 |
| N_HORIZON | 40 | MPC 预测步数 |
