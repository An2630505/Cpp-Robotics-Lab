# 轨迹优化

## 概述

轨迹优化在赛道**安全约束**下生成平滑的最短可行驶路径，替代简单跟随中心线的策略。核心思路：以中心线作全局引导，通过 Hybrid A\* 在赛道边界内逐段搜索最短路径，再经安全走廊约束的 B 样条平滑，最终由 MPC 跟踪。

## 管线架构

```
path2.png
  │
  ├─[1] map_parser ──────► 外边界 + 孔洞多边形 + 起跑线
  │
  ├─[2] centerline ──────► 中心线骨架图
  │
  ├─[3] assemble_circuit ► 闭环中心线回路 (447m, 5658 pts)
  │
  ├─[4] build_occupancy ─► 扫描线填充 → dilate_grid(6 cells=1.2m) → 占用栅格
  │                          411×411 cells, cell=0.2m, ≈82×82m
  ├─[5] Hybrid A* ───────► 沿中心线 15m 间距生成 29 道 Gate (宽 3.5m)
  │                         gates.append(gates[0]) → 30 段闭合
  │                         逐段 planToGate，输出 ≈720 点锯齿路径
  ├─[6] SafeCorridor ────► 沿 HA* 路径每 2m 采样
  │                         基于栅格逐 cell 扫描左右可通行宽度
  │                         margin=1.2m (车辆半宽+0.2m 余量)
  │                         输出 ≈220 组自由空间截面
  ├─[6] BSpline ─────────► clamped 3 次开放 B 样条, 50 控制点
  │                         最小二乘拟合 + 2 轮走廊软约束投影
  │                         等弧长 0.5m 重采样 → 输出平滑轨迹
  │
  ├─[7] MPC ─────────────► 4 状态误差动力学 (e_y, de_y, e_psi, de_psi)
                             Kalman Filter 状态估计 + 曲率前馈
                             10m/s 匀速，DT=0.1s，N=40 预测时域
                             输出仿真日志 + 可视化
```

## 核心参数

### 车辆物理参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 质量 | 1573 kg | |
| 转动惯量 | 2873 kg·m² | |
| 前轴到质心 | 1.1 m | |
| 后轴到质心 | 1.58 m | |
| 轴距 | 2.68 m | 转向半径 = wheelbase / tan(δ) |
| 前后轮侧偏刚度 | 80000 N/rad | |
| 最大转向角 | ±30° | |

### Hybrid A\* 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 栅格分辨率 | 0.2 m/cell | `CELL_SIZE` |
| 安全边距 | 1.2 m (6 cells) | `SAFETY_MARGIN = VEHICLE_HW + 0.2`，栅格膨胀距离 |
| 车辆半宽 | 1.0 m | `VEHICLE_HW`，碰撞检测用 |
| 车辆前方延伸 | 1.5 m | `VEHICLE_FWD`，后轴到车头 |
| 车辆后方延伸 | 1.0 m | `VEHICLE_REV`，后轴到车尾 |
| HA\* 碰撞盒 | (1.2, 1.7, 1.2) | `(COLLISION_MARGIN, VEHICLE_FWD+0.2, VEHICLE_REV+0.2)` |
| 单步弧长 | 0.6 m | 运动学扩展步长 |
| 转向角数 | 5 | `[-0.6, -0.3, 0, 0.3, 0.6]` rad |
| Gate 间距 | 15 m | 沿中心线等弧长采样 |
| Gate 宽度 | 3.5 m | 覆盖赛道宽度 |
| 目标位置容差 | 1.0 m | planToGate 终止条件 |
| 目标朝向容差 | 1.0 rad | planToGate 终止条件 |
| 回退容差 | 3.0 m | 连续失败时的放宽容差 |

### Safe Corridor 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 采样间隔 | 2.0 m | 沿 HA* 路径弧长采样 |
| 边界余量 | 1.2 m | `CORRIDOR_MARGIN = COLLISION_MARGIN` |
| 构建方式 | 栅格扫描 | 沿法向逐 cell 检测障碍物，到占用格即停 |

### B-Spline 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 阶数 | 3 (cubic) | `BSPLINE_DEGREE` |
| 控制点数 | 50 | `BSPLINE_NUM_CTRL` |
| 模式 | `closed=False` | clamped 开放曲线 |
| 重采样间距 | 0.5 m | `BSPLINE_RESAMPLE` |
| 走廊投影迭代 | 2 轮 | 超出边界→投影→重新拟合 |

### MPC 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 预测时域 | N=40 | 闭式无约束 QP (Cholesky 求解) |
| 仿真步长 | 0.1 s | DT |
| 目标速度 | 10 m/s | 匀速 |
| 离散化 | 前向欧拉 | `A_d = I + A_c·DT` |
| Q 权重 | diag(80, 0.5, 15, 0.5) | (e_y, de_y, e_psi, de_psi) |
| R 权重 | [0.1] | 转向角惩罚 |
| KF Q | 0.01·I₄ | 过程噪声 |
| KF R | diag(0.1, 0.1, 0.025, 0.005) | 测量噪声 |
| KF 初始 P | 1.0·I₄ | 初始协方差 |
| 初始 e_y | -0.3 m | 初始横向偏移 |
| 初始 e_psi | 0.05 rad | 初始航向偏差 |

## C++ 算法模块

所有算法实现在 `pnc/motion/` 下，通过 pybind11 暴露为 `pnc` Python 模块。

### Hybrid A\* (`pnc/motion/hybrid_astar/`)

**输入**：占用栅格 `grid`、起点 `Pose`、终点 Gate（两个 `Vec2d` 构成的线段）

**算法**：
1. **Dijkstra 启发式** — 从终点 Gate 扫过的所有栅格同时扩散到全图，构建 8-邻域距离场
2. **运动学 A\* 搜索** — 5 种转向角 `[-0.6, -0.3, 0, 0.3, 0.6]` rad，步长 0.6m
3. **车辆模型** — `R = wheelbase / tan(δ)` 精确圆弧积分
4. **终止条件** — 到 Gate 线段距离 < 1m 且朝向差 < 1 rad
5. **碰撞检测** — 车辆矩形 bounding box 与占用栅格逐格碰撞测试
6. **代价函数** — `arc_length + |δ| × arc_length × 0.3`（惩罚大转角）
7. **状态去重** — `(x/bin, y/bin, θ/bin)` 三维哈希

**关键函数**：
- `plan(Pose start, Pose goal)` — 点到点规划（首段用）
- `planToGate(Pose start, Vec2d a, Vec2d b)` — 点到线段规划（后续段用）
- `set_vehicle_dims(hw, fwd, rev)` — 设置车辆碰撞盒尺寸
- `set_grid_origin(x_min, y_min)` — 设置栅格世界坐标原点

### Safe Corridor (`pnc/motion/safe_corridor/`)

**输入**：参考路径 `vector<Pose>`、占用栅格 `grid`、栅格元信息 `(x_min, y_min, cell_size, cols, rows)`

**构建方式（栅格扫描）**：
1. 沿参考路径等弧长采样（间隔 2m）
2. 在每个采样点计算路径切线，沿左右法向逐 cell 扫描
3. 碰到占用格 (cell==1) 或栅格边界即停，取可通行 cell 数 × cell_size 为边界距离
4. 减去 margin (1.2m) 得到最终可行宽度
5. 输出 `vector<CorridorSection>`，包含中心点、左边界点、右边界点

> **与多边形求交方式的区别**：栅格扫描直接利用已膨胀的占用栅格，避免了射线-多边形求交的复杂性和孔洞处理的边界情况。

### B-Spline (`pnc/motion/bspline/`)

**输入**：参考路径（Hybrid A\* 输出）、安全走廊约束 `vector<CorridorSection>`

**算法**：

1. **等弧长采样** — 沿参考路径采样 N = max(n_ctrl, n_orig) 个数据点，覆盖 [0, total_len] 完整弧长范围
2. **构建 Knot Vector** — closed=false 时使用 clamped：`{0,0,0,0, u₁, ..., u₄₆, 1,1,1,1}`；closed=true 时使用 uniform periodic
3. **设计矩阵** — `B[i,j] = basis(j, degree, param_i, knots)`，Cox-de Boor 递推求值
4. **最小二乘** — `P = (B^T B + εI)^{-1} B^T D`，ε=1e-6 正则化防奇异
5. **走廊约束投影**（2 轮迭代）：
   - 密集评估 B 样条（≥200 点），检测超出走廊边界的点
   - 超出点沿中心-边界方向投影到走廊边界
   - 重新拟合，首尾样本固定为原始 ref_path 端点（防漂移）
6. **等弧长重采样** — 以 fixed spacing 沿 B 样条采样，二分查找 arc→param 映射

**关键设计**：
- **Clamped 右端点**：`basis()` 在 k=0 时修正了 clamped knot vector 右端点求值 —— 按「最后一个非退化 knot 跨的右边界」判定，而非硬编码索引
- **端点防漂移**：走廊投影迭代的重新拟合中，`new_samples[0]` 和 `new_samples[n_s-1]` 固定取自原始 ref_path 首尾点，不取投影点

## 管线详情

### Step 1: 地图解析 (`map_parser`)

```
path2.png → 灰度化 → Otsu 二值化 → RETR_CCOMP 轮廓提取
    → 世界坐标转换 (12.8 px/m) → cubic periodic spline 平滑 → dict
```

输出：`{"outer_boundary": [[x,y],...], "holes": [[[x,y],...],...], "starting_line": ((x1,y1),(x2,y2))}`

### Step 2: 中心线提取 (`centerline`)

```
边界掩码 → skimage.skeletonize → 交汇点检测 → 毛刺剪除
    → KDTree 聚类 → Union-Find 合并 → 边追踪 → 样条平滑 → graph
```

### Step 3: 闭环回路拼装 (`assemble_go_straight_circuit`)

从 centerline 图自动拼接连通环路，从 `start_node_id` 出发贪心走一圈。支持 3 岔路口（环岛入口）、4+ 路口（直行选择）、双向通行。

### Step 4: 占用栅格构建

```
外边界 + 孔洞多边形 → 扫描线填充 (奇偶规则)
    → 外边界内填 1，孔洞内挖 0
    → 翻转 (0=自由, 1=障碍物)
    → pnc.dilate_grid(grid, radius=6) → 膨胀 1.2m
```

### Step 5: Hybrid A\* Gate 规划

1. 沿中心线 15m 间距生成 29 道 Gate（横跨赛道 3.5m 宽）
2. `gates.append(gates[0])` → 30 段构成闭合圈
3. 第一段：`ha.plan(start_pose, gate[0].mid)` — 点到门中点
4. 后续段：`ha.plan_to_gate(current, gate[i].a, gate[i].b)` — 点到门线段
5. 段间去重拼接（距离 < 0.1m 视为重复点）
6. 失败回退：连续 3 段失败则停止；单段失败放宽 tol 到 3.0m 重试

**起点朝向**：从 gate[0] 的左右端点推导切线方向 —— `θ = atan2(-Δx, Δy)`，而非硬编码 0。

### Step 6: Safe Corridor + B-Spline

```
HA* raw path → SafeCorridor.build(grid, x_min, y_min, cell_size, cols, rows)
    → BSpline.fit(ref_path, corridors) [clamped, closed=False]
    → BSpline.resample(fitted) [0.5m spacing]
    → 平滑轨迹
```

### Step 7: MPC 仿真

4 状态误差动力学模型 (e_y, de_y, e_psi, de_psi)，自行车模型离散化，Kalman Filter 状态估计，曲率前馈补偿（运动学 + 动力学项），闭式无约束 QP 求解。

## 关键设计决策

### B 样条模式：Clamped 开放曲线

轨迹起止于同一道起跑线 Gate，但不要求曲线数学上闭合。使用 `closed=False` + clamped knot vector，样条自然起始于起点、终止于终点，首尾由最小二乘自然锚定。

对比闭合模式 (`closed=True`)：后置控制点平均使首尾 `degree` 个控制点被强制取均值，控制多边形产生折角，传播为轨迹震荡。

### Cox-de Boor 基函数右端点修复

**Bug**：`basis(k=0)` 对 clamped knot vector 右端点使用了错误索引 `knots.size()-2`，而非最后一个有效控制点索引。导致样条在 t=1 处求值为 (0,0)，终点偏离 40m。

**修复**：按「最后一个非退化 knot 跨的右边界」判定 —— `knots[i] < knots[i+1]` 确保只匹配非退化区间。

### 走廊投影迭代端点防漂移

走廊约束投影的重新拟合若全部使用投影点作为样本，端点可能被逐轮漂移。修复：`new_samples[0]` 和 `new_samples[n_s-1]` 固定取自原始 `ref_path` 首尾点，不取投影点。

### Gate 分段规划

闭环赛道 start ≈ goal 无法直接使用 Hybrid A\* 点到点规划。解决：沿中心线 15m 间距生成 Gate 线段，将一圈拆分为 30 段，逐段 `planToGate`（终点为线段而非单点）。首段用 `plan` 到门中点，末段回到起点门线段。

### 中心线只作引导

中心线用于生成 Gate 位置，但 Hybrid A\* 在 Gate 线段范围内可自由选择通过点，不受中心线约束。

## 运行

```bash
# conda 环境 (Python 3.11)
conda activate CRL

# 编译 C++ 模块
./build_pnc.sh config && ./build_pnc.sh

# 运行轨迹优化 + MPC 仿真
python pipeline/sim_trajectory_optimization.py

# 播放动画
python pipeline/sim_trajectory_optimization_animate.py
```

## 输出

| 文件 | 说明 |
|------|------|
| `output/sim_trajectory_optimization.txt` | MPC 仿真日志 (step, t, e_y, de_y, e_psi, de_psi, steer) |
| `output/sim_trajectory_optimization.png` | 可视化 (轨迹鸟瞰 + 曲率 + 误差 + 转向角) |
| `output/sim_trajectory_optimization_traj.npy` | 优化后轨迹点 (N×2) |
| `output/sim_trajectory_optimization_outer.npy` | 外边界多边形 |
| `output/sim_trajectory_optimization_hole_{i}.npy` | 孔洞边界多边形 |
| `output/sim_trajectory_optimization_corridors.npy` | 安全走廊截面 (N×3×2: left, center, right) |

## 可视化说明

`visualize()` 函数生成 2×3 布局的 6 面板图：

| 面板 | 内容 |
|------|------|
| 主图 (1,4) | 赛道边界、安全走廊填充、HA\* 路径、B-Spline 路径、车辆轨迹、起点/终点标记、朝向箭头 |
| 曲率 (2) | 参考曲率 + 前馈转向角时间序列 |
| 横向误差 (3) | e_y 时间序列 |
| 航向误差 (5) | e_psi 时间序列 |
| 转向角 (6) | 实际转向角时间序列 |

安全走廊在主图中以青色半透明填充和截面线显示，受 outer 边界 clip path 约束，不超出赛道。孔洞在走廊之后绘制以覆盖侵入孔洞的走廊区域。

## 已知问题

1. **MPC 参数敏感性** — kappa 平滑力度（`uniform_filter1d(size=21, mode='wrap')`）对 MPC 稳定性影响显著。size < 9 可能发散，size 过大弱化转向响应。

2. **B 样条端点精度** — clamped 样条末端 knot 间距仅 1/47≈0.021，端点切线对控制点位置极度敏感。当前方案通过走廊迭代端点固定来缓解，极端窄弯处仍可能出现末端不平滑。

3. **Python 版本** — C++ `.so` 编译时链接 Python 3.11（conda CRL 环境）。运行必须用对应 Python，否则 `import pnc` 失败。
