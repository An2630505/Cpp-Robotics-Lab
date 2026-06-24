# 动态障碍物避障

## 已有代码

### `dynamic_obstacles/` 模块

| 文件 | 内容 |
|------|------|
| `__init__.py` | 公开 API: `NpcVehicle`, `NpcManager` |
| `_core.py` | NPC 运动学、栅格注入 (`apply_to_grid`)、SAT 碰撞检测 (`check_collision_with_ego`) |

**NpcVehicle**: 沿中心线匀速行驶的 NPC。`update(t)` 更新位姿，`get_corners()` 返回 4 角点。

**NpcManager**: 批量管理 NPC，提供 `apply_to_grid(grid, grid_meta, dilation_m)` 将 NPC 膨胀后画入占用栅格（不调 `dilate_grid`，避免二次膨胀道路边界），`check_collision_with_ego()` 用 SAT 做 OBB 碰撞检测。

### 仿真脚本

| 文件 | 方案 | 状态 |
|------|------|------|
| `sim_dynamic_obstacles.py` | 方案 B — HA* 局部重规划 + Trajectory 替换 | ❌ 废弃 (MPC 不支持轨迹热切换) |
| `sim_dynamic_obstacles_animate.py` | 方案 B 的动画回放 | ❌ 随方案 B 废弃 |
| `sim_dynamic_obstacles_lateral.py` | 横向偏移 + 状态机决策 | ⚠️ 当前主力，有遗留问题 |

### `sim_dynamic_obstacles_lateral.py` 架构

```
map_parser → centerline → circuit → occupancy grid
    → HA* 全局规划 (一次性, 416m) → SafeCorridor → B-Spline
    → MPC + 状态机超车决策 → 可视化
```

状态机 (`IDLE → RAMPING_UP → HOLDING → RAMPING_DOWN → IDLE`):
- 决策层: NPC 进入 TRIGGER_DIST (50m) 时选方向、锁死、生成梯形 offset 曲线
- 执行层: MPC 跟踪 offset 目标, `_lateral_clearance` 每帧钳位

**参数**:

| 参数 | 值 | 含义 |
|------|-----|------|
| TRIGGER_DIST | 50m | NPC 进入此弧长距离触发超车 |
| RAMP_LENGTH | 30m | 偏移上升/下降的弧长距离 |
| MAX_OFFSET | 2.3m | 保持的横向偏移 |
| PASSED_DIST | 15m | NPC 超过自车此距离后开始归中 |

**NPC 速度观测**: 局部追踪 `find_centerline_s` (全局搜索 + ±5m 窗口), 两帧弧长差 / DT, EMA 平滑.

**弧长查找**: 已修复从全局最近邻到局部窗口追踪, 消除"幻视障碍物"问题.

---

## 当前存在的问题

### 1. 窄弯道超车失败 (核心)

自车 10m/s 追 NPC 5m/s, 在弯道 s≈185m 处需要 ≥2.5m 横向偏移才能安全通过, 但该处可用路宽仅约 3m, 即使方向正确也有碰撞风险。

**根因**: 纯横向避障, 无纵向控制 (速度固定 10m/s). 物理极限决定了某些弯道就是过不去.

### 2. 方向决策局部贪心

方向在触发点 (s≈108m) 选左, 但左前方 s≈196m 处路窄, 选了左边之后执行到窄处推超界.

**根因**: 决策只看了当前位置的路宽, 没扫描未来路径.

### 3. 路宽钳位导致 offset 突变

`_lateral_clearance` 探测 outer 多边形边界, 窄路段突然变窄导致 offset 目标跳变, MPC 的车辆动力学跟不上下游, 超界.

**根因**: 硬钳位 + MPC 响应时延.

### 4. `_lateral_clearance` 精度有限

步长 0.5m 探测 20m 深度, 宽路段始终返回 20m max, 两边相等时方向选择无意义.

---

## 可采取的方案

### A. 决策层前扫路宽 (推荐)

选方向时, 沿中心线前扫 RAMP_LENGTH + 最高持有距离 (≈80m), 取左右方向的最小路宽. 如果左边窄就用右边, 两边都窄就降 MAX_OFFSET.

```
if left_min_width < MAX_OFFSET and right_min_width >= MAX_OFFSET:
    pick right
elif both < MAX_OFFSET:
    MAX_OFFSET = max(left_min, right_min) * 0.8  # 自适应降低
```

### B. 纵横向联合避障

加入纵向控制——检测到前方窄弯道时减速, 让 NPC 先走, 到宽路段再加速超车. 需要改动 MPC 的速度控制或加一个外环.

### C. 回方案 B — HA* 重规划 + C++ MPC 修改

在 C++ `BicycleModel` 上增加 `set_state()` 方法 (只改状态, 不重置 KF 协方差), 使方案 B 的轨迹热切换可行. 这是真正的普适方案: HA* 看到的就是占用栅格里的障碍物, 不管是 NPC 还是静态障碍物、不管 NPC 做什么运动.

### D. 用 SafeCorridor 替代 `_lateral_clearance`

初始规划时 SafeCorridor 已算出沿途左右边界, 决策时直接查表, 比探测 outer 多边形更快更准. 但走廊不更新, 静态障碍物无法反映.

### E. 多传感器融合预测

取代简单的"匀速沿中心线"预测, 加入 UKF/EKF 对其他车辆的运动估计, 支持变道、加减速等非合作行为. 配合 HA* 重规划可实现真正的动态避障.

---

## 建议路线

1. **短期**: 方案 A (前扫路宽) + 方案 D (SafeCorridor), 快速修当前碰撞问题
2. **中期**: 方案 C (C++ MPC 改), 解锁方案 B, 获得普适的 HA* 动态重规划能力
3. **长期**: 方案 B + 方案 E (预测 + 重规划), 达到类 Apollo 的感知-决策-规划分层架构
