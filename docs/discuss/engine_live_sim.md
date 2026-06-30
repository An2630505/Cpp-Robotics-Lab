# 引擎实时仿真 — 动态避障 (LiveSim)

## 概述

`sim_engine_dynamic_obstacles_live.py` 是引擎版动态避障的**实时可视化**实现。所有实体（ego、NPC、赛道边界）均在引擎中统一管理，引擎边计算边渲染，非离线回灌。

## 架构

```
LiveSim
  │
  ├─ setup()
  │   ├─ 地图解析 → 中心线 → HA* → SafeCorridor → BSpline
  │   ├─ 引擎 World: ego(BicycleModel) + NPCs(SimpleModel) + 536段墙体
  │   ├─ MPC + KF 初始化
  │   └─ matplotlib 画布 + artists
  │
  └─ run()  ← 单层循环, 实时渲染
      for each MPC step (10Hz):
        ├─ 读引擎 ego 世界位姿 → 计算 e_y, e_psi
        ├─ 状态机 (超车决策)
        ├─ KF + MPC → steer
        ├─ 速度控制器 → ax
        ├─ for 10 引擎子步 (100Hz):
        │    ├─ ego + NPC 控制下发
        │    ├─ World.step() → 物理 + 碰撞检测 + 弹性响应
        │    └─ 每3步: _render() 刷新画面
        └─ 记录历史
```

## 关键设计决策

### 1. 所有实体进引擎

ego 不再由 pnc.BicycleModel 独立推进，而是在引擎中用 `BicycleModel(L_WB)` 驱动。MPC 输出的 steer 直接下发到引擎，引擎负责积分和碰撞响应。

### 2. 侧向阻尼 (Lat Damping)

碰撞后引擎 BicycleModel 保留侧向速度（动量守恒），但加入轮胎侧向阻尼模拟侧偏刚度：

```
v_lat *= max(0, 1 - lat_damping * dt)   // lat_damping=15, dt=0.01
```

碰撞侧滑 ~0.15s 衰减到零，模拟真实轮胎抓地力。无阻尼则侧滑永不消失（冰面）。

参见: [engine/physics/motion_model.h](../engine/physics/motion_model.h) `BicycleModel::lat_damping_`

### 3. 巡航速度控制

MPC 只输出 steer，不输出油门。引擎 BicycleModel 的 `ax` 由独立速度控制器提供：

```python
ax = (VX - cur_spd) * 3.0   # P 控制维持 10m/s 巡航
```

### 4. 碰撞后行为

碰撞后 ego 被弹性反弹改变速度方向。MPC 尝试用 steer 纠偏，但 ego 只会前进不会倒车。若被弹到墙上可能卡住——这是符合物理预期的结果。

### 5. 实时渲染

- `plt.ion()` + `plt.show(block=False)` 非阻塞窗口
- `fig.canvas.draw()` + `plt.pause(0.001)` 每帧刷新
- 信息文本直接从引擎读取最新状态（非 MPC 步进前的旧值）
- 空格暂停，关闭窗口退出
- 默认跟随 ego 视角（30m 范围）

## 与离线版的对比

| | 离线版 (`sim_engine_dynamic_obstacles.py`) | 实时版 (`_live.py`) |
|---|---|---|
| 仿真方式 | 所有步进完再画图 | 边步进边画 |
| 数据流 | 写 .npy → 动画脚本读取 | 引擎 `get_entity_state()` 直接读 |
| 碰撞特效 | 无 | 碰撞时终端打印 |
| 交互 | 无 | 空格暂停、关窗退出 |
| 渲染帧率 | 一次性生成 PNG/GIF | ~30 FPS 实时 |

## 用法

```bash
python pipeline/sim_engine_dynamic_obstacles_live.py             # 实时窗口, 跑一圈
python pipeline/sim_engine_dynamic_obstacles_live.py --time 20   # 跑20秒
```

## 相关文件

- 引擎核心: `engine/physics/` (C++), `engine/execution/`, `engine/perception/` (Python)
- 离线仿真: `pipeline/sim_engine_dynamic_obstacles.py`
- 离线动画: `pipeline/sim_engine_dynamic_obstacles_animate.py`
- 引擎设计: `docs/discuss/engine_design.md`
