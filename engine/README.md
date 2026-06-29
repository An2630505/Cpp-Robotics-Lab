# Engine — 2D 仿真引擎

## 设计思想

引擎是一个**纯物理世界模拟器**——它只忠实地执行物理规律，不做任何决策。

### 核心理念：碰撞 ≠ 行驶

```
┌──────────────────────────────────────────┐
│  碰撞 (物理定律)                            │
│  - 动量守恒, 弹性碰撞                       │
│  - 只修改速度 (vx, vy), 不动 heading        │
│  - 所有动态实体均遵循                        │
├──────────────────────────────────────────┤
│  行驶 (控制模型)                            │
│  - Agent 下发控制指令 (steer, ax)           │
│  - 运动模型将指令转为速度变化                 │
│  - BicycleModel: 只影响纵向, 保留侧向速度     │
│  - SimpleModel: 直接叠加加速度               │
└──────────────────────────────────────────┘
```

碰撞后产生的侧向速度会被保留——这是物理定律。Agent 通过控制指令逐步将车辆"拉回"正常行驶状态。

### 架构

```
engine/
├── physics/       C++ 物理层 (pybind11 模块: engine_physics)
│   ├── types.h        基础类型 (Vec2d, Pose, Polygon, EntityState…)
│   ├── motion_model   运动模型 (BicycleModel, SimpleModel)
│   ├── collision      SAT 碰撞检测 + 弹性碰撞响应
│   └── physics_world  物理世界: 实体管理 + step()
├── execution/     Python 执行层
│   ├── world.py       World: 实体管理, 控制下发, 步进调度
│   └── agent.py       Agent 基类: init() + tick(percepts) → ControlInput
└── perception/    Python 感知层 (v0: 真值透传)
    └── sensor.py      Sensor + Percepts
```

### 三层架构

```
Pipeline (集成层)
  │
  ├─→ perception/  感知层 — 给 Agent 提供世界信息 (v0: 真值透传)
  ├─→ execution/   执行层 — 主循环, Agent 管理, 控制下发
  └─→ physics/     物理层 — 运动积分, 碰撞检测/响应
```

## 使用方法

### 编译

```bash
cmake --build build  # engine_physics 自动编译到 build/engine/physics/
```

### 快速开始

```python
import sys
sys.path.insert(0, "build/engine/physics")
sys.path.insert(0, ".")
import engine_physics as ep
from engine import World, Agent, Sensor

# 1. 创建世界
world = World(dt=0.01)

# 2. 添加静态障碍物 (围墙)
wall = ep.EntityState()
wall.pose = ep.Pose(0, 15, 0)
wall.geometry = ep.Polygon.aabb(20, 0.5)
wall.is_static = True
world.add_entity(wall, None)

# 3. 添加车辆
ego = ep.EntityState()
ego.pose = ep.Pose(0, 0, 0)
ego.vel = ep.Velocity(10, 0, 0)
ego.geometry = ep.Polygon.vehicle(half_width=1.0, forward=1.5, backward=1.0)
ego_id = world.add_entity(ego, ep.BicycleModel(wheelbase=2.68))

# 4. 实现 Agent
class MyAgent(Agent):
    def init(self, world): pass
    def tick(self, percepts):
        return ep.ControlInput(steer=0.05, ax=0.0)  # 恒定右转

# 5. 仿真循环
sensor = Sensor()
agent = MyAgent(ego_id)
agent.init(world)
world.register_agent(agent)
world.start()

while world.running:
    percepts = sensor.get_percepts(world, ego_id)
    cmd = agent.tick(percepts)
    world.apply_control(ego_id, cmd)
    collisions = world.step()
    for c in collisions:
        print(f"碰撞: {c.entity_a} ↔ {c.entity_b} pen={c.result.penetration:.3f}m")
```

### 关键类型

| 类型 | 说明 |
|------|------|
| `World(dt)` | 仿真世界, 封装 PhysicsWorld + Agent 管理 |
| `Agent` | 智能体基类, 需实现 `init()` + `tick()` |
| `EntityState` | 实体状态: pose, vel, geometry, mass, is_static |
| `Polygon.vehicle(hw, fwd, rev)` | 车辆矩形 (前/后/半宽) |
| `Polygon.aabb(hw, hh)` | 轴对齐矩形 |
| `BicycleModel(L)` | 自行车模型: steer→ω=v·tan(steer)/L |
| `SimpleModel()` | 简单模型: steer→ω, ax→沿车头加速 |
| `ControlInput(steer, ax)` | 控制指令 |
| `Sensor.get_percepts(world, id)` | 感知透传 → `Percepts` |

### 运动模型

```
BicycleModel — 车辆
  控制: steer → omega = v_lon * tan(steer) / L
        ax    → v_lon += ax * dt
  侧向速度保留 (碰撞后侧滑不丢失)

SimpleModel — 简单实体
  控制: steer → 直接作为 omega
        ax    → 沿车头方向加速
  侧向速度保留
```

### 碰撞检测与响应

- **SAT** (Separating Axis Theorem): 支持任意凸多边形
- **弹性碰撞**: 完全弹性 (e=1), 动量守恒, 无摩擦
- **质量**: 自动从 `geometry.area()` 计算 (密度=1)
- **静态实体**: 质量视为 ∞, 撞击后对方反弹
- **位置修正**: 按质量反比分离穿透

## 运行 Demo

```bash
python pipeline/sim_engine_demo.py                    # 台球竞技场实时动画
python pipeline/sim_engine_demo.py --save demo.gif     # 保存为 GIF
```
