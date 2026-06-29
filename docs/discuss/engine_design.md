# 2D 仿真引擎设计文档 (v0.1.0)

## 动机

Cpp-Robotics-Lab 之前主要做静态路径规划, 仿真脚本各自内联轨迹类、NPC 运动学、碰撞检测等逻辑, 缺乏统一的世界模型和时间推进机制。现需支持实时动态避障与碰撞检测, 需要一个独立的 2D 仿真引擎。

## 核心设计原则

1. **引擎独立**: `engine/` 不依赖项目任何其他模块 (`pnc/`, `pipeline/`), 只有 `pipeline/` 可以集成引擎
2. **物理规律模拟器**: 引擎忠实地执行物理规律 (运动积分、碰撞检测/响应), 不做决策
3. **统一对待实体**: Ego 和 NPC 在引擎眼里没有区别——都是带几何外形 + 运动模型的实体, 仅控制来源不同
4. **固定步长**: dt = 10ms, 同步更新所有实体
5. **C++ 算法 / Python 框架**: 物理层用 C++ 保证性能, 执行层和感知层用 Python 保证灵活性

## 三层架构

```
┌─────────────────────────────────────┐
│  Pipeline (集成层)                    │
│  ├─ World 构建 + 实体添加             │
│  ├─ Agent 初始化 (各自独立感知/规划)    │
│  └─ 主循环 (每 dt)                     │
│      ├─ Sensor 获取世界真值            │
│      ├─ Agent.tick() 计算控制指令     │
│      ├─ World.apply_control()        │
│      └─ World.step() 物理推进         │
└─────────────────────────────────────┘

引擎内部:
  ┌── 感知层 (Python) ────────────┐
  │  Sensor.get_percepts()        │
  │  v0: 真值透传, 预留噪声扩展点   │
  └───────────────────────────────┘
  ┌── 执行层 (Python) ────────────┐
  │  World: 实体管理, 控制下发,     │
  │         step 调度, 碰撞查询    │
  │  Agent: 智能体基类             │
  └───────────────────────────────┘
  ┌── 物理层 (C++ via pybind11) ──┐
  │  EntityState, MotionModel,    │
  │  SAT 碰撞检测, 弹性碰撞响应,   │
  │  运动积分, PhysicsWorld        │
  └───────────────────────────────┘
```

## 目录结构

```
engine/
├── __init__.py                    # from engine import World, Agent, Sensor, Percepts
├── physics/                       # C++ 物理层
│   ├── CMakeLists.txt             # pybind11 module: engine_physics
│   ├── types.h                    # Vec2d, Pose, Velocity, Polygon, EntityState, ...
│   ├── motion_model.h/.cc         # BicycleModel, SimpleModel
│   ├── collision.h/.cc            # SAT 碰撞检测 + 弹性碰撞响应
│   ├── physics_world.h/.cc        # 物理世界: step(), 实体管理
│   └── bindings.cpp               # pybind11 绑定
├── execution/                     # Python 执行层
│   ├── __init__.py
│   ├── world.py                   # World 主循环
│   └── agent.py                   # Agent 基类
└── perception/                    # Python 感知层
    ├── __init__.py
    └── sensor.py                  # Sensor + Percepts
```

## C++ 数据类型 (physics/types.h)

| 类型 | 字段 | 说明 |
|------|------|------|
| `Vec2d` | `x, y` | 2D 向量, 支持加减乘除、点积、叉积、归一化 |
| `Pose` | `x, y, theta` | 2D 位姿, 支持局部→世界坐标变换 |
| `Velocity` | `vx, vy, omega` | 世界坐标系下的速度 (线速度 + 角速度) |
| `Polygon` | `vertices: Vec2d[]` | 凸多边形 (CCW), 支持 area(), aabb(), vehicle() |
| `EntityState` | `id, pose, vel, geometry, mass, is_static` | 实体完整状态 |
| `ControlInput` | `steer, ax` | 控制指令 (方向盘转角 / 纵向加速度) |
| `CollisionResult` | `collides, normal, penetration, contact_point` | 碰撞检测结果 |
| `CollisionEvent` | `entity_a, entity_b, result` | 碰撞事件 |

## 运动模型 (physics/motion_model.h)

```
MotionModel (抽象基类)
├─ BicycleModel  — 运动学自行车模型
│   控制: steer → omega = v_lon * tan(steer) / L
│         ax    → v_lon += ax * dt
│   无侧滑假设: 速度方向始终沿车头朝向
│
└─ SimpleModel   — 简单运动模型
    控制: steer → 直接作为 omega
         ax    → 沿车头方向加速度
    允许侧滑: 碰撞后保留侧向速度分量
```

## 碰撞检测 (physics/collision.h)

### SAT (Separating Axis Theorem)

- 输入: 两凸多边形 + 各自位姿
- 算法: 将两多边形变换到世界坐标系, 对所有边法向做投影测试
- 找到最小穿透轴, 法向从实体 A 指向实体 B
- 复杂度: O((n+m)²) 对凸多边形, O(1) 对简单形状

### 弹性碰撞响应

- 质量 = 凸多边形面积 × 密度 (density=1.0), 静态实体质量视为 ∞
- 完全弹性碰撞 (恢复系数 e=1), 动量守恒
- 静态碰撞: 动态实体沿法向速度分量取反
- 两动态实体: 标准 2D 弹性碰撞公式交换法向速度分量
- 位置修正: 按质量反比沿法向分离穿透深度
- 无摩擦力

## 物理世界 (physics/physics_world.h)

```
PhysicsWorld
  add_entity(state, model) → id
  remove_entity(id)
  get_entity_state(id) → EntityState*
  apply_control(id, cmd)
  step(dt):                        # 同步推进
    1. 运动模型计算新速度
    2. 欧拉积分更新位姿
    3. O(n²) 碰撞检测 (所有实体对)
    4. 弹性碰撞响应 (速度 + 位置修正)
  get_collisions() → [CollisionEvent]
```

## Python 执行层 (execution/)

### World — 仿真世界

封装 C++ PhysicsWorld, 提供 Agent 注册、控制下发、步进调度。

```python
world = World(dt=0.01)
ego_id = world.add_entity(ego_state, BicycleModel(2.68))
world.register_agent(ego_agent)
collisions = world.step()
```

### Agent — 智能体基类

```python
class MyAgent(Agent):
    def init(self, world): ...
    def tick(self, percepts: Percepts) -> ControlInput: ...
```

## Python 感知层 (perception/)

### Sensor — 真值透传 (v0)

```python
sensor = Sensor()
percepts = sensor.get_percepts(world, entity_id)
# → Percepts(ego_state, other_entities, static_obstacles, timestamp)
```

当前不做任何噪声/延迟/遮挡处理, 预留 `Sensor` 子类化扩展点。

## 仿真循环 (两层)

```
# 1. 世界构建
world = World(dt=0.01)

# 2. 引擎初始化 — 地图解析 → 构建仿真世界
map_data = parse_map(image)
for boundary in map_data.boundaries:
    world.add_entity(StaticEntity(polygon=boundary))
for obs in static_obstacles:
    world.add_entity(StaticEntity(polygon=obs.polygon))

# 3. Agent 初始化 (各自独立感知/规划)
ego = EgoAgent()
ego.init(world)

# 4. 主循环
world.start()
sensor = Sensor()
while world.running:
    for agent in world.agents:
        percepts = sensor.get_percepts(world, agent.entity_id)
        cmd = agent.tick(percepts)
        world.apply_control(agent.entity_id, cmd)
    collisions = world.step()
```

## MVP 范围 (v0.1.0) — 已实现

- [x] 基础数据类型 (Vec2d, Pose, Velocity, Polygon, EntityState, ControlInput)
- [x] 运动模型 (BicycleModel, SimpleModel)
- [x] SAT 凸多边形碰撞检测
- [x] 完全弹性碰撞响应 + 位置修正
- [x] PhysicsWorld 同步步进
- [x] pybind11 绑定
- [x] Python World / Agent / Sensor 框架
- [x] 独立 CMake 构建集成

## 不在当前范围

- 多圆盘/复杂几何组合 (只支持凸多边形)
- 感知噪声模拟 (框架已留, 逻辑未实现)
- 事件驱动 (仅固定步长)
- NPC 行为决策 (属于 pipeline, 不在 engine)
- 摩擦力 / 非弹性碰撞 / 角速度耦合

## 构建

```bash
cmake --build build  # engine_physics 自动编译
```

Python 使用:

```python
import engine_physics as ep  # C++ 物理层
from engine import World, Agent, Sensor  # Python 框架
```

## 验证结果

| 测试 | 结果 |
|------|------|
| SAT 重叠/分离检测 | ✅ |
| 自行车模型恒 steer 圆周运动 | ✅ |
| 两车等质量弹性碰撞 (速度交换) | ✅ |
| 车撞静态墙反弹 (vx 取反) | ✅ |
| Python World/Agent/Sensor 集成 | ✅ |
| pnc 模块不受影响 | ✅ |
