"""
World — 2D 仿真世界

职责:
  - 封装 C++ PhysicsWorld
  - 实体增删查
  - Agent 注册
  - 控制指令下发
  - 主步进 (派发到物理层)
  - 碰撞事件查询
"""

from __future__ import annotations

import sys
import os
from typing import Optional

# 确保能找到 engine_physics 模块
_build_dir = os.path.join(os.path.dirname(__file__), "..", "..", "build", "engine", "physics")
if os.path.isdir(_build_dir) and _build_dir not in sys.path:
    sys.path.insert(0, _build_dir)

import engine_physics as _ep

from engine.execution.agent import Agent


class World:
    """2D 仿真世界"""

    def __init__(self, dt: float = 0.01):
        """
        Parameters
        ----------
        dt : float
            仿真步长 (s), 默认 0.01 (10ms)
        """
        self.dt = dt
        self._physics = _ep.PhysicsWorld()
        self._agents: list[Agent] = []
        self._running = False
        self._step_count = 0

    # ================================================================
    # 属性
    # ================================================================

    @property
    def dt(self) -> float:
        return self._dt

    @dt.setter
    def dt(self, value: float):
        if value <= 0:
            raise ValueError("dt must be positive")
        self._dt = value

    @property
    def step_count(self) -> int:
        return self._step_count

    @property
    def running(self) -> bool:
        return self._running

    @property
    def agents(self) -> list[Agent]:
        return self._agents

    # ================================================================
    # 实体管理
    # ================================================================

    def add_entity(self, state: _ep.EntityState,
                   model: _ep.MotionModel) -> int:
        """
        添加实体到世界。

        Parameters
        ----------
        state : EntityState
            实体初始状态 (geometry 为局部坐标系下的凸多边形)
        model : MotionModel
            实体的运动模型 (BicycleModel / SimpleModel)

        Returns
        -------
        int
            实体 ID
        """
        return self._physics.add_entity(state, model)

    def remove_entity(self, entity_id: int) -> None:
        """移除实体"""
        self._physics.remove_entity(entity_id)

    def get_entity_state(self, entity_id: int) -> Optional[_ep.EntityState]:
        """获取实体状态 (只读)"""
        return self._physics.get_entity_state(entity_id)

    def get_all_states(self) -> list[_ep.EntityState]:
        """获取所有实体状态"""
        return self._physics.get_all_states()

    def get_all_entity_ids(self) -> list[int]:
        """获取所有实体 ID"""
        return self._physics.get_all_entity_ids()

    @property
    def entity_count(self) -> int:
        return self._physics.entity_count()

    # ================================================================
    # Agent 管理
    # ================================================================

    def register_agent(self, agent: Agent) -> None:
        """注册智能体"""
        self._agents.append(agent)

    # ================================================================
    # 控制接口
    # ================================================================

    def apply_control(self, entity_id: int, cmd: _ep.ControlInput) -> None:
        """下发控制指令"""
        self._physics.apply_control(entity_id, cmd)

    # ================================================================
    # 步进
    # ================================================================

    def step(self) -> list[_ep.CollisionEvent]:
        """
        推进仿真一步。

        注意: agent 决策 (tick) 应该在 step() 之前由外部循环完成。
        本方法只执行物理积分 + 碰撞检测/响应。

        Returns
        -------
        list[CollisionEvent]
            本步产生的碰撞事件
        """
        self._physics.step(self._dt)
        self._step_count += 1
        return self._physics.get_collisions()

    def start(self) -> None:
        """标记仿真开始"""
        self._running = True
        self._step_count = 0

    def stop(self) -> None:
        """标记仿真停止"""
        self._running = False
