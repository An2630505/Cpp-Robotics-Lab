"""
Engine — 2D 仿真引擎

三层架构:
  - physics/  : C++ 物理层 (运动模型、碰撞检测、物理积分)
  - execution/: Python 执行层 (World 主循环、Agent 基类)
  - perception/: Python 感知层 (真值透传, 预留噪声扩展)
"""

from engine.execution.world import World
from engine.execution.agent import Agent
from engine.perception.sensor import Sensor, Percepts

__all__ = ["World", "Agent", "Sensor", "Percepts"]
