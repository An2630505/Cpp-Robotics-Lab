"""
Agent 基类

每个 Agent 是引擎世界中一个智能体的抽象。
Agent 持有引擎中对应实体的 ID, 通过感知层获取世界信息, 输出控制指令。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from engine.execution.world import World
    from engine.perception.sensor import Percepts
    from engine_physics import ControlInput


class Agent(ABC):
    """智能体基类"""

    def __init__(self, entity_id: int = -1):
        self.entity_id = entity_id

    @abstractmethod
    def init(self, world: World) -> None:
        """
        智能体初始化。

        在仿真循环开始前调用。agent 在此方法内:
        1. 感知世界 (地图解析、中心线等)
        2. 初始化自己的规划/控制算法
        3. 创建并注册引擎实体 (如果还没做)

        Parameters
        ----------
        world : World
            仿真世界实例
        """
        ...

    @abstractmethod
    def tick(self, percepts: Percepts) -> ControlInput:
        """
        每帧决策。

        Parameters
        ----------
        percepts : Percepts
            从感知层获取的世界信息 (v0: 真值透传)

        Returns
        -------
        ControlInput
            本帧的控制指令 (steer, ax)
        """
        ...
