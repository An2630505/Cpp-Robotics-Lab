"""
Sensor — 感知层 (v0: 真值透传)

为每个 agent 提供世界信息的访问接口。
当前版本不做任何噪声/延迟/遮挡处理, 直接透传引擎真值。
后续可在此层添加:
  - 定位噪声 (GPS/IMU 误差模型)
  - 检测噪声 (漏检/误检/位置误差)
  - 感知范围限制 (FOV / 最大距离)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from engine.execution.world import World
    from engine_physics import EntityState, Polygon


@dataclass
class Percepts:
    """一次感知帧的结果"""
    ego_state: "EntityState"          # 自身的完整状态
    other_entities: list["EntityState"] = field(default_factory=list)  # 其他实体的状态
    static_obstacles: list["Polygon"] = field(default_factory=list)    # 静态障碍物的几何 (世界坐标系)
    timestamp: float = 0.0


class Sensor:
    """
    感知传感器 (v0: 真值透传)

    Usage
    -----
    sensor = Sensor()
    percepts = sensor.get_percepts(world, ego_entity_id)
    """

    def get_percepts(self, world: "World", entity_id: int) -> Percepts:
        """
        获取指定实体的感知信息。

        Parameters
        ----------
        world : World
            仿真世界
        entity_id : int
            感知主体 (ego) 的实体 ID

        Returns
        -------
        Percepts
            感知结果 (v0: 完整真值)
        """
        all_states = world.get_all_states()

        ego_state = None
        other_entities = []
        static_obstacles = []

        import engine_physics as _ep

        for s in all_states:
            if s.id == entity_id:
                ego_state = s
            elif s.is_static:
                # 将静态障碍物几何变换到世界坐标系后返回
                world_verts = [s.pose.transform(v) for v in s.geometry.vertices]
                static_obstacles.append(_ep.Polygon(world_verts))
            else:
                other_entities.append(s)

        if ego_state is None:
            raise ValueError(f"Entity {entity_id} not found in world")

        return Percepts(
            ego_state=ego_state,
            other_entities=other_entities,
            static_obstacles=static_obstacles,
            timestamp=float(world.step_count) * world.dt,
        )
