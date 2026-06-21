"""
static_obstacles — 静态障碍物模块

支持编程式和配置文件两种方式在赛道上定义静态障碍物（矩形、圆形、多边形），
并注入到占用栅格和 SafeCorridor 中。

用法:
    from pipeline.static_obstacles import ObstacleLayer

    obs = ObstacleLayer()

    # 编程式添加
    obs.add_rectangle(center=(15, 20), width=2.0, height=5.0, yaw=0.3)
    obs.add_circle(center=(30, 15), radius=1.5)

    # 从 JSON 配置文件加载
    obs.add_from_json("config/obstacles.json")

    # 应用到占用栅格
    obs.apply_to_grid(grid, grid_meta)

    # 转为多边形列表（供 SafeCorridor 使用）
    extra_holes = obs.to_polygons()
"""

from ._core import Obstacle, ObstacleLayer

__all__ = ["Obstacle", "ObstacleLayer"]
