"""
dynamic_obstacles — 动态障碍物模块

管理沿道路中心线匀速行驶的 NPC 车辆，支持位姿预测、栅格注入和碰撞检测。

用法:
    from pipeline.dynamic_obstacles import NpcVehicle, NpcManager

    mgr = NpcManager()
    mgr.add_npc(NpcVehicle(start_ratio=0.25, speed=5.0,
                           half_width=1.0, half_len_fwd=1.5, half_len_rev=1.0))

    # 每仿真步更新
    mgr.update(t, centerline_pts, cum_s, total_len)

    # 注入栅格 (用于 HA* 重规划)
    mgr.apply_to_grid(grid, grid_meta, dilation_m=1.5)

    # 碰撞检测
    if mgr.check_collision_with_ego(ego_x, ego_y, ego_heading, 1.2, 1.7, 1.2):
        print("碰撞!")
"""

from ._core import NpcVehicle, NpcManager

__all__ = ["NpcVehicle", "NpcManager"]
