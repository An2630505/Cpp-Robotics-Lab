"""
sim_engine_demo.py — 仿真引擎最小示例

场景: 矩形围墙内多车运动, 展示引擎核心功能:
  - World 创建与实体管理
  - 静态障碍物 (围墙)
  - BicycleModel (自车) + SimpleModel (NPC)
  - 弹性碰撞检测与响应
  - Agent 模式 (控制与仿真分离)
  - matplotlib 动画可视化

用法:
  python pipeline/sim_engine_demo.py
  python pipeline/sim_engine_demo.py --save output/engine_demo.gif
"""

from __future__ import annotations

import os, sys, math, argparse
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon as MplPolygon

_project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_build_dir = os.path.join(_project_root, "build", "engine", "physics")
if _build_dir not in sys.path:
    sys.path.insert(0, _build_dir)
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

import engine_physics as ep
from engine.execution.world import World
from engine.execution.agent import Agent
from engine.perception.sensor import Sensor

# ============================================================
#  参数
# ============================================================

DT          = 0.01       # 仿真步长 10ms
SIM_TIME    = 12.0       # 总仿真时间 (s)
WALL_THICK  = 0.5        # 墙厚
ARENA_W     = 40.0       # 竞技场宽 (内径)
ARENA_H     = 30.0       # 竞技场高 (内径)
WHEELBASE   = 2.68       # 轴距
VEH_HW      = 1.0        # 车半宽
VEH_FWD     = 1.5        # 车头 (质心→前)
VEH_REV     = 1.0        # 车尾 (质心→后)
RECORD_EVERY = 5         # 每 N 步记录一帧 (动画用)

# ============================================================
#  Agent 实现
# ============================================================

class CircleAgent(Agent):
    """绕圈行驶 agent: 恒定 steer + 恒定加速度"""

    def __init__(self, entity_id: int, steer: float = 0.08, ax: float = 0.0,
                 name: str = "Agent"):
        super().__init__(entity_id)
        self.steer = steer
        self.ax = ax
        self.name = name

    def init(self, world: World) -> None:
        pass  # 简单 agent 无需初始化

    def tick(self, percepts):
        return ep.ControlInput(self.steer, self.ax)


class StraightAgent(Agent):
    """直线行驶 agent"""

    def __init__(self, entity_id: int, vx: float, vy: float, name: str = "NPC"):
        super().__init__(entity_id)
        self.vx = vx
        self.vy = vy
        self.name = name

    def init(self, world: World) -> None:
        pass

    def tick(self, percepts):
        # 保持恒定速度 (加速度为0 → SimpleModel 保速)
        return ep.ControlInput(0.0, 0.0)


# ============================================================
#  场景构建
# ============================================================

def build_arena(world: World) -> list[int]:
    """构建矩形围墙竞技场, 返回墙体 ID 列表"""
    wall_ids = []
    hw = ARENA_W / 2  # 半宽
    hh = ARENA_H / 2  # 半高
    wt = WALL_THICK / 2

    walls = [
        # (cx, cy, half_w, half_h) — AABB
        (0,  hh + wt, hw + wt, wt),    # 上墙
        (0, -hh - wt, hw + wt, wt),    # 下墙
        ( hw + wt, 0, wt, hh),          # 右墙
        (-hw - wt, 0, wt, hh),          # 左墙
    ]

    for cx, cy, wh, ht in walls:
        state = ep.EntityState()
        state.pose = ep.Pose(cx, cy, 0.0)
        state.geometry = ep.Polygon.aabb(wh, ht)
        state.is_static = True
        wid = world.add_entity(state, None)
        wall_ids.append(wid)

    return wall_ids


def build_vehicles(world: World) -> dict[str, int]:
    """添加车辆实体, 返回 {name: entity_id}"""
    ids = {}

    # 自车: 从左侧出发, 向右行驶, 轻微右转 (绕圈)
    ego_state = ep.EntityState()
    ego_state.pose = ep.Pose(-ARENA_W / 2 + 5.0, 0.0, 0.0)
    ego_state.vel = ep.Velocity(10.0, 0.0, 0.0)
    ego_state.geometry = ep.Polygon.vehicle(VEH_HW, VEH_FWD, VEH_REV)
    ids["ego"] = world.add_entity(ego_state, ep.BicycleModel(WHEELBASE))

    # NPC 1: 从右上方出发, 向左下斜行
    npc1 = ep.EntityState()
    npc1.pose = ep.Pose(ARENA_W / 2 - 6.0, ARENA_H / 2 - 6.0, math.radians(-135))
    npc1.vel = ep.Velocity(-4.0, -4.0, 0.0)
    npc1.geometry = ep.Polygon.vehicle(VEH_HW, VEH_FWD, VEH_REV)
    ids["npc1"] = world.add_entity(npc1, ep.SimpleModel())

    # NPC 2: 从下方出发, 向右上斜行
    npc2 = ep.EntityState()
    npc2.pose = ep.Pose(-2.0, -ARENA_H / 2 + 5.0, math.radians(60))
    npc2.vel = ep.Velocity(5.0, 8.0, 0.0)
    npc2.geometry = ep.Polygon.vehicle(VEH_HW * 0.8, VEH_FWD * 0.7, VEH_REV * 0.7)
    ids["npc2"] = world.add_entity(npc2, ep.SimpleModel())

    return ids


# ============================================================
#  仿真主循环
# ============================================================

def run_simulation(world: World, agents: list[Agent],
                   sensor: Sensor, dt: float, total_time: float,
                   record_every: int = 1):
    """运行仿真, 返回 log 字典"""
    n_steps = int(total_time / dt)
    record_n = (n_steps + record_every - 1) // record_every

    # 预分配日志
    log = {
        "t":        np.zeros(record_n),
        "ego_x":    np.zeros(record_n), "ego_y":    np.zeros(record_n),
        "ego_h":    np.zeros(record_n), "ego_vx":   np.zeros(record_n),
        "ego_vy":   np.zeros(record_n), "ego_steer": np.zeros(record_n),
        "npc1_x":   np.zeros(record_n), "npc1_y":   np.zeros(record_n),
        "npc1_h":   np.zeros(record_n),
        "npc2_x":   np.zeros(record_n), "npc2_y":   np.zeros(record_n),
        "npc2_h":   np.zeros(record_n),
        "collisions": [],  # list[CollisionEvent] per frame
    }

    entity_ids = {a.name: a.entity_id for a in agents}
    ego_id = entity_ids.get("ego", -1)
    npc1_id = entity_ids.get("npc1", -1)
    npc2_id = entity_ids.get("npc2", -1)
    name_by_id = {v: k for k, v in entity_ids.items()}

    agent_map = {a.entity_id: a for a in agents}

    world.start()
    ri = 0

    for step in range(n_steps):
        t = step * dt

        # Agent 决策
        for agent in agents:
            percepts = sensor.get_percepts(world, agent.entity_id)
            cmd = agent.tick(percepts)
            world.apply_control(agent.entity_id, cmd)

        # 物理步进
        collisions = world.step()

        # 记录
        if step % record_every == 0 and ri < record_n:
            log["t"][ri] = t

            if ego_id >= 0:
                s = world.get_entity_state(ego_id)
                if s:
                    log["ego_x"][ri] = s.pose.x; log["ego_y"][ri] = s.pose.y
                    log["ego_h"][ri] = s.pose.theta
                    log["ego_vx"][ri] = s.vel.vx; log["ego_vy"][ri] = s.vel.vy
                    # 记录 steer (从 agent 取)
                    log["ego_steer"][ri] = agent_map[ego_id].steer

            if npc1_id >= 0:
                s = world.get_entity_state(npc1_id)
                if s:
                    log["npc1_x"][ri] = s.pose.x; log["npc1_y"][ri] = s.pose.y
                    log["npc1_h"][ri] = s.pose.theta

            if npc2_id >= 0:
                s = world.get_entity_state(npc2_id)
                if s:
                    log["npc2_x"][ri] = s.pose.x; log["npc2_y"][ri] = s.pose.y
                    log["npc2_h"][ri] = s.pose.theta

            # 碰撞事件 (含参与方名称)
            log["collisions"].append([
                {
                    "a": c.entity_a, "b": c.entity_b,
                    "a_name": name_by_id.get(c.entity_a, f"wall_{c.entity_a}"),
                    "b_name": name_by_id.get(c.entity_b, f"wall_{c.entity_b}"),
                    "pen": c.result.penetration,
                    "nx": c.result.normal.x, "ny": c.result.normal.y,
                }
                for c in collisions
            ])
            ri += 1

    world.stop()
    log["n_frames"] = ri
    return log


# ============================================================
#  可视化
# ============================================================

def _vehicle_corners(cx, cy, h, hw, fwd, rev):
    """车辆世界坐标顶点 (4角)"""
    c, s = math.cos(h), math.sin(h)
    corners = []
    for lx, ly in [(fwd, hw), (fwd, -hw), (-rev, -hw), (-rev, hw)]:
        corners.append((cx + lx * c - ly * s, cy + lx * s + ly * c))
    return corners


def _wall_rect(cx, cy, wh, ht):
    """墙矩形顶点"""
    return [(cx - wh, cy - ht), (cx + wh, cy - ht),
            (cx + wh, cy + ht), (cx - wh, cy + ht)]


def visualize(log: dict, dt: float, record_every: int, save_path: str = None):
    """matplotlib 动画"""
    nf = log["n_frames"]

    # 预计算车载矩形
    ego_rects, npc1_rects, npc2_rects = [], [], []
    for i in range(nf):
        ego_rects.append(_vehicle_corners(
            log["ego_x"][i], log["ego_y"][i], log["ego_h"][i],
            VEH_HW, VEH_FWD, VEH_REV))
        npc1_rects.append(_vehicle_corners(
            log["npc1_x"][i], log["npc1_y"][i], log["npc1_h"][i],
            VEH_HW, VEH_FWD, VEH_REV))
        npc2_rects.append(_vehicle_corners(
            log["npc2_x"][i], log["npc2_y"][i], log["npc2_h"][i],
            VEH_HW * 0.8, VEH_FWD * 0.7, VEH_REV * 0.7))

    # 预计算围墙
    hw, hh = ARENA_W / 2, ARENA_H / 2
    wt = WALL_THICK / 2
    wall_rects = [
        _wall_rect(0,  hh + wt, hw + wt, wt),
        _wall_rect(0, -hh - wt, hw + wt, wt),
        _wall_rect( hw + wt, 0, wt, hh),
        _wall_rect(-hw - wt, 0, wt, hh),
    ]

    # 轨迹线
    ego_trail_x = [log["ego_x"][:i+1] for i in range(nf)]
    ego_trail_y = [log["ego_y"][:i+1] for i in range(nf)]
    npc1_trail_x = [log["npc1_x"][:i+1] for i in range(nf)]
    npc1_trail_y = [log["npc1_y"][:i+1] for i in range(nf)]
    npc2_trail_x = [log["npc2_x"][:i+1] for i in range(nf)]
    npc2_trail_y = [log["npc2_y"][:i+1] for i in range(nf)]

    # 碰撞标记
    collision_markers = []
    for i in range(nf):
        cs = log["collisions"][i]
        markers = []
        for c in cs:
            # 取接触点: 两实体 pose 中点
            a_name = c["a_name"]
            b_name = c["b_name"]
            if a_name in log and b_name in log:
                mx = (log[f"{a_name}_x"][i] + log[f"{b_name}_x"][i]) / 2
                my = (log[f"{a_name}_y"][i] + log[f"{b_name}_y"][i]) / 2
            elif a_name in log:
                mx, my = log[f"{a_name}_x"][i], log[f"{a_name}_y"][i]
            elif b_name in log:
                mx, my = log[f"{b_name}_x"][i], log[f"{b_name}_y"][i]
            else:
                mx, my = 0, 0
            markers.append((mx, my, c["pen"]))
        collision_markers.append(markers)

    # ======================== Figure ========================
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle("Engine Demo — 2D Simulation (Billiard Arena)", fontsize=14)

    # 主地图
    ax = fig.add_subplot(1, 1, 1)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")

    # 围墙
    for wr in wall_rects:
        ax.add_patch(MplPolygon(wr, closed=True, fc="#555", ec="#333", lw=1.5, alpha=0.7))

    # 车辆 patches
    ego_patch = MplPolygon([[0,0]]*4, closed=True, fc="#e63946", ec="#c1121f",
                            lw=2, alpha=0.85, zorder=5)
    npc1_patch = MplPolygon([[0,0]]*4, closed=True, fc="#457b9d", ec="#1d3557",
                             lw=1.5, alpha=0.75, zorder=4)
    npc2_patch = MplPolygon([[0,0]]*4, closed=True, fc="#2a9d8f", ec="#1b4332",
                             lw=1.5, alpha=0.75, zorder=4)
    ax.add_patch(ego_patch); ax.add_patch(npc1_patch); ax.add_patch(npc2_patch)

    # 轨迹线
    trail_ego,  = ax.plot([], [], "r-", lw=1.2, alpha=0.5)
    trail_npc1, = ax.plot([], [], "b-", lw=0.8, alpha=0.4)
    trail_npc2, = ax.plot([], [], "g-", lw=0.8, alpha=0.4)

    # 碰撞闪烁 (红色圆圈)
    flash, = ax.plot([], [], "ro", ms=12, alpha=0.0, zorder=10, mec="darkred", mew=2)

    # 信息文本
    info_txt = ax.text(0.015, 0.985, "", transform=ax.transAxes,
                       fontsize=9, va="top", family="monospace",
                       bbox=dict(boxstyle="round", fc="lightyellow", alpha=0.85))

    # 图例
    from matplotlib.lines import Line2D
    ax.legend(handles=[
        MplPolygon([[0,0]], fc="#e63946", ec="#c1121f", lw=2, alpha=0.85, label="Ego (Bicycle)"),
        MplPolygon([[0,0]], fc="#457b9d", ec="#1d3557", lw=1.5, alpha=0.75, label="NPC 1"),
        MplPolygon([[0,0]], fc="#2a9d8f", ec="#1b4332", lw=1.5, alpha=0.75, label="NPC 2"),
    ], loc="lower left", fontsize=8)

    # 初始视角: 整个竞技场
    ax.set_xlim(-ARENA_W / 2 - 2, ARENA_W / 2 + 2)
    ax.set_ylim(-ARENA_H / 2 - 2, ARENA_H / 2 + 2)

    # 速度曲线 (inset)
    ax_vel = ax.inset_axes([0.63, 0.02, 0.35, 0.22])
    ax_vel.set_facecolor("#f8f8f8"); ax_vel.set_alpha(0.9)
    ax_vel.set_title("Speed", fontsize=8)
    ax_vel.set_xlabel("t (s)", fontsize=7); ax_vel.set_ylabel("m/s", fontsize=7)
    ax_vel.tick_params(labelsize=6)
    ax_vel.grid(True, alpha=0.3)
    vel_ego,  = ax_vel.plot([], [], "r-", lw=1.0, label="Ego")
    vel_npc1, = ax_vel.plot([], [], "b-", lw=0.8, label="NPC1")
    vel_npc2, = ax_vel.plot([], [], "g-", lw=0.8, label="NPC2")
    ax_vel.legend(fontsize=6, loc="upper right")
    ax_vel.set_xlim(0, SIM_TIME)
    ax_vel.set_ylim(0, 18)

    # ======================== Update ========================

    def update(frame):
        fi = frame; t = log["t"][fi]

        # 车辆位置
        ego_patch.set_xy(ego_rects[fi])
        npc1_patch.set_xy(npc1_rects[fi])
        npc2_patch.set_xy(npc2_rects[fi])

        # 轨迹
        trail_ego.set_data(ego_trail_x[fi], ego_trail_y[fi])
        trail_npc1.set_data(npc1_trail_x[fi], npc1_trail_y[fi])
        trail_npc2.set_data(npc2_trail_x[fi], npc2_trail_y[fi])

        # 碰撞闪烁
        if collision_markers[fi]:
            c0 = collision_markers[fi][0]
            flash.set_data([c0[0]], [c0[1]])
            flash.set_alpha(0.8)
            flash.set_markersize(min(20, 8 + c0[2] * 8))
        else:
            flash.set_alpha(0.0)

        # 速度
        ego_speed = math.hypot(log["ego_vx"][fi], log["ego_vy"][fi])
        npc1_speed = math.hypot(
            log["npc1_x"][fi] - log["npc1_x"][max(0, fi-1)],
            log["npc1_y"][fi] - log["npc1_y"][max(0, fi-1)],
        ) / (dt * record_every) if fi > 0 else 4.0
        npc2_speed = math.hypot(
            log["npc2_x"][fi] - log["npc2_x"][max(0, fi-1)],
            log["npc2_y"][fi] - log["npc2_y"][max(0, fi-1)],
        ) / (dt * record_every) if fi > 0 else 5.0

        vel_ego.set_data(log["t"][:fi+1],
                         [math.hypot(log["ego_vx"][j], log["ego_vy"][j])
                          for j in range(fi+1)])
        vel_npc1.set_data(log["t"][:fi+1],
                          [math.hypot(
                              log["npc1_x"][j] - log["npc1_x"][max(0, j-1)],
                              log["npc1_y"][j] - log["npc1_y"][max(0, j-1)],
                          ) / (dt * record_every) if j > 0 else 4.0
                           for j in range(fi+1)])
        vel_npc2.set_data(log["t"][:fi+1],
                          [math.hypot(
                              log["npc2_x"][j] - log["npc2_x"][max(0, j-1)],
                              log["npc2_y"][j] - log["npc2_y"][max(0, j-1)],
                          ) / (dt * record_every) if j > 0 else 5.0
                           for j in range(fi+1)])

        # 碰撞统计
        total_collisions = sum(len(cs) for cs in log["collisions"][:fi+1])
        current_collisions = len(collision_markers[fi])

        info_txt.set_text(
            f"t = {t:.2f}s  |  Ego speed = {ego_speed:.1f} m/s  |  "
            f"Collisions: {total_collisions} total, {current_collisions} this frame")

        return (ego_patch, npc1_patch, npc2_patch,
                trail_ego, trail_npc1, trail_npc2,
                flash, info_txt, vel_ego, vel_npc1, vel_npc2)

    # ======================== 动画 ========================
    frame_interval = max(10, int(dt * record_every * 1000 / 1.5))  # 1.5x 播放速度
    ani = FuncAnimation(fig, update, frames=nf, interval=frame_interval, blit=False)

    if save_path:
        fps = max(1, int(round(1000 / frame_interval)))
        print(f"\nSaving {save_path} ({nf} frames, {fps} fps) ...")
        os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
        ani.save(save_path, writer="pillow", fps=fps, dpi=120)
        print("Done!")
    else:
        plt.show()


# ============================================================
#  Main
# ============================================================

def main():
    ap = argparse.ArgumentParser(description="Engine Demo — 2D Simulation")
    ap.add_argument("--save", default=None, help="保存为 GIF")
    ap.add_argument("--time", type=float, default=SIM_TIME,
                    help=f"仿真时长 (s), 默认 {SIM_TIME}")
    args = ap.parse_args()

    print("=" * 60)
    print("  2D Simulation Engine — Demo (Billiard Arena)")
    print("=" * 60)

    # [1] 创建世界
    print(f"\n[1] 创建 World (dt={DT}s) ...")
    world = World(dt=DT)

    # [2] 构建围墙
    print("[2] 构建静态围墙 ...")
    wall_ids = build_arena(world)
    print(f"    围墙: {len(wall_ids)} 段, entity_count={world.entity_count}")

    # [3] 添加车辆
    print("[3] 添加车辆 ...")
    veh_ids = build_vehicles(world)
    for name, eid in veh_ids.items():
        s = world.get_entity_state(eid)
        print(f"    {name} (id={eid}): pos=({s.pose.x:.0f},{s.pose.y:.0f}), "
              f"vel=({s.vel.vx:.0f},{s.vel.vy:.0f}), mass={s.mass:.1f}")

    # [4] 注册 Agent
    print("[4] 注册 Agent ...")
    agents = [
        CircleAgent(veh_ids["ego"], steer=0.06, ax=0.0, name="ego"),
        StraightAgent(veh_ids["npc1"], vx=-4.0, vy=-4.0, name="npc1"),
        StraightAgent(veh_ids["npc2"], vx=5.0, vy=8.0, name="npc2"),
    ]
    for a in agents:
        world.register_agent(a)
    print(f"    {len(agents)} agents 已注册")

    # [5] 运行仿真
    print(f"\n[5] 运行仿真 ({args.time:.0f}s, {int(args.time/DT)} steps) ...")
    sensor = Sensor()
    log = run_simulation(world, agents, sensor, DT, args.time,
                         record_every=RECORD_EVERY)

    nf = log["n_frames"]
    total_collisions = sum(len(cs) for cs in log["collisions"][:nf])
    print(f"    完成: {nf} 帧, {total_collisions} 次碰撞")

    # 碰撞详情
    if total_collisions > 0:
        print("\n    碰撞事件 (前10次):")
        count = 0
        for fi in range(nf):
            for c in log["collisions"][fi]:
                if count >= 10:
                    break
                print(f"      t={log['t'][fi]:.2f}s  "
                      f"{c['a_name']} ↔ {c['b_name']}  "
                      f"pen={c['pen']:.3f}m")
                count += 1

    # [6] 可视化
    print(f"\n[6] 可视化 ...")
    visualize(log, DT, RECORD_EVERY, save_path=args.save)


if __name__ == "__main__":
    main()
