"""
test_static_obstacles.py — 验证 static_obstacles 模块

用法:
    python pipeline/test_static_obstacles.py
    python pipeline/test_static_obstacles.py --visualize
    python pipeline/test_static_obstacles.py --save-json config/test_obstacles.json
"""

from __future__ import annotations

import argparse
import json
import os
import sys

_self_dir = os.path.dirname(os.path.abspath(__file__))
if _self_dir not in sys.path:
    sys.path.insert(0, _self_dir)

import numpy as np

from static_obstacles import Obstacle, ObstacleLayer


# =====================================================================
#  验证函数
# =====================================================================


def validate_obstacle() -> list[str]:
    """验证 Obstacle 数据类创建。"""
    issues: list[str] = []

    # 矩形
    o = Obstacle(type="rectangle",
                 params={"center": (1, 2), "width": 3, "height": 4, "yaw": 0.5},
                 dilate_margin=0.2)
    if o.type != "rectangle":
        issues.append("FAIL: Obstacle type 应为 rectangle")
    else:
        issues.append("OK:   Obstacle(type=rectangle) 创建成功")

    # 圆形
    o = Obstacle(type="circle",
                 params={"center": (5, 6), "radius": 2.0},
                 dilate_margin=0.1)
    if o.type != "circle":
        issues.append("FAIL: Obstacle type 应为 circle")
    else:
        issues.append("OK:   Obstacle(type=circle) 创建成功")

    # 多边形
    o = Obstacle(type="polygon",
                 params={"vertices": [(0, 0), (1, 0), (1, 1)]},
                 dilate_margin=0.0)
    if o.type != "polygon":
        issues.append("FAIL: Obstacle type 应为 polygon")
    else:
        issues.append("OK:   Obstacle(type=polygon) 创建成功")

    return issues


def validate_layer_add() -> list[str]:
    """验证 ObstacleLayer 添加方法。"""
    issues: list[str] = []

    layer = ObstacleLayer()
    if len(layer) != 0:
        issues.append("FAIL: 空 ObstacleLayer 长度应为 0")
    else:
        issues.append("OK:   空 ObstacleLayer 长度=0")

    # add_rectangle
    r = layer.add_rectangle(center=(10, 20), width=2.0, height=5.0, yaw=0.3)
    if r.type != "rectangle":
        issues.append("FAIL: add_rectangle 返回类型错误")
    else:
        issues.append("OK:   add_rectangle() 成功")

    # add_circle
    c = layer.add_circle(center=(30, 15), radius=1.5)
    if c.type != "circle":
        issues.append("FAIL: add_circle 返回类型错误")
    else:
        issues.append("OK:   add_circle() 成功")

    # add_polygon
    p = layer.add_polygon(vertices=[(0, 0), (2, 0), (2, 2), (0, 2)])
    if p.type != "polygon":
        issues.append("FAIL: add_polygon 返回类型错误")
    else:
        issues.append("OK:   add_polygon() 成功")

    if len(layer) != 3:
        issues.append(f"FAIL: 应有 3 个 obstacle，当前 {len(layer)}")
    else:
        issues.append("OK:   ObstacleLayer 长度=3")

    # 参数校验
    try:
        layer.add_rectangle(center=(0, 0), width=-1, height=2)
        issues.append("FAIL: add_rectangle(width=-1) 应抛出 ValueError")
    except ValueError:
        issues.append("OK:   add_rectangle(width=-1) 正确抛出 ValueError")

    try:
        layer.add_circle(center=(0, 0), radius=0)
        issues.append("FAIL: add_circle(radius=0) 应抛出 ValueError")
    except ValueError:
        issues.append("OK:   add_circle(radius=0) 正确抛出 ValueError")

    try:
        layer.add_polygon(vertices=[(0, 0), (1, 1)])
        issues.append("FAIL: add_polygon(2点) 应抛出 ValueError")
    except ValueError:
        issues.append("OK:   add_polygon(2点) 正确抛出 ValueError")

    return issues


def validate_apply_to_grid() -> list[str]:
    """验证 apply_to_grid 栅格注入。"""
    issues: list[str] = []

    # 构造 20×20 grid，cell_size=1.0，从 (0,0) 开始
    rows, cols = 20, 20
    cell_size = 1.0
    grid = [[0] * cols for _ in range(rows)]
    grid_meta = {
        "x_min": 0.0, "y_min": 0.0,
        "x_max": 20.0, "y_max": 20.0,
        "rows": rows, "cols": cols,
        "cell_size": cell_size,
    }

    # 添加一个 4×6 矩形在 (10, 10)
    layer = ObstacleLayer()
    layer.add_rectangle(center=(10, 10), width=4.0, height=6.0)
    layer.apply_to_grid(grid, grid_meta)

    # 检查: (10,10) 应该在障碍物内 → grid[10][10] == 1
    if grid[10][10] != 1:
        issues.append(f"FAIL: (10,10) 应在障碍物内，grid[10][10]={grid[10][10]}")
    else:
        issues.append("OK:   障碍物中心 (10,10) grid=1")

    # 检查: (0,0) 应该在障碍物外 → grid[0][0] == 0
    if grid[0][0] != 0:
        issues.append(f"FAIL: (0,0) 应在障碍物外，grid[0][0]={grid[0][0]}")
    else:
        issues.append("OK:   障碍物外 (0,0) grid=0")

    # 验证障碍物 cell 数量 (4×6 = 24)
    obs_cells = sum(row.count(1) for row in grid)
    # 允许少量偏差 (扫描线 + cell 对齐)
    if abs(obs_cells - 24) > 6:
        issues.append(f"WARN: 障碍物 cell 数 {obs_cells} 偏离预期 24")
    else:
        issues.append(f"OK:   障碍物 cell 数 = {obs_cells} (预期约 24)")

    return issues


def validate_to_polygons() -> list[str]:
    """验证 to_polygons 多边形输出。"""
    issues: list[str] = []

    layer = ObstacleLayer()
    layer.add_rectangle(center=(10, 20), width=2.0, height=4.0, yaw=0.0)
    layer.add_circle(center=(30, 15), radius=3.0)

    polys = layer.to_polygons()

    if len(polys) != 2:
        issues.append(f"FAIL: to_polygons 应返回 2 个多边形，当前 {len(polys)}")
    else:
        issues.append("OK:   to_polygons() 返回 2 个多边形")

    # 矩形: 4 个顶点
    if polys[0].shape != (4, 2):
        issues.append(f"FAIL: 矩形多边形 shape 应为 (4,2)，当前 {polys[0].shape}")
    else:
        issues.append(f"OK:   矩形 shape={polys[0].shape}")

    # 验证矩形顶点 (center=(10,20), w=2, h=4, yaw=0)
    expected = np.array([
        [9, 18], [11, 18], [11, 22], [9, 22]
    ])
    if not np.allclose(polys[0], expected, atol=0.01):
        issues.append(
            f"FAIL: 矩形顶点不匹配\n"
            f"      期望 {expected.tolist()}\n"
            f"      实际 {polys[0].tolist()}"
        )
    else:
        issues.append("OK:   矩形顶点坐标正确")

    # 圆: 32 段 (默认)
    if polys[1].shape[0] != 32:
        issues.append(f"FAIL: 圆多边形应有 32 段，当前 {polys[1].shape[0]}")
    else:
        issues.append("OK:   圆多边形 = 32 段")

    # 验证圆半径
    cx, cy = 30, 15
    dists = np.sqrt((polys[1][:, 0] - cx) ** 2 + (polys[1][:, 1] - cy) ** 2)
    if not np.allclose(dists, 3.0, atol=0.01):
        issues.append(
            f"FAIL: 圆半径不匹配，min={dists.min():.3f}, max={dists.max():.3f}"
        )
    else:
        issues.append("OK:   圆半径一致性正确")

    return issues


def validate_json_roundtrip(tmp_path: str) -> list[str]:
    """验证 JSON 保存和加载往返。"""
    issues: list[str] = []

    # 创建 layer → 保存 JSON
    layer1 = ObstacleLayer()
    layer1.add_rectangle(center=(5, 6), width=3, height=4, yaw=0.5,
                         dilate_margin=0.2)
    layer1.add_circle(center=(10, 12), radius=2.5, dilate_margin=0.1)

    # 手动构造 JSON 数据
    data = {
        "obstacles": [
            {
                "type": "rectangle",
                "center": [5.0, 6.0],
                "width": 3.0,
                "height": 4.0,
                "yaw": 0.5,
                "dilate_margin": 0.2,
            },
            {
                "type": "circle",
                "center": [10.0, 12.0],
                "radius": 2.5,
                "dilate_margin": 0.1,
            },
        ]
    }
    with open(tmp_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

    # 加载
    layer2 = ObstacleLayer()
    n = layer2.add_from_json(tmp_path)
    if n != 2:
        issues.append(f"FAIL: add_from_json 应加载 2 个，实际 {n}")
    else:
        issues.append("OK:   add_from_json 加载 2 个障碍物")

    if len(layer2) != 2:
        issues.append(f"FAIL: 加载后应有 2 个 obstacle，当前 {len(layer2)}")
    else:
        issues.append("OK:   加载后 layer 长度=2")

    # 验证加载的障碍物
    o0 = layer2.obstacles[0]
    if o0.type != "rectangle" or o0.dilate_margin != 0.2:
        issues.append("FAIL: 加载的矩形参数不正确")
    else:
        issues.append("OK:   矩形参数正确 (type=rectangle, dilate=0.2)")

    o1 = layer2.obstacles[1]
    if o1.type != "circle" or o1.params["radius"] != 2.5:
        issues.append("FAIL: 加载的圆形参数不正确")
    else:
        issues.append("OK:   圆形参数正确 (type=circle, radius=2.5)")

    # cleanup
    os.remove(tmp_path)
    return issues


def validate_rectangle_rotation() -> list[str]:
    """验证矩形旋转。"""
    issues: list[str] = []

    layer = ObstacleLayer()
    # 矩形 center=(10,10), w=4, h=2, yaw=90° → 宽变成高
    layer.add_rectangle(center=(10, 10), width=4.0, height=2.0,
                        yaw=math.radians(90))
    polys = layer.to_polygons()
    rect = polys[0]

    # yaw=90°: (x,y) → (-y, x), 原先 (-2,-1) → (1,-2) + (10,10) = (11,8)
    # 四个角: (11,8), (9,8), (9,12), (11,12) 略——等等让我重算
    # 原四角 (w=4,h=2): (-2,-1), (2,-1), (2,1), (-2,1)
    # 旋转90°: (1,-2), (1,2), (-1,2), (-1,-2)
    # + (10,10): (11,8), (11,12), (9,12), (9,8)

    # 验证: 新 bounding box = [9,11] × [8,12] → 宽2 高4 (正确交换)
    w_out = float(np.max(rect[:, 0]) - np.min(rect[:, 0]))
    h_out = float(np.max(rect[:, 1]) - np.min(rect[:, 1]))
    if abs(w_out - 2.0) > 0.1 or abs(h_out - 4.0) > 0.1:
        issues.append(
            f"FAIL: 旋转 90° 后 bbox 应为 2×4，实际 {w_out:.3f}×{h_out:.3f}"
        )
    else:
        issues.append(f"OK:   旋转 90° bbox = {w_out:.1f}×{h_out:.1f} (宽高交换)")

    return issues


def validate_empty_layer() -> list[str]:
    """验证空 layer 的边界行为。"""
    issues: list[str] = []

    # 空 layer 的 apply_to_grid 不应报错
    layer = ObstacleLayer()
    grid = [[0] * 10 for _ in range(10)]
    grid_meta = {
        "x_min": 0, "y_min": 0, "x_max": 10, "y_max": 10,
        "rows": 10, "cols": 10, "cell_size": 1.0,
    }
    try:
        layer.apply_to_grid(grid, grid_meta)
        issues.append("OK:   空 layer.apply_to_grid() 不报错")
    except Exception as e:
        issues.append(f"FAIL: 空 layer.apply_to_grid() 报错: {e}")

    # 空 layer 的 to_polygons 应返回空列表
    polys = layer.to_polygons()
    if len(polys) != 0:
        issues.append(f"FAIL: 空 layer.to_polygons() 应返回 []，实际 {polys}")
    else:
        issues.append("OK:   空 layer.to_polygons() 返回 []")

    # repr
    r = repr(layer)
    if "empty" not in r:
        issues.append(f"FAIL: 空 layer repr 应包含 'empty'，实际 '{r}'")
    else:
        issues.append("OK:   空 layer repr = '{r}'")

    return issues


# =====================================================================
#  可视化
# =====================================================================

import math


def visualize():
    """在 path2 赛道上展示障碍物。"""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib 未安装，跳过可视化")
        return

    from pipeline.map_parser import parse_map

    _proj_root = os.path.dirname(_self_dir)
    img_path = os.path.join(_proj_root, "map", "path2.png")
    if not os.path.exists(img_path):
        print(f"地图文件不存在: {img_path}，跳过可视化")
        return

    print(f"解析地图: {img_path}")
    bounds = parse_map(img_path, pixels_per_meter=12.8, smoothing_factor=0.0,
                       num_control_points=200, resample_spacing_m=0.1,
                       has_starting_line=True)
    outer = np.array(bounds["outer_boundary"])
    holes = [np.array(h) for h in bounds["holes"]]

    # 创建障碍物
    obs = ObstacleLayer()

    # 取外边界中间区域放几个障碍物
    cx = float(np.mean(outer[:, 0]))
    cy = float(np.mean(outer[:, 1]))
    span_x = float(np.max(outer[:, 0]) - np.min(outer[:, 0]))
    span_y = float(np.max(outer[:, 1]) - np.min(outer[:, 1]))
    scale = min(span_x, span_y)

    # 几个示例障碍物
    obs.add_rectangle(
        center=(cx + scale * 0.05, cy + scale * 0.08),
        width=scale * 0.04, height=scale * 0.12, yaw=0.3,
    )
    obs.add_circle(
        center=(cx - scale * 0.08, cy - scale * 0.05),
        radius=scale * 0.03,
    )
    obs.add_rectangle(
        center=(cx + scale * 0.10, cy - scale * 0.10),
        width=scale * 0.06, height=scale * 0.06, yaw=math.radians(45),
    )

    fig, ax = plt.subplots(1, 1, figsize=(14, 12))
    ax.set_aspect("equal")
    ax.set_title("static_obstacles — 可视化验证")

    # 赛道边界
    ax.plot(outer[:, 0], outer[:, 1], "k-", lw=1.5, alpha=0.5, label="Track boundary")
    for i, h in enumerate(holes):
        ax.fill(h[:, 0], h[:, 1], fc="white", ec="gray", lw=0.5, alpha=0.9,
                label="Holes" if i == 0 else "")

    # 障碍物
    polys = obs.to_polygons()
    colors = ["red", "orange", "purple", "brown", "cyan"]
    for i, poly in enumerate(polys):
        color = colors[i % len(colors)]
        ax.fill(poly[:, 0], poly[:, 1], fc=color, ec="darkred",
                lw=2.0, alpha=0.6,
                label=f"Obstacle {i} ({obs.obstacles[i].type})")
        # 标注中心
        pc = np.mean(poly, axis=0)
        ax.text(pc[0], pc[1], str(i), ha="center", va="center",
                fontsize=9, fontweight="bold")

    ax.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    plt.show()


# =====================================================================
#  main
# =====================================================================


def main() -> int:
    ap = argparse.ArgumentParser(
        description="验证 static_obstacles 模块"
    )
    ap.add_argument("--visualize", action="store_true",
                    help="在赛道图上可视化障碍物")
    ap.add_argument("--save-json", default=None,
                    help="保存示例 JSON 配置文件到指定路径")
    args = ap.parse_args()

    # 保存 JSON 示例
    if args.save_json:
        data = {
            "obstacles": [
                {
                    "type": "rectangle",
                    "center": [15.0, 20.0],
                    "width": 2.0,
                    "height": 5.0,
                    "yaw": 0.3,
                    "dilate_margin": 0.2,
                },
                {
                    "type": "circle",
                    "center": [30.0, 15.0],
                    "radius": 1.5,
                    "dilate_margin": 0.1,
                },
                {
                    "type": "polygon",
                    "vertices": [[10, 10], [12, 10], [12, 13], [11, 14], [10, 13]],
                    "dilate_margin": 0.0,
                },
            ]
        }
        os.makedirs(os.path.dirname(args.save_json) or ".", exist_ok=True)
        with open(args.save_json, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
            f.write("\n")
        print(f"[OK] 示例配置已保存: {args.save_json}")
        return 0

    # 运行测试
    print("=" * 60)
    print("  static_obstacles 模块测试")
    print("=" * 60)

    all_issues: list[str] = []

    test_suites = [
        ("Obstacle 数据类", validate_obstacle),
        ("ObstacleLayer 添加方法", validate_layer_add),
        ("apply_to_grid 栅格注入", validate_apply_to_grid),
        ("to_polygons 多边形输出", validate_to_polygons),
        ("矩形旋转", validate_rectangle_rotation),
        ("空 layer 边界行为", validate_empty_layer),
    ]

    for name, fn in test_suites:
        print(f"\n--- {name} ---")
        issues = fn()
        all_issues.extend(issues)
        for issue in issues:
            prefix = "  "
            if issue.startswith("FAIL"):
                prefix = "  [FAIL] "
            elif issue.startswith("WARN"):
                prefix = "  [WARN] "
            elif issue.startswith("OK"):
                prefix = "  [OK]   "
            print(f"{prefix}{issue}")

    # JSON 往返测试 (需要临时文件)
    print(f"\n--- JSON 配置文件加载 ---")
    tmp_json = os.path.join(
        os.path.dirname(__file__) or ".", "__test_obstacles_tmp.json"
    )
    json_issues = validate_json_roundtrip(tmp_json)
    all_issues.extend(json_issues)
    for issue in json_issues:
        prefix = "  "
        if issue.startswith("FAIL"):
            prefix = "  [FAIL] "
        elif issue.startswith("WARN"):
            prefix = "  [WARN] "
        elif issue.startswith("OK"):
            prefix = "  [OK]   "
        print(f"{prefix}{issue}")

    # 汇总
    has_failures = any("FAIL" in i for i in all_issues)
    n_ok = sum(1 for i in all_issues if i.startswith("OK:"))
    n_fail = sum(1 for i in all_issues if i.startswith("FAIL:"))
    n_warn = sum(1 for i in all_issues if i.startswith("WARN:"))

    print(f"\n{'=' * 60}")
    print(f"  OK: {n_ok}  FAIL: {n_fail}  WARN: {n_warn}")
    print(f"  {'[PASS] 全部测试通过' if not has_failures else '[FAIL] 存在失败项'}")
    print(f"{'=' * 60}")

    if not has_failures and args.visualize:
        print("\n启动可视化...")
        visualize()

    return 0 if not has_failures else 1


if __name__ == "__main__":
    sys.exit(main())
