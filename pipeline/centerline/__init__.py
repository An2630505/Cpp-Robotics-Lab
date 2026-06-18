"""
centerline — 赛道中心线拓扑图提取模块

从赛道边界提取中心线骨架图（节点 + 边）。

用法:
    from centerline import extract_centerline_graph

    graph = extract_centerline_graph(
        outer_boundary=boundaries["outer_boundary"],
        holes=boundaries["holes"],
    )
    print(f"{len(graph['nodes'])} nodes, {len(graph['edges'])} edges")
"""

from ._core import extract_centerline_graph

__all__ = ["extract_centerline_graph"]
