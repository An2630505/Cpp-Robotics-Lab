"""
test_centerline.py — verify centerline graph extraction from path1.jpg

Usage:
    python pipeline/test_centerline.py
    python pipeline/test_centerline.py --visualize
    python pipeline/test_centerline.py --save output/test_graph.json
"""

from __future__ import annotations

import argparse
import json
import os
import sys

_self_dir = os.path.dirname(os.path.abspath(__file__))
if _self_dir not in sys.path:
    sys.path.insert(0, _self_dir)

from map_parser import parse_map
from centerline import extract_centerline_graph


def validate_graph(graph: dict) -> list[str]:
    """Run validation checks, return list of issues."""
    issues: list[str] = []
    nodes = graph["nodes"]
    edges = graph["edges"]
    meta = graph["metadata"]

    # 1. Node count
    if len(nodes) < 1:
        issues.append("WARN: 0 nodes found (expected at least 1)")
    else:
        issues.append(f"OK: {len(nodes)} node(s)")

    # 2. Edge count
    if len(edges) == 0:
        issues.append("FAIL: No edges found")
    else:
        issues.append(f"OK: {len(edges)} edge(s)")

    # 3. Edge validation
    for e in edges:
        n_pts = len(e["points"])
        if n_pts < 5:
            issues.append(f"WARN: Edge {e['id']} has only {n_pts} points")
        length = e["length_m"]
        if length < 0.1:
            issues.append(f"WARN: Edge {e['id']} length = {length:.3f}m (too short)")
        # Check from/to refer to valid nodes
        if e["from"] >= len(nodes) or e["to"] >= len(nodes):
            issues.append(f"FAIL: Edge {e['id']} references invalid node {e['from']}->{e['to']}")
        else:
            issues.append(
                f"OK: Edge {e['id']}: node{e['from']}->node{e['to']}, "
                f"{n_pts} pts, {length:.1f}m"
            )

    # 4. Total graph length sanity
    total_len = sum(e["length_m"] for e in edges)
    outer_perimeter = meta.get("outer_perimeter_m", 0)
    if total_len > outer_perimeter * 1.5:
        issues.append(
            f"WARN: Total edge length {total_len:.1f}m >> outer perimeter {outer_perimeter:.1f}m"
        )
    issues.append(f"INFO: Total graph length = {total_len:.1f}m")

    # 5. Metadata
    for key in ["num_nodes", "num_edges", "smoothing_factor", "actual_scale"]:
        if key not in meta:
            issues.append(f"WARN: Metadata missing '{key}'")

    # Check expected for path1.jpg: 1 node, 4 edges
    if len(nodes) == 1 and len(edges) == 4:
        issues.append("OK: Match expected structure for path1.jpg (1 node, 4 edges)")
    elif len(nodes) >= 1 and len(edges) >= 2:
        issues.append(f"INFO: {len(nodes)} node(s), {len(edges)} edges — "
                      "expected 1 node, 4 edges for path1.jpg")
    else:
        issues.append("WARN: Unexpected graph structure")

    return issues


def visualize(graph: dict, boundaries: dict, save_path: str | None = None):
    """Plot centerline graph over track boundaries."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available, skipping visualization")
        return

    fig, ax = plt.subplots(figsize=(10, 10))

    # Draw boundaries
    outer = boundaries["outer_boundary"]
    if len(outer) > 0:
        ox, oy = zip(*outer)
        ax.plot(ox, oy, "gray", linewidth=0.5, alpha=0.5, label="Outer boundary")
    for i, hole in enumerate(boundaries.get("holes", [])):
        hx, hy = zip(*hole)
        label = "Holes" if i == 0 else None
        ax.plot(hx, hy, "gray", linewidth=0.5, alpha=0.3, label=label)

    # Draw edges
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]
    for i, e in enumerate(graph["edges"]):
        pts = e["points"]
        ex, ey = zip(*pts)
        c = colors[i % len(colors)]
        ax.plot(ex, ey, "-", color=c, linewidth=2.0,
                label=f"Edge {e['id']} ({e['length_m']:.1f}m)")

    # Draw nodes
    for n in graph["nodes"]:
        ax.plot(n["x"], n["y"], "ro", markersize=8, zorder=5)
        ax.annotate(f"  N{n['id']}", (n["x"], n["y"]),
                    fontsize=10, fontweight="bold", color="red")

    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend(loc="upper right")
    ax.set_title("Centerline Graph — path1.jpg")
    ax.invert_yaxis()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Plot saved: {save_path}")
    else:
        plt.show()


def main() -> int:
    ap = argparse.ArgumentParser(description="Verify centerline graph extraction")
    ap.add_argument("--image", default="pipeline/map_parser/path1.jpg")
    ap.add_argument("--visualize", action="store_true")
    ap.add_argument("--save", default=None, help="Save graph JSON")
    ap.add_argument("--save-plot", default=None, help="Save visualization plot")
    ap.add_argument("--smoothing-factor", type=float, default=0.02)
    ap.add_argument("--pixels-per-meter", type=float, default=12.8)
    args = ap.parse_args()

    image_path = args.image
    if not os.path.isabs(image_path) and not os.path.exists(image_path):
        alt_path = os.path.join(_self_dir, "..", args.image)
        if os.path.exists(alt_path):
            image_path = os.path.normpath(alt_path)

    print(f"=== Testing centerline graph extraction: {image_path} ===")

    # Step 1: Parse boundaries
    print("Step 1: Parsing boundaries with map_parser...")
    boundaries = parse_map(
        image_path=image_path,
        pixels_per_meter=args.pixels_per_meter,
        smoothing_factor=args.smoothing_factor,
    )
    print(f"  Outer: {len(boundaries['outer_boundary'])} pts, "
          f"Holes: {len(boundaries['holes'])}")

    # Step 2: Extract centerline graph
    print("Step 2: Extracting centerline graph...")
    graph = extract_centerline_graph(
        outer_boundary=boundaries["outer_boundary"],
        holes=boundaries["holes"],
        pixels_per_meter=args.pixels_per_meter,
        smoothing_factor=args.smoothing_factor,
    )

    # Step 3: Validate
    print("Step 3: Validating...")
    issues = validate_graph(graph)
    for issue in issues:
        prefix = {
            "FAIL": "  [FAIL] ",
            "WARN": "  [WARN] ",
            "OK": "  [OK]   ",
        }.get(issue[:4], "  ")
        print(f"{prefix}{issue}")

    has_failures = any(i.startswith("FAIL") for i in issues)

    if args.save:
        os.makedirs(os.path.dirname(os.path.abspath(args.save)) or ".", exist_ok=True)
        with open(args.save, "w", encoding="utf-8") as f:
            json.dump(graph, f, indent=2, ensure_ascii=False)
        print(f"Saved: {args.save}")

    if args.visualize or args.save_plot:
        visualize(graph, boundaries, save_path=args.save_plot)

    print(f"\n{'[PASS] Test passed' if not has_failures else '[FAIL] Test failed'}")
    return 0 if not has_failures else 1


if __name__ == "__main__":
    sys.exit(main())
