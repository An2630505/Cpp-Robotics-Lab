"""
test_map_parser.py — 验证 map_parser 模块对 path1.jpg 的处理

用法:
    python pipeline/test_map_parser.py
    python pipeline/test_map_parser.py --visualize
    python pipeline/test_map_parser.py --save output/test_boundaries.json
"""

from __future__ import annotations

import argparse
import json
import os
import sys

# Ensure map_parser is importable from pipeline/ directory
_self_dir = os.path.dirname(os.path.abspath(__file__))
if _self_dir not in sys.path:
    sys.path.insert(0, _self_dir)

from map_parser import parse_map


def validate_result(result: dict) -> list[str]:
    """Run validation checks and return list of issues (empty = passes)."""
    issues: list[str] = []

    outer = result["outer_boundary"]
    holes = result["holes"]
    meta = result["metadata"]

    # 1. Outer boundary must exist and be plausible
    if len(outer) == 0:
        issues.append("FAIL: No outer boundary found")
    elif len(outer) < 10:
        issues.append(f"WARN: Outer boundary has only {len(outer)} points (likely noise)")
    else:
        dx = outer[0][0] - outer[-1][0]
        dy = outer[0][1] - outer[-1][1]
        gap = (dx * dx + dy * dy) ** 0.5
        if gap > 0.5:
            issues.append(f"WARN: Outer boundary gap = {gap:.3f}m (may not be closed)")
        else:
            issues.append(f"OK: Outer boundary closed, gap = {gap:.4f}m, {len(outer)} pts")

    # 2. Holes
    issues.append(f"INFO: Found {len(holes)} hole(s)")
    for i, hole in enumerate(holes):
        if len(hole) < 3:
            issues.append(f"WARN: Hole[{i}] has only {len(hole)} points")
        else:
            dx = hole[0][0] - hole[-1][0]
            dy = hole[0][1] - hole[-1][1]
            gap = (dx * dx + dy * dy) ** 0.5
            issues.append(f"OK: Hole[{i}] {len(hole)} pts, gap = {gap:.4f}m")

    # 2.5 Starting line
    starting = result.get("starting_line", [])
    if starting:
        issues.append(f"INFO: Found {len(starting)} starting line stripe(s)")
        for i, sl in enumerate(starting):
            issues.append(f"OK: StartingLine[{i}] {len(sl)} pts")

    # 3. Metadata completeness
    for key in ["image_path", "image_size", "pixels_per_meter",
                "threshold_used", "smoothing_factor"]:
        if key not in meta:
            issues.append(f"WARN: Metadata missing '{key}'")
    issues.append(
        f"INFO: threshold = {meta.get('threshold_used')}, "
        f"smoothing = {meta.get('smoothing_factor')}"
    )

    # 4. World coordinate range sanity
    if len(outer) > 0:
        max_x = max(p[0] for p in outer)
        max_y = max(p[1] for p in outer)
        expected_max = meta["image_size"][0] / meta["pixels_per_meter"]
        if max_x > expected_max * 1.1 or max_y > expected_max * 1.1:
            issues.append(
                f"WARN: Coordinates exceed expected range ({max_x:.1f}, {max_y:.1f}) "
                f"vs expected ~{expected_max:.1f}"
            )
        else:
            issues.append(f"OK: Coordinate range ({max_x:.1f}, {max_y:.1f}) within {expected_max:.1f}")

    return issues


def visualize(result: dict, save_path: str | None = None):
    """Plot boundaries with matplotlib."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available, skipping visualization")
        return

    fig, ax = plt.subplots(figsize=(10, 10))

    outer = result["outer_boundary"]
    if len(outer) > 0:
        ox, oy = zip(*outer)
        ax.plot(ox, oy, "b-", linewidth=1.5, label=f"Outer ({len(outer)} pts)")

    for i, hole in enumerate(result["holes"]):
        hx, hy = zip(*hole)
        ax.plot(hx, hy, "r-", linewidth=1.0, label=f"Hole {i} ({len(hole)} pts)")

    starting = result.get("starting_line", [])
    for i, sl in enumerate(starting):
        sx, sy = zip(*sl)
        ax.plot(sx, sy, "g-", linewidth=2.0, label=f"StartLine {i} ({len(sl)} pts)")

    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend()
    ax.set_title(f"map_parser: {os.path.basename(result['metadata']['image_path'])}")
    ax.set_xlim(-5, result["metadata"]["image_size"][0] /
                result["metadata"]["pixels_per_meter"] + 5)
    ax.set_ylim(result["metadata"]["image_size"][1] /
                result["metadata"]["pixels_per_meter"] + 5, -5)

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Plot saved: {save_path}")
    else:
        plt.show()


def main() -> int:
    ap = argparse.ArgumentParser(description="Verify map_parser module")
    ap.add_argument("--image", default="pipeline/map_parser/path2.png")
    ap.add_argument("--visualize", action="store_true",
                    help="Show matplotlib boundary plot")
    ap.add_argument("--save", default=None, help="Save JSON output to file")
    ap.add_argument("--save-plot", default=None, help="Save plot to file")
    ap.add_argument("--smoothing-factor", type=float, default=0.0)
    ap.add_argument("--pixels-per-meter", type=float, default=12.8)
    ap.add_argument("--has-starting-line", action=argparse.BooleanOptionalAction, default=True,
                    help="Enable starting line detection (default: on, use --no-has-starting-line to disable)")
    ap.add_argument("--max-starting-line-area", type=int, default=200,
                    help="Max pixel perimeter for starting line stripes")
    args = ap.parse_args()

    # Handle relative path: works when running from project root
    image_path = args.image
    if not os.path.isabs(image_path) and not os.path.exists(image_path):
        # Try relative to workspace root
        alt_path = os.path.join(_self_dir, "..", args.image)
        if os.path.exists(alt_path):
            image_path = os.path.normpath(alt_path)

    print(f"=== Testing map_parser: {image_path} ===")

    result = parse_map(
        image_path=image_path,
        pixels_per_meter=args.pixels_per_meter,
        smoothing_factor=args.smoothing_factor,
        has_starting_line=args.has_starting_line,
        max_starting_line_area=args.max_starting_line_area,
    )

    issues = validate_result(result)
    for issue in issues:
        prefix = "  "
        if issue.startswith("FAIL"):
            prefix = "  [FAIL] "
        elif issue.startswith("WARN"):
            prefix = "  [WARN] "
        elif issue.startswith("OK"):
            prefix = "  [OK]   "
        print(f"{prefix}{issue}")

    has_failures = any("FAIL" in i for i in issues)

    if args.save:
        payload = {
            "outer_boundary": result["outer_boundary"],
            "holes": result["holes"],
            "metadata": result["metadata"],
        }
        if "starting_line" in result:
            payload["starting_line"] = result["starting_line"]
        os.makedirs(os.path.dirname(os.path.abspath(args.save)) or ".", exist_ok=True)
        with open(args.save, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
        print(f"Saved: {args.save}")

    if args.visualize or args.save_plot:
        visualize(result, save_path=args.save_plot)

    print(f"\n{'[PASS] Test passed' if not has_failures else '[FAIL] Test failed'}")
    return 0 if not has_failures else 1


if __name__ == "__main__":
    sys.exit(main())
