"""
centerline CLI — convenience command-line wrapper

Usage:
    python centerline/cli.py path/to/boundaries.json
    python centerline/cli.py path/to/boundaries.json -o graph.json
"""

from __future__ import annotations

import argparse
import json
import sys
import os

_self_dir = os.path.dirname(os.path.abspath(__file__))
_parent = os.path.dirname(_self_dir)
if _parent not in sys.path:
    sys.path.insert(0, _parent)

from centerline import extract_centerline_graph


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Extract centerline graph from track boundaries"
    )
    ap.add_argument("boundaries", help="Path to boundaries JSON (map_parser output)")
    ap.add_argument("-o", "--output", default=None,
                    help="Output JSON file path (default: stdout)")
    ap.add_argument("--pixels-per-meter", type=float, default=12.8)
    ap.add_argument("--smoothing-factor", type=float, default=0.02,
                    help="Spline smoothing factor (default 0.02)")
    ap.add_argument("--resample-spacing", type=float, default=None,
                    help="Edge point spacing in meters (default: auto)")
    ap.add_argument("--render-resolution", type=int, default=1024,
                    help="Render canvas max size (default 1024)")
    ap.add_argument("--prune-spur-length", type=float, default=2.0,
                    help="Spur prune threshold in meters (default 2.0)")
    args = ap.parse_args(argv)

    try:
        with open(args.boundaries) as f:
            boundaries = json.load(f)
    except Exception as e:
        print(f"Error reading boundaries file: {e}", file=sys.stderr)
        return 1

    outer = boundaries.get("outer_boundary")
    holes = boundaries.get("holes")
    if outer is None:
        print("Error: JSON must contain 'outer_boundary'", file=sys.stderr)
        return 1

    try:
        graph = extract_centerline_graph(
            outer_boundary=outer,
            holes=holes or [],
            pixels_per_meter=args.pixels_per_meter,
            smoothing_factor=args.smoothing_factor,
            resample_spacing_m=args.resample_spacing,
            render_resolution=args.render_resolution,
            prune_spur_length_m=args.prune_spur_length,
        )
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1

    json_text = json.dumps(graph, indent=2, ensure_ascii=False)

    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(json_text)
            f.write("\n")
        print(f"Saved: {args.output}", file=sys.stderr)
    else:
        print(json_text)

    meta = graph["metadata"]
    total_len = sum(e["length_m"] for e in graph["edges"])
    print(
        f"Nodes: {meta['num_nodes']}, "
        f"Edges: {meta['num_edges']}, "
        f"Total length: {total_len:.1f}m",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
