"""
map_parser CLI — convenience command-line wrapper

Usage:
    python map_parser/cli.py path/to/image.jpg
    python map_parser/cli.py path/to/image.jpg -o result.json
    python map_parser/cli.py path/to/image.jpg --pixels-per-meter 10.0 --smoothing-factor 0.05
"""

from __future__ import annotations

import argparse
import json
import sys
import os

# Ensure map_parser is importable when run from pipeline/ directory
_self_dir = os.path.dirname(os.path.abspath(__file__))
_parent = os.path.dirname(_self_dir)
if _parent not in sys.path:
    sys.path.insert(0, _parent)

from map_parser import parse_map  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Extract track boundaries from a rendered map image (outer + holes)"
    )
    ap.add_argument("image", help="Path to input JPG/PNG image")
    ap.add_argument("-o", "--output", default=None,
                    help="Output JSON file path (default: stdout)")
    ap.add_argument("--pixels-per-meter", type=float, default=12.8,
                    help="Pixels-to-meters scale (default 12.8 = 1280px/100m)")
    ap.add_argument("--smoothing-factor", type=float, default=0.0,
                    help="Spline smoothing factor, 0=none, larger=smoother (default 0.0)")
    ap.add_argument("--num-control-points", type=int, default=200,
                    help="Number of spline control points (default 200)")
    ap.add_argument("--resample-spacing", type=float, default=None,
                    help="Output point arc-length spacing in meters (default: auto)")
    ap.add_argument("--threshold-method", default="otsu",
                    choices=["otsu", "adaptive", "manual"],
                    help="Binarization method (default: otsu)")
    ap.add_argument("--manual-threshold", type=int, default=None,
                    help="Manual threshold value 0-255 (for threshold_method=manual)")
    ap.add_argument("--min-contour-area", type=int, default=100,
                    help="Minimum contour pixel count (default 100)")
    args = ap.parse_args(argv)

    try:
        result = parse_map(
            image_path=args.image,
            pixels_per_meter=args.pixels_per_meter,
            smoothing_factor=args.smoothing_factor,
            num_control_points=args.num_control_points,
            resample_spacing_m=args.resample_spacing,
            threshold_method=args.threshold_method,
            manual_threshold=args.manual_threshold,
            min_contour_area=args.min_contour_area,
        )
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1

    payload = {
        "outer_boundary": result["outer_boundary"],
        "holes": result["holes"],
        "metadata": result["metadata"],
    }
    json_text = json.dumps(payload, indent=2, ensure_ascii=False)

    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(json_text)
            f.write("\n")
        print(f"Saved: {args.output}", file=sys.stderr)
    else:
        print(json_text)

    # Print summary stats to stderr
    meta = result["metadata"]
    outer_pts = len(result["outer_boundary"])
    n_holes = len(result["holes"])
    hole_pts = sum(len(h) for h in result["holes"])
    print(
        f"Outer: {outer_pts} pts, "
        f"{n_holes} hole(s) ({hole_pts} pts total), "
        f"threshold={meta['threshold_used']}",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
