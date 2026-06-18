"""
map_parser — 赛道几何边界提取模块

用法:
    from map_parser import parse_map

    result = parse_map("path/to/track.jpg")
    print(len(result["outer_boundary"]), "个外边界点")
    print(len(result["holes"]), "个孔洞")
"""

from ._core import parse_map

__all__ = ["parse_map"]
