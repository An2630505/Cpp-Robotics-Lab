# map_parser — 赛道几何边界提取

从赛道渲染图中提取外边界 + 孔洞/障碍物 + 起跑线标记。

## 快速开始

```python
import sys; sys.path.insert(0, 'pipeline')
from map_parser import parse_map

# 解析普通赛道图片
result = parse_map("pipeline/map_parser/path1.jpg")

# 查看结果
print(f"外边界: {len(result['outer_boundary'])} 个点")
print(f"孔洞:   {len(result['holes'])} 个")
print(f"元信息: {result['metadata']}")

# 带起跑线的赛道
result2 = parse_map("pipeline/map_parser/path2.png", has_starting_line=True)
print(f"起跑线: {len(result2['starting_line'])} 条")

# 保存为 JSON
import json
with open("output/boundaries.json", "w") as f:
    json.dump(result, f, indent=2)
```

## CLI 用法

```bash
# 输出 JSON 到 stdout
python pipeline/map_parser/cli.py pipeline/map_parser/path1.jpg

# 保存到文件
python pipeline/map_parser/cli.py pipeline/map_parser/path1.jpg -o output/boundaries.json

# 调整参数
python pipeline/map_parser/cli.py track.jpg --pixels-per-meter 10.0 --smoothing-factor 0.05 --threshold-method adaptive

# 带起跑线的赛道
python pipeline/map_parser/cli.py pipeline/map_parser/path2.png --has-starting-line
```

## API 参考

### `parse_map()`

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `image_path` | `str` | (必需) | JPG/PNG 图像路径 |
| `pixels_per_meter` | `float` | `12.8` | 像素→世界米 缩放比例 |
| `smoothing_factor` | `float` | `0.0` | 样条平滑系数，0=无平滑，越大越平滑 |
| `num_control_points` | `int` | `200` | splprep 控制点数量 |
| `resample_spacing_m` | `float \| None` | `None` | 输出点弧长间距(m)，None=自动 |
| `threshold_method` | `str` | `"otsu"` | `"otsu"` / `"adaptive"` / `"manual"` |
| `manual_threshold` | `int \| None` | `None` | manual 方法时的 0-255 阈值 |
| `min_contour_area` | `int` | `100` | 最小轮廓像素数（噪声过滤） |
| `has_starting_line` | `bool` | `False` | 赛道内部有起跑线（黑白短横线）时设为 True |
| `max_starting_line_area` | `int` | `200` | 起跑线条纹最大像素周长；≤此值=起跑线，>此值=障碍物 |

### 返回值

```python
{
    "outer_boundary": [[float, float], ...],   # 外边界点序列
    "holes": [[[float, float], ...], ...],     # 孔洞/障碍物列表
    "starting_line": [[[float, float], ...]],  # 起跑线列表（仅 has_starting_line=True）
    "metadata": {
        "image_path": str,
        "image_size": [int, int],      # [width, height] 像素
        "pixels_per_meter": float,
        "num_outer_contours_found": int,
        "num_holes_found": int,
        "threshold_used": int,          # 实际使用的二值化阈值
        "smoothing_factor": float,
    }
}
```

### 起跑线检测

赛道上有黑白相间的短横线（起跑线）时，启用 `has_starting_line=True`。
解析器会按 `max_starting_line_area`（默认 200 px）将孔洞分类：

| 分类 | 条件 | 处理方式 |
|------|------|----------|
| `"holes"` | 孔洞周长 > `max_starting_line_area` | 正常样条平滑 |
| `"starting_line"` | 孔洞周长 ≤ `max_starting_line_area` | 保留原始像素坐标，不做平滑 |

## 验证测试

```bash
# 基础测试（无起跑线）
python pipeline/test_map_parser.py

# 起跑线赛道测试
python pipeline/test_map_parser.py --image pipeline/map_parser/path2.png --has-starting-line

# 含可视化
python pipeline/test_map_parser.py --visualize

# 保存输出
python pipeline/test_map_parser.py --save output/test_result.json --save-plot output/test_plot.png
```

## 依赖

```bash
pip install opencv-python scipy numpy
```
