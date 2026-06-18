# Map Parser — 赛道几何边界提取

从赛道渲染图（JPG/PNG）中提取几何边界，输出外边界 + 所有内部孔洞/岛屿轮廓。

## 设计决策

| 维度 | 决策 | 理由 |
|------|------|------|
| 输入 | JPG/PNG 渲染图 | 仿真场景的视觉表示 |
| 赛道识别 | 亮色区域（vs 暗色背景） | Otsu 自适应阈值分离 |
| 输出结构 | 外边界 + N 条孔洞轮廓 | 不预设孔洞数量，忠实反映地图 |
| 方向性 | 无向闭合多边形 | 纯几何描述，方向留给下游 |
| 坐标系统 | 世界坐标（米） | `pixels_per_meter` 参数可配置 |
| 平滑方式 | cubic periodic spline (`splprep per=1 k=3`) | C2 连续闭合，消除像素锯齿 |
| 输出格式 | 单个 JSON 文件 | 结构化存储，数量不固定的轮廓天然适合 |
| 使用形态 | Python importable 模块 | 可被其他 pipeline 脚本直接调用 |

## 处理管线

```
加载图像 → 灰度化 → 二值化(Otsu) → 轮廓提取(RETR_CCOMP) → 分类(外/孔洞) → 世界坐标转换 → 样条平滑 → JSON
```

### 为什么选 RETR_CCOMP

`cv2.RETR_CCOMP` 将轮廓组织为两级层级：level 0 = 外边界，level 1 = 孔洞。这直接映射到需求模型"1 条外边界 + N 个孔洞"。`RETR_TREE` 会给出完整嵌套树，对本场景是过度设计。

### 为什么选 splprep(per=1)

`scipy.interpolate.splprep` 配合 `per=1`（periodic boundary conditions）自动保证闭合曲线的 C2 连续性——起点和终点的位置、一阶导、二阶导都匹配。零额外代码。

## 输出结构

```json
{
  "outer_boundary": [[x1, y1], [x2, y2], ...],
  "holes": [
    [[x1, y1], ...],
    [[x1, y1], ...]
  ],
  "metadata": {
    "image_path": "path/to/track.jpg",
    "image_size": [1280, 1280],
    "pixels_per_meter": 12.8,
    "num_outer_contours_found": 1,
    "num_holes_found": 2,
    "threshold_used": 139,
    "smoothing_factor": 0.0
  }
}
```

## API

```python
from map_parser import parse_map

result = parse_map(
    image_path="track.jpg",
    pixels_per_meter=12.8,       # 像素→米的缩放比例
    smoothing_factor=0.0,       # 样条平滑系数，0=无平滑
    num_control_points=200,      # splprep 控制点数量
    resample_spacing_m=None,     # 弧长重采样间距，None=自动
    threshold_method="otsu",     # 二值化方法：otsu/adaptive/manual
    manual_threshold=None,       # manual 方法的 0-255 阈值
    min_contour_area=100,        # 最小轮廓像素数（噪声过滤）
)
```

## 依赖

- `opencv-python` — 图像加载、二值化、轮廓提取
- `scipy` — 样条平滑 (`splprep` / `splev`)
- `numpy` — 数组运算

## 边界情况

| 情况 | 策略 |
|------|------|
| 图像无法读取 | `FileNotFoundError` |
| 无轮廓（纯色图） | 返回空 `outer_boundary` + 空 `holes` |
| 轮廓 < 4 点 | 跳过（样条至少需要 4 点） |
| scipy 未安装 | 清晰的 ImportError 提示 |
| 多外边界 | 取周长最大的作为主外边界 |

---

> 实现于 `pipeline/map_parser/` ，作为独立 Python 包。
