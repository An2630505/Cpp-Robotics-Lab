#ifndef PNC_COMMON_TYPES_H_
#define PNC_COMMON_TYPES_H_

#include <vector>
#include <cstddef>

/// 离散网格坐标 (行, 列)
struct Point {
    int row, col;
    bool operator==(const Point& o) const { return row == o.row && col == o.col; }
};

/// 连续世界位姿 (x, y, θ)
struct Pose {
    double x, y, theta;
};

/// 占用网格数据
struct GridData {
    std::vector<std::vector<int>> grid;
    Point start, goal;
    int size;
};

/// 二维向量 (连续世界坐标)
struct Vec2d {
    double x, y;
};

/// 安全走廊截面
struct CorridorSection {
    Vec2d center;  // 参考路径上的采样点
    Vec2d left;    // 左边界点 (世界坐标)
    Vec2d right;   // 右边界点 (世界坐标)
};

#endif  // PNC_COMMON_TYPES_H_
