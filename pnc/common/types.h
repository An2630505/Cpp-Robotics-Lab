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

#endif  // PNC_COMMON_TYPES_H_
