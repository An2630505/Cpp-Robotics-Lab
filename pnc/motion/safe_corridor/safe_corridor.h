#ifndef PNC_MOTION_SAFE_CORRIDOR_H_
#define PNC_MOTION_SAFE_CORRIDOR_H_

#include <vector>
#include "../../common/types.h"

/// 安全走廊构建器 — 沿参考路径逐 cell 扫描占用栅格, 生成凸约束管道
class SafeCorridor {
public:
    SafeCorridor();

    void setMargin(double m)          { margin_ = m; }
    void setSampleInterval(double ds) { sample_interval_ = ds; }

    /// 从参考路径和占用栅格构建安全走廊 (沿法向逐 cell 扫描)
    /// @param ref_path  参考路径 (连续位姿序列)
    /// @param grid      占用栅格 (0=自由, 1=障碍物)
    /// @param x_min,y_min  栅格原点世界坐标
    /// @param cell_size 栅格分辨率
    /// @param cols,rows 栅格尺寸
    /// @return          安全走廊截面序列
    std::vector<CorridorSection> build(
        const std::vector<Pose>& ref_path,
        const std::vector<std::vector<int>>& grid,
        double x_min, double y_min,
        double cell_size, int cols, int rows);

private:
    double margin_ = 0.5;
    double sample_interval_ = 2.0;
};

#endif  // PNC_MOTION_SAFE_CORRIDOR_H_
