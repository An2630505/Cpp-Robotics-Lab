#ifndef PNC_MOTION_SAFE_CORRIDOR_H_
#define PNC_MOTION_SAFE_CORRIDOR_H_

#include <vector>
#include "../../common/types.h"

/// 安全走廊构建器 — 沿参考路径生成凸约束管道
class SafeCorridor {
public:
    SafeCorridor();

    void setMargin(double m)          { margin_ = m; }
    void setSampleInterval(double ds) { sample_interval_ = ds; }

    /// 从参考路径和多边形边界构建安全走廊
    /// @param ref_path  参考路径 (连续位姿序列)
    /// @param outer     外边界多边形顶点 (需闭合, 逆时针)
    /// @param holes     孔洞多边形列表 (顺时针)
    /// @return          安全走廊截面序列
    std::vector<CorridorSection> build(
        const std::vector<Pose>& ref_path,
        const std::vector<Vec2d>& outer,
        const std::vector<std::vector<Vec2d>>& holes);

private:
    double margin_ = 0.5;
    double sample_interval_ = 2.0;

    /// 射线-线段交点: 射线 orig + t*dir (t>=0) 与线段 a→b
    static bool raySegIntersect(const Vec2d& orig, const Vec2d& dir,
                                 const Vec2d& a, const Vec2d& b,
                                 double& t);
    /// 射线与多边形的最短交距 (正方向)
    static double rayPolygonDist(const Vec2d& orig, const Vec2d& dir,
                                  const std::vector<Vec2d>& poly);
};

#endif  // PNC_MOTION_SAFE_CORRIDOR_H_
