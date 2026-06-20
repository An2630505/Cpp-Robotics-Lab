#ifndef PNC_MOTION_BSPLINE_H_
#define PNC_MOTION_BSPLINE_H_

#include <vector>
#include <Eigen/Dense>
#include "../../common/types.h"

/// B 样条拟合参数
struct BSplineParams {
    int degree = 3;
    int num_control_points = 50;
    bool closed = true;
    double resample_spacing = 0.5;
};

/// B 样条曲线拟合 & 等弧长重采样
class BSpline {
public:
    BSpline();
    void setParams(const BSplineParams& p);
    const BSplineParams& getParams() const { return params_; }

    /// 拟合 B 样条到参考路径, 受走廊约束
    /// @param ref_path   参考路径 (HNode 输出)
    /// @param corridors  安全走廊约束
    /// @return           平滑后的路径 (含朝向)
    std::vector<Pose> fit(
        const std::vector<Pose>& ref_path,
        const std::vector<CorridorSection>& corridors);

    /// 等弧长重采样
    std::vector<Pose> resample(const std::vector<Pose>& path);

private:
    BSplineParams params_;

    /// Cox-de Boor 递推基函数
    double basis(int i, int k, double t,
                 const std::vector<double>& knots) const;

    /// 在参数 t 处评估 B 样条 (返回 x,y)
    Eigen::Vector2d eval(double t, const std::vector<double>& knots,
                          const Eigen::MatrixX2d& ctrl_pts) const;

    /// 构建周期 knot vector (closed)
    std::vector<double> periodicKnots(int n_ctrl, int degree) const;

    /// 点到线段最近点的投影
    static Vec2d projectToSegment(double px, double py,
                                   double ax, double ay,
                                   double bx, double by);
};

#endif  // PNC_MOTION_BSPLINE_H_
