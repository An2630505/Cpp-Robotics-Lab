#ifndef ___STRAIGHT_PATH_H___
#define ___STRAIGHT_PATH_H___

#include "Path.h"

class StraightPath : public Path {
public:
    /// 直线路径：起点 (start_x, start_y)，航向 psi（弧度）
    StraightPath(float start_x, float start_y, float psi);

    Eigen::VectorXd getState(float s) override;
    void findNearest(const Eigen::VectorXd &pos, float &s,
                     float &e_y, float &e_psi, float &kappa) override;
    std::string getRefString(float dt, float Vx) const override;

private:
    float start_x_, start_y_;
    float psi_;          // 路径航向（恒定）
    float nx_, ny_;      // 法向量（psi 左侧）
};

#endif
