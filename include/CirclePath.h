#ifndef ___CIRCLE_PATH_H___
#define ___CIRCLE_PATH_H___

#include "Path.h"

class CirclePath : public Path {
public:
    // cx, cy: 圆心；R: 半径；start_s: 起始弧长
    CirclePath(float cx, float cy, float R, float start_s = 0.0f);

    Eigen::VectorXd getState(float s) override;
    void findNearest(const Eigen::VectorXd &pos, float &s,
                     float &e_y, float &e_psi, float &kappa) override;
    std::string getRefString(float dt, float Vx) const override;

private:
    float cx_, cy_;   // 圆心坐标
    float R_;         // 半径
    float theta0_;    // 起始角度（对应 s=0）
};

#endif
