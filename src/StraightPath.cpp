#include "StraightPath.h"
#include <cmath>
#include <sstream>

StraightPath::StraightPath(float start_x, float start_y, float psi)
    : start_x_(start_x), start_y_(start_y), psi_(psi)
{
    nx_ = -std::sin(psi);
    ny_ = std::cos(psi);
}

Eigen::VectorXd StraightPath::getState(float s)
{
    float x = start_x_ + s * std::cos(psi_);
    float y = start_y_ + s * std::sin(psi_);

    Eigen::VectorXd state(4);
    state << x, y, psi_, 0.0f;  // 直道曲率为 0
    return state;
}

void StraightPath::findNearest(const Eigen::VectorXd &pos, float &s,
                                float &e_y, float &e_psi, float &kappa)
{
    float dx = pos(0) - start_x_;
    float dy = pos(1) - start_y_;

    // 沿方向投影 = 弧长
    s = dx * std::cos(psi_) + dy * std::sin(psi_);
    if (s < 0.0f) s = 0.0f;

    // 法向投影 = 侧向误差（正 = 左侧）
    e_y = dx * nx_ + dy * ny_;

    // 航向误差
    e_psi = pos(2) - psi_;
    e_psi = std::fmod(e_psi + M_PI, 2.0f * M_PI);
    if (e_psi < 0) e_psi += 2.0f * M_PI;
    e_psi -= M_PI;

    kappa = 0.0f;
}

std::string StraightPath::getRefString(float dt, float Vx) const
{
    std::ostringstream oss;
    oss << "type=straight dt=" << dt << " Vx=" << Vx
        << " start_x=" << start_x_ << " start_y=" << start_y_
        << " psi=" << psi_;
    return oss.str();
}
