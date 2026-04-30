#include "CirclePath.h"
#include <cmath>
#include <sstream>

CirclePath::CirclePath(float cx, float cy, float R, float start_s)
    : cx_(cx), cy_(cy), R_(R)
{
    // 从起始弧长计算起始角度（逆时针为正）
    theta0_ = -M_PI / 2.0f + start_s / R_;
}

Eigen::VectorXd CirclePath::getState(float s)
{
    float theta = theta0_ + s / R_;
    float x = cx_ + R_ * std::cos(theta);
    float y = cy_ + R_ * std::sin(theta);
    // 切线方向（逆时针圆周运动）
    float psi_ref = std::fmod(theta + M_PI / 2.0f, 2.0f * M_PI);
    float kappa = 1.0f / R_;

    Eigen::VectorXd state(4);
    state << x, y, psi_ref, kappa;
    return state;
}

void CirclePath::findNearest(const Eigen::VectorXd &pos, float &s,
                              float &e_y, float &e_psi, float &kappa)
{
    // pos = [x, y, psi]
    float dx = pos(0) - cx_;
    float dy = pos(1) - cy_;
    float d = std::sqrt(dx * dx + dy * dy);

    float theta = std::atan2(dy, dx);

    // 从起始角度的弧长增量（归一化到 [-pi, pi]）
    float da = theta - theta0_;
    da = std::fmod(da + M_PI, 2.0f * M_PI);
    if (da < 0) da += 2.0f * M_PI;
    da -= M_PI;
    s = R_ * da;

    // 侧向误差（正 = 圆外侧）
    e_y = d - R_;

    // 路径航向
    float psi_ref = theta + M_PI / 2.0f;
    // 航向误差
    float psi_vehicle = pos(2);
    e_psi = psi_vehicle - psi_ref;
    // 归一化到 [-pi, pi]
    e_psi = std::fmod(e_psi + M_PI, 2.0f * M_PI);
    if (e_psi < 0) e_psi += 2.0f * M_PI;
    e_psi -= M_PI;

    kappa = 1.0f / R_;
}

std::string CirclePath::getRefString(float dt, float Vx) const
{
    float w_cur = Vx / R_;
    float x0 = cx_ + R_ * std::cos(theta0_);
    float y0 = cy_ + R_ * std::sin(theta0_);
    float psi0 = theta0_ + (float)M_PI / 2.0f;
    float vx0 = Vx * std::cos(psi0);
    float vy0 = Vx * std::sin(psi0);

    std::ostringstream oss;
    oss << "type=circle w=" << w_cur << " dt=" << dt
        << " Vx=" << Vx << " R=" << R_
        << " x0=" << x0 << " y0=" << y0
        << " vx0=" << vx0 << " vy0=" << vy0;
    return oss.str();
}
