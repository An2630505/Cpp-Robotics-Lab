#include "SlalomPath.h"
#include <cmath>
#include <sstream>

SlalomPath::SlalomPath(float A, float omega, float start_x, float start_y)
    : A_(A), omega_(omega), start_x_(start_x), start_y_(start_y)
{
}

Eigen::VectorXd SlalomPath::getState(float s)
{
    float x = start_x_ + s;
    float y = start_y_ + A_ * std::sin(omega_ * s);

    float dy = A_ * omega_ * std::cos(omega_ * s);
    float psi_ref = std::atan2(dy, 1.0f);

    // 曲率: kappa = (x'*y'' - y'*x'') / (x'^2 + y'^2)^(3/2)
    float ddy = -A_ * omega_ * omega_ * std::sin(omega_ * s);
    float denom = std::pow(1.0f + dy * dy, 1.5f);
    float kappa = ddy / denom;

    Eigen::VectorXd state(4);
    state << x, y, psi_ref, kappa;
    return state;
}

void SlalomPath::findNearest(const Eigen::VectorXd &pos, float &s,
                              float &e_y, float &e_psi, float &kappa)
{
    // pos = [x, y, psi]
    // 沿 x 方向投影作为弧长近似（A*omega 较小时精度足够）
    s = pos(0) - start_x_;
    if (s < 0.0f) s = 0.0f;

    // 路径点信息
    float y_path = start_y_ + A_ * std::sin(omega_ * s);
    float dy = A_ * omega_ * std::cos(omega_ * s);
    float psi_ref = std::atan2(dy, 1.0f);

    // 侧向误差: 法向量 n = [-sin(psi_ref), cos(psi_ref)]
    float dx_pos = pos(0) - (start_x_ + s);
    float dy_pos = pos(1) - y_path;
    e_y = dx_pos * (-std::sin(psi_ref)) + dy_pos * std::cos(psi_ref);

    // 航向误差
    e_psi = pos(2) - psi_ref;
    e_psi = std::fmod(e_psi + M_PI, 2.0f * M_PI);
    if (e_psi < 0) e_psi += 2.0f * M_PI;
    e_psi -= M_PI;

    // 曲率
    float ddy = -A_ * omega_ * omega_ * std::sin(omega_ * s);
    float denom = std::pow(1.0f + dy * dy, 1.5f);
    kappa = ddy / denom;
}

std::string SlalomPath::getRefString(float dt, float Vx) const
{
    std::ostringstream oss;
    oss << "type=slalom dt=" << dt << " Vx=" << Vx
        << " A=" << A_ << " omega=" << omega_
        << " start_x=" << start_x_ << " start_y=" << start_y_;
    return oss.str();
}
