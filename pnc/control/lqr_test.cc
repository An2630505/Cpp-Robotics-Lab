/**
 * LQR 单元测试
 *
 * 测试离散 LQR 控制律求解与运行:
 * - 求解 Riccati 方程得到反馈增益 K
 * - run() 输出控制量使状态向目标收敛
 */
#include "lqr.h"

#include <cmath>
#include <cassert>
#include <iostream>

static bool approx(double a, double b, double tol = 1e-4) {
    return std::abs(a - b) < tol;
}

int main() {
    std::cout << "=== LQR 单元测试 ===" << std::endl;

    // ---- 二阶系统 (位置+速度) ----
    double dt = 0.1;
    Eigen::MatrixXd A(2, 2);
    A << 1, dt, 0, 1;

    Eigen::MatrixXd B(2, 1);
    B << 0.5 * dt * dt, dt;  // 加速度输入

    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(2, 2);

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2, 2);
    Q(0, 0) = 10.0;  // 位置权重高
    Q(1, 1) = 1.0;

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1) * 0.1;
    Eigen::MatrixXd S = Eigen::MatrixXd::Identity(2, 2);

    LQR lqr;
    lqr.Init(A, B, C, Q, R, S);

    // ---- 初始状态: 位置=5, 速度=0, 目标: 位置=0, 速度=0 ----
    Eigen::VectorXd x(2);
    x << 5.0, 0.0;
    Eigen::VectorXd y_ref(2);
    y_ref << 0.0, 0.0;

    // 运行几步, 状态应朝零移动
    for (int step = 0; step < 20; step++) {
        Eigen::VectorXd u = lqr.run(y_ref, x);
        x = A * x + B * u;
    }

    std::cout << "  最终状态: x=[" << x(0) << ", " << x(1) << "]" << std::endl;
    // 经过 20 步, 位置应显著减小
    assert(std::abs(x(0)) < 1.0);
    std::cout << "  测试: LQR 镇定          通过" << std::endl;

    std::cout << "=== LQR 测试全部通过 ===" << std::endl;
    return 0;
}
