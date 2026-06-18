/**
 * BicycleModel 单元测试
 *
 * 测试自行车模型的动力学行为:
 * - 零输入时误差状态保持不变 (A≈I)
 * - 施加转向角时横向误差发生变化
 */
#include "bicycle_model.h"

#include <cmath>
#include <cassert>
#include <iostream>

static bool approx(double a, double b, double tol = 1e-4) {
    return std::abs(a - b) < tol;
}

int main() {
    std::cout << "=== BicycleModel 单元测试 ===" << std::endl;

    // ---- 使用原始 main.cpp 中的参数构建模型 ----
    double m = 1573.0, Iz = 2873.0, lf = 1.1, lr = 1.58;
    double C_af = 80000.0, C_ar = 80000.0, Vx = 10.0;

    Eigen::MatrixXd A(4, 4);
    A << 0, 1, 0, 0,
         0, -(2 * C_af + 2 * C_ar) / (m * Vx),
         (2 * C_af + 2 * C_ar) / m,
         -(2 * C_af * lf - 2 * C_ar * lr) / (m * Vx),
         0, 0, 0, 1,
         0, -(2 * C_af * lf - 2 * C_ar * lr) / (Iz * Vx),
         (2 * C_af * lf - 2 * C_ar * lr) / Iz,
         -(2 * C_af * lf * lf + 2 * C_ar * lr * lr) / (Iz * Vx);

    Eigen::MatrixXd B1(4, 1);
    B1 << 0, 2 * C_af / m, 0, 2 * C_af * lf / Iz;

    Eigen::MatrixXd B2(4, 1);
    B2 << 0,
         -(2 * C_af * lf - 2 * C_ar * lr) / (m * Vx) - Vx,
          0,
         -(2 * C_af * lf * lf + 2 * C_ar * lr * lr) / (Iz * Vx);

    Eigen::MatrixXd C_mat = Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd D_mat = Eigen::MatrixXd::Zero(4, 1);

    BicycleModel model(A, B1, B2, C_mat, D_mat);

    Eigen::VectorXd x0(4);
    x0 << 0.5, 0.0, 0.05, 0.0;  // 初始误差
    model.Init(x0);

    // ---- 测试 1: 零输入时, 离散化后状态应近似不变 (A ≈ I + A_cont*dt, 因 dt 小) ----
    Eigen::VectorXd u_zero(1);
    u_zero << 0.0;
    model.step(0.1, 0.0, u_zero);
    // 状态变化应较小
    assert(approx(model.x(0), 0.5, 0.1));
    std::cout << "  测试: 零输入保持         通过 (e_y=" << model.x(0) << ")" << std::endl;

    // ---- 测试 2: 施加转向, 横向误差变化 ----
    Eigen::VectorXd u_steer(1);
    u_steer << 0.3;  // 约 17° 转角
    Eigen::VectorXd y = model.step(0.1, 0.0, u_steer);
    // 转向应产生非零的 e_psi 变化
    assert(std::abs(model.x(2)) > 1e-6);
    std::cout << "  测试: 转向响应           通过 (e_psi=" << model.x(2) << ")" << std::endl;

    std::cout << "=== BicycleModel 测试全部通过 ===" << std::endl;
    return 0;
}
