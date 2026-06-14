/**
 * KF 单元测试 — 卡尔曼滤波器
 *
 * 测试思路: 构造一个简单的恒速运动模型, 加入已知噪声,
 * 验证 KF 估计值比原始测量值更接近真实状态。
 */
#include "kf.h"

#include <cmath>
#include <cassert>
#include <iostream>
#include <random>

static bool approx(double a, double b, double tol = 0.1) {
    return std::abs(a - b) < tol;
}

int main() {
    std::cout << "=== KF 单元测试 ===" << std::endl;

    // ---- 恒速运动模型 (1D 位置+速度) ----
    double dt = 0.1;
    Eigen::MatrixXd A(2, 2);
    A << 1, dt, 0, 1;

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(2, 1);

    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(2, 2);

    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2, 2) * 0.01;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2) * 0.1;

    Eigen::VectorXd x0(2);
    x0 << 0.0, 1.0;  // 初始位置 0, 速度 1

    KF kf;
    kf.init(A, B, C, P, Q, R, x0);

    // ---- 模拟运动 + 测量 ----
    Eigen::VectorXd x_true = x0;
    Eigen::VectorXd u(1);
    u << 0.0;

    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, 0.3);

    int passed = 0;
    for (int step = 0; step < 50; step++) {
        // 真实状态更新 (恒速)
        x_true = A * x_true;

        // 带噪声的测量
        Eigen::VectorXd z = x_true;
        z(0) += noise(rng);
        z(1) += noise(rng);

        // KF 更新
        kf.update(z, u);

        // KF 估计应比测量更接近真实值
        double err_meas = (z - x_true).norm();
        double err_kf = (kf.x_post - x_true).norm();
        if (err_kf < err_meas) passed++;
    }

    std::cout << "  测试: KF 优于测量  " << passed << "/50 步" << std::endl;
    assert(passed >= 30);  // KF 在大多数步应优于裸测量

    // ---- 最终估计应接近真实 ----
    assert(approx(kf.x_post(0), x_true(0), 0.5));
    assert(approx(kf.x_post(1), x_true(1), 0.5));
    std::cout << "  测试: 最终估计收敛  通过" << std::endl;

    std::cout << "=== KF 测试全部通过 ===" << std::endl;
    return 0;
}
