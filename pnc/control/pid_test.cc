/**
 * PID 单元测试
 *
 * 测试位置式 PID 和增量式 PID 的基本行为:
 * - 比例控制: 误差响应方向正确
 * - 积分控制: 持续误差产生累积
 * - 限幅功能正常
 */
#include "pid.h"

#include <cmath>
#include <cassert>
#include <iostream>

static bool approx(double a, double b, double tol = 1e-6) {
    return std::abs(a - b) < tol;
}

int main() {
    std::cout << "=== PID 单元测试 ===" << std::endl;

    // ---- 1. 位置式 PID ----
    {
        PID pid(1);
        Eigen::VectorXd kp(1), ki(1), kd(1);
        kp << 2.0; ki << 0.0; kd << 0.0;  // 纯 P 控制

        Eigen::VectorXd min_out(1), max_out(1);
        min_out << -100.0; max_out << 100.0;
        pid.setParam(kp, ki, kd, min_out, max_out);

        Eigen::VectorXd target(1), current(1);
        target << 10.0; current << 0.0;

        Eigen::VectorXd u = pid.positionPID(target, current);
        // u = 2.0 * (10 - 0) = 20.0, 被内部限幅 [-10, 10] 截断为 10.0
        assert(approx(u(0), 10.0));
        std::cout << "  测试: 位置式 P 控制+限幅 通过 (u=" << u(0) << ")" << std::endl;
    }

    // ---- 2. 位置式 PID 含积分 ----
    {
        PID pid(1);
        Eigen::VectorXd kp(1), ki(1), kd(1);
        kp << 1.0; ki << 0.5; kd << 0.0;
        Eigen::VectorXd min_out(1), max_out(1);
        min_out << -100.0; max_out << 100.0;
        pid.setParam(kp, ki, kd, min_out, max_out);

        Eigen::VectorXd target(1), current(1);
        target << 5.0; current << 0.0;

        // 第一步: u = 1*5 + 0.5*5 = 7.5
        Eigen::VectorXd u1 = pid.positionPID(target, current);
        assert(approx(u1(0), 7.5));
        std::cout << "  测试: 位置式 PI 控制   通过 (u1=" << u1(0) << ")" << std::endl;

        // 第二步: 积分累加, u 应更大
        Eigen::VectorXd u2 = pid.positionPID(target, current);
        assert(u2(0) > u1(0));  // 积分累积导致输出增大
        std::cout << "  测试: 积分累积         通过 (u2=" << u2(0) << ")" << std::endl;
    }

    // ---- 3. 增量式 PID ----
    {
        PID pid(1);
        Eigen::VectorXd kp(1), ki(1), kd(1);
        kp << 1.0; ki << 0.1; kd << 0.0;
        Eigen::VectorXd min_out(1), max_out(1);
        min_out << -100.0; max_out << 100.0;
        pid.setParam(kp, ki, kd, min_out, max_out);

        Eigen::VectorXd target(1), current(1);
        target << 10.0; current << 0.0;

        Eigen::VectorXd u = pid.incrementalPID(target, current);
        // Δu = 1*(10-0) + 0.1*10 = 10 + 1 = 11
        assert(approx(u(0), 11.0));
        std::cout << "  测试: 增量式 PI 控制   通过 (u=" << u(0) << ")" << std::endl;
    }

    // ---- 4. 限幅 ----
    {
        PID pid(2);
        Eigen::VectorXd val(2), min_v(2), max_v(2);
        val << 15.0, -5.0;
        min_v << -10.0, -10.0;
        max_v << 10.0, 10.0;
        Eigen::VectorXd limited = pid.limit(val, min_v, max_v);
        assert(approx(limited(0), 10.0));
        assert(approx(limited(1), -5.0));
        std::cout << "  测试: 限幅              通过" << std::endl;
    }

    std::cout << "=== PID 测试全部通过 ===" << std::endl;
    return 0;
}
