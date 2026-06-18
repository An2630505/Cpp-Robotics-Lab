/**
 * Path 单元测试
 *
 * 测试路径生成功能: 直道、圆弧、S弯的组合
 */
#include "path.h"

#include <cmath>
#include <cassert>
#include <iostream>

static bool approx(double a, double b, double tol = 1e-4) {
    return std::abs(a - b) < tol;
}

int main() {
    std::cout << "=== Path 单元测试 ===" << std::endl;

    // ---- 直道 ----
    Path path;
    path.addStraight(10.0f);
    path.build();

    {
        Eigen::VectorXd s = path.getState(5.0f);
        assert(approx(s(0), 5.0));
        assert(approx(s(1), 0.0));
        assert(approx(s(2), 0.0));
        assert(approx(s(3), 0.0));  // kappa=0
        std::cout << "  测试: 直道路径           通过" << std::endl;
    }

    // ---- 圆弧 ----
    {
        Path arc_path;
        arc_path.addArc(M_PI * 10.0f, 10.0f);  // 半圆
        arc_path.build();

        Eigen::VectorXd mid = arc_path.getState(M_PI * 5.0f);  // 1/4 圆
        // 左转, 1/4 圆后应在 (10, 10), 航向 90°
        assert(approx(mid(0), 10.0, 0.5));
        assert(approx(mid(1), 10.0, 0.5));
        std::cout << "  测试: 圆弧路径           通过 (x=" << mid(0) << ", y=" << mid(1) << ")" << std::endl;
    }

    // ---- 组合路径 ----
    {
        Path combo;
        combo.addStraight(10.0f);
        combo.addArc(15.7f, 10.0f);  // 左转 90°
        combo.build();

        float len = combo.totalLength();
        assert(len > 25.0f);

        Eigen::VectorXd end = combo.getState(len);
        // 路径末端应远离原点
        assert(end(0) > 5.0);
        std::cout << "  测试: 组合路径           通过 (总长=" << len << ")" << std::endl;
    }

    std::cout << "=== Path 测试全部通过 ===" << std::endl;
    return 0;
}
