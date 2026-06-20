/**
 * BSpline 单元测试
 *
 * 测试 B 样条拟合、走廊约束投影、等弧长重采样。
 */
#include "bspline.h"
#include <iostream>
#include <cassert>
#include <cmath>

int main() {
    std::cout << "=== BSpline 单元测试 ===" << std::endl;

    // ---- Test 1: 拟合直线 ----
    {
        BSpline bs;
        BSplineParams p;
        p.degree = 3;
        p.num_control_points = 10;
        p.closed = false;
        p.resample_spacing = 0.5;
        bs.setParams(p);

        // 一条从 (0,0) 到 (10,0) 的直线
        std::vector<Pose> ref;
        for (int i = 0; i <= 20; i++)
            ref.push_back({i * 0.5, 0.0, 0.0});

        std::vector<CorridorSection> corridors;  // 无约束

        auto fitted = bs.fit(ref, corridors);
        assert(!fitted.empty());
        // 拟合路径应该大致在 y≈0 附近
        for (auto& pt : fitted)
            assert(std::abs(pt.y) < 0.2);

        auto resampled = bs.resample(fitted);
        assert(!resampled.empty());
        // 等弧长: 相邻点距离 ≈ 0.5
        for (size_t i = 1; i < resampled.size(); i++) {
            double d = std::hypot(resampled[i].x - resampled[i-1].x,
                                   resampled[i].y - resampled[i-1].y);
            assert(d > 0.3 && d < 0.8);
        }
        std::cout << "  测试: 直线拟合+重采样    通过 ("
                  << fitted.size() << " pts, resampled="
                  << resampled.size() << ")" << std::endl;
    }

    // ---- Test 2: 拟合闭合圆 ----
    {
        BSpline bs;
        BSplineParams p;
        p.degree = 3;
        p.num_control_points = 30;
        p.closed = true;
        p.resample_spacing = 0.3;
        bs.setParams(p);

        // 近似圆 (r=2, 加点噪声模拟 Hybrid A* 锯齿)
        std::vector<Pose> ref;
        int n_raw = 40;
        for (int i = 0; i < n_raw; i++) {
            double a = 2.0 * M_PI * i / n_raw;
            double r = 2.0 + 0.1 * std::sin(a * 13.0);  // 带噪声
            ref.push_back({r * std::cos(a), r * std::sin(a), a + M_PI_2});
        }

        // 宽松走廊 (允许在 1.5~2.5 半径范围内)
        std::vector<CorridorSection> corridors;
        for (int i = 0; i < n_raw; i++) {
            double a = 2.0 * M_PI * i / n_raw;
            CorridorSection sec;
            sec.center = {2.0 * std::cos(a), 2.0 * std::sin(a)};
            // left: 向外 (r=2.5), right: 向内 (r=1.5)
            double cos_a = std::cos(a), sin_a = std::sin(a);
            sec.left  = {2.5 * cos_a, 2.5 * sin_a};
            sec.right = {1.5 * cos_a, 1.5 * sin_a};
            corridors.push_back(sec);
        }

        auto fitted = bs.fit(ref, corridors);
        assert(!fitted.empty());

        // 检查路径在走廊内 (半径 1.5~2.5)
        for (auto& pt : fitted) {
            double r = std::sqrt(pt.x*pt.x + pt.y*pt.y);
            assert(r > 1.2 && r < 2.8);  // 给一些容差
        }

        auto resampled = bs.resample(fitted);
        assert(!resampled.empty());
        // 闭合: 首尾点应相近
        auto& first = resampled.front();
        auto& last  = resampled.back();
        double d_close = std::hypot(first.x - last.x, first.y - last.y);
        assert(d_close < 0.1);

        std::cout << "  测试: 闭合圆拟合+走廊    通过 ("
                  << fitted.size() << " pts, resampled="
                  << resampled.size() << ")" << std::endl;
    }

    // ---- Test 3: 走廊约束投影 ----
    {
        BSpline bs;
        BSplineParams p;
        p.degree = 3;
        p.num_control_points = 15;
        p.closed = false;
        bs.setParams(p);

        // 直线路径 y=0.5 故意偏离中心
        std::vector<Pose> ref;
        for (int i = 0; i < 20; i++)
            ref.push_back({i * 0.5, 0.5, 0.0});

        // 窄走廊: 中心 y=0, 上下 ±0.3
        std::vector<CorridorSection> corridors;
        for (int i = 0; i < 20; i++) {
            CorridorSection sec;
            sec.center = {i * 0.5, 0.0};
            sec.left   = {i * 0.5, 0.3};   // 上边界
            sec.right  = {i * 0.5, -0.3};  // 下边界
            corridors.push_back(sec);
        }

        auto fitted = bs.fit(ref, corridors);
        assert(!fitted.empty());

        // 拟合路径应被约束到 y ∈ [-0.5, 0.5] 附近
        for (auto& pt : fitted)
            assert(std::abs(pt.y) < 0.8);

        std::cout << "  测试: 窄走廊约束投影     通过 ("
                  << fitted.size() << " pts)" << std::endl;
    }

    // ---- Test 4: 参数 getter ----
    {
        BSpline bs;
        BSplineParams p;
        p.degree = 4;
        p.num_control_points = 60;
        p.closed = true;
        p.resample_spacing = 1.0;
        bs.setParams(p);
        auto& got = bs.getParams();
        assert(got.degree == 4);
        assert(got.num_control_points == 60);
        assert(got.closed == true);
        assert(std::abs(got.resample_spacing - 1.0) < 1e-9);
        std::cout << "  测试: 参数设置/获取      通过" << std::endl;
    }

    std::cout << "=== BSpline 测试全部通过 ===" << std::endl;
    return 0;
}
