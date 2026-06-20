/**
 * SafeCorridor 单元测试
 *
 * 测试在简单多边形中构建安全走廊。
 */
#include "safe_corridor.h"
#include <iostream>
#include <cassert>
#include <cmath>

int main() {
    std::cout << "=== SafeCorridor 单元测试 ===" << std::endl;

    // ---- Test 1: 矩形边界, 直线路径 ----
    {
        SafeCorridor sc;
        sc.setMargin(0.5);
        sc.setSampleInterval(2.0);

        // 一条从左到右的直线穿过 10x10 矩形
        std::vector<Pose> ref_path;
        for (int i = 0; i < 6; i++)
            ref_path.push_back({1.0 + i, 5.0, 0.0});

        std::vector<Vec2d> outer = {
            {0, 0}, {10, 0}, {10, 10}, {0, 10}
        };
        std::vector<std::vector<Vec2d>> holes;

        auto corridor = sc.build(ref_path, outer, holes);
        assert(!corridor.empty());
        // 走廊应该在左右方向都有约束
        for (auto& sec : corridor) {
            double dl = std::hypot(sec.left.x - sec.center.x,
                                    sec.left.y - sec.center.y);
            double dr = std::hypot(sec.right.x - sec.center.x,
                                    sec.right.y - sec.center.y);
            // 左: 到 y=10 约 5m, 减去 0.5m margin ≈ 4.5m
            // 右: 到 y=0 约 5m, 减去 0.5m margin ≈ 4.5m
            assert(dl > 0.5 && dl < 6.0);
            assert(dr > 0.5 && dr < 6.0);
        }
        std::cout << "  测试: 矩形走廊            通过 ("
                  << corridor.size() << " sections)" << std::endl;
    }

    // ---- Test 2: 带孔洞边界 ----
    {
        SafeCorridor sc;
        sc.setMargin(0.3);
        sc.setSampleInterval(2.0);

        std::vector<Pose> ref_path;
        for (int i = 0; i < 6; i++)
            ref_path.push_back({1.0 + i, 5.0, 0.0});

        std::vector<Vec2d> outer = {
            {0, 0}, {10, 0}, {10, 10}, {0, 10}
        };
        // 路径上方放一个孔洞
        std::vector<Vec2d> hole = {
            {2, 6.5}, {6, 6.5}, {6, 7.5}, {2, 7.5}
        };
        std::vector<std::vector<Vec2d>> holes = {hole};

        auto corridor = sc.build(ref_path, outer, holes);
        assert(!corridor.empty());
        // 中间部分 (经过孔洞下方) 的上方约束应收紧
        for (auto& sec : corridor) {
            double dl = std::hypot(sec.left.x - sec.center.x,
                                    sec.left.y - sec.center.y);
            // 路径 y=5, 孔洞底边 y=6.5, 上方自由空间仅 ~1.5m - 0.3m = 1.2m
            if (sec.center.x > 2.5 && sec.center.x < 5.5) {
                assert(dl < 2.0);  // 被孔洞限制
            }
            double dr = std::hypot(sec.right.x - sec.center.x,
                                    sec.right.y - sec.center.y);
            assert(dr > 0.5);  // 下方无障碍
        }
        std::cout << "  测试: 带孔洞走廊          通过 ("
                  << corridor.size() << " sections)" << std::endl;
    }

    // ---- Test 3: 空路径 ----
    {
        SafeCorridor sc;
        std::vector<Pose> empty_path;
        std::vector<Vec2d> outer = {{0,0},{1,0},{1,1},{0,1}};
        std::vector<std::vector<Vec2d>> holes;
        auto corridor = sc.build(empty_path, outer, holes);
        assert(corridor.empty());
        std::cout << "  测试: 空路径返回空         通过" << std::endl;
    }

    // ---- Test 4: 弯曲路径 ----
    {
        SafeCorridor sc;
        sc.setMargin(0.2);
        sc.setSampleInterval(1.5);

        // 半圆弧路径
        std::vector<Pose> ref_path;
        for (int i = 0; i < 15; i++) {
            double a = M_PI * i / 14;  // 0 → π
            ref_path.push_back({5.0 + 2.0 * std::cos(a),
                                 5.0 + 2.0 * std::sin(a), a + M_PI_2});
        }

        std::vector<Vec2d> outer = {
            {0, 0}, {12, 0}, {12, 12}, {0, 12}
        };
        std::vector<std::vector<Vec2d>> holes;

        auto corridor = sc.build(ref_path, outer, holes);
        assert(!corridor.empty());
        for (auto& sec : corridor) {
            double dl = std::hypot(sec.left.x - sec.center.x,
                                    sec.left.y - sec.center.y);
            double dr = std::hypot(sec.right.x - sec.center.x,
                                    sec.right.y - sec.center.y);
            assert(dl >= 0.0 && dr >= 0.0);
        }
        std::cout << "  测试: 弯曲路径走廊        通过 ("
                  << corridor.size() << " sections)" << std::endl;
    }

    std::cout << "=== SafeCorridor 测试全部通过 ===" << std::endl;
    return 0;
}
