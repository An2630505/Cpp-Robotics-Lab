/**
 * SafeCorridor 单元测试 (栅格扫描方案)
 */
#include "safe_corridor.h"
#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>

/// 辅助: 构建全自由栅格
static std::vector<std::vector<int>> makeFreeGrid(int cols, int rows) {
    return std::vector<std::vector<int>>(rows, std::vector<int>(cols, 0));
}

int main() {
    std::cout << "=== SafeCorridor 单元测试 ===" << std::endl;

    // ---- Test 1: 全自由栅格, 直线路径 ----
    {
        SafeCorridor sc;
        sc.setMargin(0.5);
        sc.setSampleInterval(2.0);

        std::vector<Pose> ref_path;
        for (int i = 0; i < 6; i++)
            ref_path.push_back({1.0 + i, 5.0, 0.0});

        int cols = 100, rows = 100;
        double cell_size = 0.2;
        double x_min = 0.0, y_min = 0.0;
        auto grid = makeFreeGrid(cols, rows);

        auto corridor = sc.build(ref_path, grid, x_min, y_min,
                                  cell_size, cols, rows);
        assert(!corridor.empty());
        // 全自由 → 扫描到栅格边界
        for (auto& sec : corridor) {
            double dl = std::hypot(sec.left.x - sec.center.x,
                                    sec.left.y - sec.center.y);
            double dr = std::hypot(sec.right.x - sec.center.x,
                                    sec.right.y - sec.center.y);
            assert(dl > 0.5 && dl < 12.0);
            assert(dr > 0.5 && dr < 12.0);
        }
        std::cout << "  测试: 全自由栅格          通过 ("
                  << corridor.size() << " sections)" << std::endl;
    }

    // ---- Test 2: 带障碍物栅格 ----
    {
        SafeCorridor sc;
        sc.setMargin(0.0);  // 不用 margin, 精确测试
        sc.setSampleInterval(2.0);

        std::vector<Pose> ref_path;
        for (int i = 0; i < 6; i++)
            ref_path.push_back({1.0 + i, 5.0, 0.0});

        int cols = 100, rows = 100;
        double cell_size = 0.2;
        double x_min = 0.0, y_min = 0.0;
        auto grid = makeFreeGrid(cols, rows);

        // 路径上方放障碍物: x∈[2,6], y∈[6.5,7.5]
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                double wx = x_min + c * cell_size;
                double wy = y_min + r * cell_size;
                if (wx >= 2.0 && wx <= 6.0 && wy >= 6.5 && wy <= 7.5)
                    grid[r][c] = 1;
            }
        }

        auto corridor = sc.build(ref_path, grid, x_min, y_min,
                                  cell_size, cols, rows);
        assert(!corridor.empty());
        for (auto& sec : corridor) {
            double dl = std::hypot(sec.left.x - sec.center.x,
                                    sec.left.y - sec.center.y);
            // 经过障碍物下方的截面应收紧 (路径 y=5, 障碍物底边 y=6.5)
            if (sec.center.x > 2.5 && sec.center.x < 5.5) {
                assert(dl < 2.0);
            }
            double dr = std::hypot(sec.right.x - sec.center.x,
                                    sec.right.y - sec.center.y);
            assert(dr > 0.5);
        }
        std::cout << "  测试: 带障碍物栅格        通过 ("
                  << corridor.size() << " sections)" << std::endl;
    }

    // ---- Test 3: 空路径 ----
    {
        SafeCorridor sc;
        std::vector<Pose> empty_path;
        auto grid = makeFreeGrid(10, 10);
        auto corridor = sc.build(empty_path, grid,
                                  0.0, 0.0, 0.2, 10, 10);
        assert(corridor.empty());
        std::cout << "  测试: 空路径返回空         通过" << std::endl;
    }

    // ---- Test 4: 弯曲路径 ----
    {
        SafeCorridor sc;
        sc.setMargin(0.2);
        sc.setSampleInterval(1.5);

        std::vector<Pose> ref_path;
        for (int i = 0; i < 15; i++) {
            double a = M_PI * i / 14;
            ref_path.push_back({5.0 + 2.0 * std::cos(a),
                                 5.0 + 2.0 * std::sin(a), a + M_PI_2});
        }

        int cols = 120, rows = 120;
        double cell_size = 0.2;
        double x_min = 0.0, y_min = 0.0;
        auto grid = makeFreeGrid(cols, rows);

        auto corridor = sc.build(ref_path, grid, x_min, y_min,
                                  cell_size, cols, rows);
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
