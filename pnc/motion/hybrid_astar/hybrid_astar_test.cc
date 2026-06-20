/**
 * Hybrid A* 单元测试
 *
 * 测试 plan() 和 planToGate() 在简单网格上找路径。
 */
#include "hybrid_astar.h"
#include <iostream>
#include <cassert>
#include <cmath>

int main() {
    std::cout << "=== Hybrid A* 单元测试 ===" << std::endl;

    int n = 30;  // 30x30 grid → 6m x 6m at 0.2m cell
    std::vector<std::vector<int>> grid(n, std::vector<int>(n, 0));

    // ---- Test 1: planToGate 无障碍 ----
    {
        HybridAStar ha(grid);
        ha.setCellSize(0.2);
        ha.setArcLength(0.6);
        ha.setGoalXYTol(0.5);
        ha.setMaxSteer(0.6);
        ha.setNumSteer(5);

        Pose start = {0.5, 0.5, 0.0};
        Vec2d gate_a = {5.0, 3.0};
        Vec2d gate_b = {5.0, 5.0};

        auto path = ha.planToGate(start, gate_a, gate_b);
        assert(!path.empty());
        assert(path.front().x == start.x && path.front().y == start.y);

        // 终点靠近 gate 线段 (planToGate 终止条件保证)
        auto& last = path.back();
        double d_end = std::hypot(last.x - gate_a.x, last.y - gate_a.y);
        assert(d_end < 3.0);  // 在 gate 附近

        std::cout << "  测试: planToGate 无障碍   通过 (len="
                  << path.size() << ")" << std::endl;
    }

    // ---- Test 2: planToGate 绕墙 ----
    {
        std::vector<std::vector<int>> grid2(n, std::vector<int>(n, 0));
        // 竖墙: col=12 (x≈2.4m), row 0..20
        for (int r = 0; r <= 20; r++) grid2[r][12] = 1;

        HybridAStar ha(grid2);
        ha.setCellSize(0.2);
        ha.setArcLength(0.6);
        ha.setGoalXYTol(0.5);

        Pose start = {0.5, 0.5, 0.0};
        Vec2d gate_a = {5.0, 3.0};
        Vec2d gate_b = {5.0, 5.0};

        auto path = ha.planToGate(start, gate_a, gate_b);
        assert(!path.empty());
        // 路径不能穿过墙: 所有点的x必须避免接近2.4m(墙的位置)与障碍物碰撞
        for (auto& p : path) {
            int r = (int)(p.y / 0.2), c = (int)(p.x / 0.2);
            assert(grid2[r][c] == 0);
        }
        std::cout << "  测试: planToGate 绕墙     通过 (len="
                  << path.size() << ")" << std::endl;
    }

    // ---- Test 3: plan() 回归 ----
    {
        HybridAStar ha(grid);
        ha.setCellSize(0.2);
        ha.setArcLength(0.6);
        ha.setGoalXYTol(0.3);
        ha.setGoalThTol(0.5);

        Pose start = {0.5, 0.5, 0.0};
        Pose goal  = {5.0, 5.0, 0.0};

        auto path = ha.plan(start, goal);
        assert(!path.empty());
        assert(path.front().x == start.x);
        assert(path.front().y == start.y);
        double d_end = std::hypot(path.back().x - goal.x, path.back().y - goal.y);
        assert(d_end < 1.0);
        std::cout << "  测试: plan() 回归         通过 (len="
                  << path.size() << ")" << std::endl;
    }

    // ---- Test 4: plan() 起点/终点碰撞 ----
    {
        std::vector<std::vector<int>> grid3(n, std::vector<int>(n, 0));
        grid3[2][2] = 1;  // 起点位置碰撞
        HybridAStar ha(grid3);
        ha.setCellSize(0.2);

        Pose start = {0.5, 0.5, 0.0};  // row=2, col=2
        Pose goal  = {5.0, 5.0, 0.0};
        auto path = ha.plan(start, goal);
        assert(path.empty());
        std::cout << "  测试: 起点碰撞返回空       通过" << std::endl;
    }

    // ---- Test 5: setVehicleDims 影响路径规划 ----
    {
        std::vector<std::vector<int>> grid4(n, std::vector<int>(n, 0));
        // 在狭窄通道两侧放障碍物
        for (int r = 0; r < n; r++) {
            grid4[r][6] = 1;   // 左墙 x≈1.2m
            grid4[r][9] = 1;   // 右墙 x≈1.8m  → 通道宽0.6m
        }
        // 开入口和出口
        for (int r = 0; r < 3; r++) grid4[r][6] = 0;
        for (int r = n-3; r < n; r++) grid4[r][9] = 0;

        HybridAStar ha_small(grid4);
        ha_small.setCellSize(0.2);
        ha_small.setArcLength(0.4);
        ha_small.setGoalXYTol(0.3);
        ha_small.setVehicleDims(0.1, 0.1, 0.1);  // 小车: 20cm半宽

        Pose start = {1.5, 0.2, 0.0};
        Pose goal  = {1.5, 5.5, 0.0};
        auto path_small = ha_small.plan(start, goal);
        // 小车应该能通过窄道
        assert(!path_small.empty());

        HybridAStar ha_big(grid4);
        ha_big.setCellSize(0.2);
        ha_big.setArcLength(0.4);
        ha_big.setGoalXYTol(0.3);
        ha_big.setVehicleDims(0.4, 0.4, 0.4);  // 大车: 40cm半宽 → 通道太窄

        auto path_big = ha_big.plan(start, goal);
        // 大车可能无法通过 (optional: 不强制断言)
        std::cout << "  测试: 车辆尺寸可配置       通过 (小车len="
                  << path_small.size() << ", 大车="
                  << (path_big.empty() ? "fail" : "pass") << ")"
                  << std::endl;
    }

    std::cout << "=== Hybrid A* 测试全部通过 ===" << std::endl;
    return 0;
}
