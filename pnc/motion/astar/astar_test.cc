/**
 * A* 单元测试
 *
 * 测试 8 方向 A* 在简单网格上找到最短路径。
 */
#include "astar.h"
#include <iostream>
#include <cassert>

int main() {
    std::cout << "=== A* 单元测试 ===" << std::endl;

    // 10x10 grid, 1=obstacle, 0=free
    int n = 10;
    std::vector<std::vector<int>> grid(n, std::vector<int>(n, 0));
    grid[5][3] = 1; grid[5][4] = 1; grid[5][5] = 1;  // 竖墙

    Point start = {0, 0};
    Point goal  = {9, 9};

    AStar astar(grid, start, goal);
    auto path = astar.findPath();

    assert(!path.empty());
    assert(path.front() == start);
    assert(path.back() == goal);

    // 连续两步不能跳对角线穿墙
    for (size_t i = 1; i < path.size(); i++) {
        int dr = std::abs(path[i].row - path[i-1].row);
        int dc = std::abs(path[i].col - path[i-1].col);
        assert(dr <= 1 && dc <= 1);
    }

    std::cout << "  测试: 找到路径, 长度=" << path.size()
              << ", 展开=" << astar.getSearchHistory().size() << std::endl;

    // 无解网格: 起点被堵死
    std::vector<std::vector<int>> blocked(n, std::vector<int>(n, 0));
    blocked[0][1] = 1; blocked[1][0] = 1; blocked[1][1] = 1;  // 封死起点
    AStar astar2(blocked, start, goal);
    auto path2 = astar2.findPath();
    assert(path2.empty());
    std::cout << "  测试: 死路返回空         通过" << std::endl;

    std::cout << "=== A* 测试全部通过 ===" << std::endl;
    return 0;
}
