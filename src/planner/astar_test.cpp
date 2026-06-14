#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <cmath>
#include <string>
#include <limits>
#include <algorithm>
#include <sstream>
#include <cstdlib>
#include "astar.h"

// ========================== 主函数 ==========================
int main(int argc, char* argv[]) {
    std::string input_file = "output/grid.txt";

    // 支持命令行指定输入文件
    if (argc > 1) {
        input_file = argv[1];
    }

    std::cout << "=== A* 路径规划 (C++) ===" << std::endl;

    // 1. 读取网格
    GridData data = readGrid(input_file);

    // 2. 执行 A*
    AStar astar(data.grid, data.start, data.goal);
    std::vector<Point> path = astar.findPath();

    if (path.empty()) {
        std::cout << "未找到从起点到终点的路径!" << std::endl;
        return 1;
    }

    // 3. 统计信息
    printPathStats(path);

    // 4. 保存路径文本
    savePath(path, "output/path.txt");

    // 5. 保存搜索历史 (供动画使用)
    saveSearchHistory(astar.getSearchHistory(), "output/search_history.txt");

    // 6. 保存结果图片
    saveResultPPM(data.grid, data.start, data.goal, path, "output/path_result.ppm");

    std::cout << "\n查看结果: open output/path_result.ppm" << std::endl;

    return 0;
}
