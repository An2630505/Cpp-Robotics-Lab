#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <cmath>
#include <string>

// ========================== 参数配置 ==========================
const int GRID_SIZE = 256;           // 网格大小 256x256
const double OBSTACLE_RATIO = 0.2;  // 障碍物占比 (0.0 ~ 1.0)
const int MARGIN = 10;               // 起点/终点距边界的最小距离
const double MIN_DIST_RATIO = 0.3;   // 起点到终点的最小距离比例

// ========================== 随机数生成 ==========================
// 取消注释下面一行可固定随机种子，便于复现
// #define FIXED_SEED 42

// ========================== PPM 颜色定义 ==========================
struct Color {
    int r, g, b;
};

const Color WHITE    = {255, 255, 255};  // 空闲
const Color BLACK    = {0,   0,   0  };  // 障碍
const Color BLUE     = {0,   0,   255};  // 起点
const Color RED      = {255, 0,   0  };  // 终点

// ========================== 网格生成 ==========================

// 在 grid 上填充一个矩形障碍物，跳过起点/终点区域（如果已设置）
void fillRect(std::vector<std::vector<int>>& grid,
              int r1, int c1, int r2, int c2) {
    for (int r = r1; r <= r2; r++) {
        for (int c = c1; c <= c2; c++) {
            grid[r][c] = 1;
        }
    }
}

void generateGrid(std::vector<std::vector<int>>& grid, std::mt19937& rng) {
    grid.assign(GRID_SIZE, std::vector<int>(GRID_SIZE, 0));

    int total_cells = GRID_SIZE * GRID_SIZE;
    int target_obs = static_cast<int>(total_cells * OBSTACLE_RATIO);
    int placed = 0;

    // 矩形尺寸范围
    std::uniform_int_distribution<int> w_dist(4, 30);  // 宽度
    std::uniform_int_distribution<int> h_dist(4, 30);  // 高度
    std::uniform_int_distribution<int> pos_dist(0, GRID_SIZE - 1);

    // 随机放置矩形块
    while (placed < target_obs) {
        int w = w_dist(rng);
        int h = h_dist(rng);
        int r = pos_dist(rng);
        int c = pos_dist(rng);

        // 裁剪到边界内
        int r_end = std::min(r + h - 1, GRID_SIZE - 1);
        int c_end = std::min(c + w - 1, GRID_SIZE - 1);

        // 检查该区域是否全为空（避免重叠太多）
        int area = (r_end - r + 1) * (c_end - c + 1);
        int already_filled = 0;
        for (int rr = r; rr <= r_end; rr++)
            for (int cc = c; cc <= c_end; cc++)
                if (grid[rr][cc] == 1) already_filled++;

        // 如果重叠超过 30%，跳过
        if (already_filled > area * 0.3) continue;

        fillRect(grid, r, c, r_end, c_end);
        placed += (area - already_filled);
    }

    // 额外添加一些细长的"墙壁"（横向 + 纵向），让地图更有结构感
    std::uniform_int_distribution<int> wall_len(20, 80);
    std::uniform_int_distribution<int> thin_pos(5, GRID_SIZE - 5);
    int walls = GRID_SIZE / 16;  // ~16 面墙

    for (int i = 0; i < walls; i++) {
        int len = wall_len(rng);
        int r = thin_pos(rng);
        int c = thin_pos(rng);

        if (i % 2 == 0) {
            // 横向墙 (厚度 1~2)
            int thick = (rng() % 2) + 1;
            int c_end = std::min(c + len, GRID_SIZE - 1);
            int r_end = std::min(r + thick - 1, GRID_SIZE - 1);
            fillRect(grid, r, c, r_end, c_end);
        } else {
            // 纵向墙 (厚度 1~2)
            int thick = (rng() % 2) + 1;
            int r_end = std::min(r + len, GRID_SIZE - 1);
            int c_end = std::min(c + thick - 1, GRID_SIZE - 1);
            fillRect(grid, r, c, r_end, c_end);
        }
    }
}

// ========================== 起点/终点生成 ==========================
struct Point {
    int row, col;
};

void generateStartGoal(const std::vector<std::vector<int>>& grid,
                       Point& start, Point& goal,
                       std::mt19937& rng) {
    // 收集符合条件的空闲格子
    std::vector<Point> free_cells;
    for (int r = MARGIN; r < GRID_SIZE - MARGIN; r++) {
        for (int c = MARGIN; c < GRID_SIZE - MARGIN; c++) {
            if (grid[r][c] == 0) {
                free_cells.push_back({r, c});
            }
        }
    }

    if (free_cells.size() < 2) {
        std::cerr << "错误: 空闲格子不足，无法放置起点和终点!" << std::endl;
        exit(1);
    }

    std::uniform_int_distribution<size_t> dist(0, free_cells.size() - 1);
    double min_dist = GRID_SIZE * MIN_DIST_RATIO;

    // 保证起点和终点有足够的间距
    do {
        size_t i1 = dist(rng);
        size_t i2 = dist(rng);
        while (i2 == i1) i2 = dist(rng);

        start = free_cells[i1];
        goal  = free_cells[i2];
    } while (std::hypot(start.row - goal.row, start.col - goal.col) < min_dist);
}

// ========================== 保存文本网格 ==========================
void saveGridText(const std::vector<std::vector<int>>& grid,
                  const Point& start, const Point& goal,
                  const std::string& filepath) {
    std::ofstream out(filepath);
    if (!out.is_open()) {
        std::cerr << "错误: 无法创建文件 " << filepath << std::endl;
        return;
    }

    out << "# A* Grid Map\n";
    out << "# size: " << GRID_SIZE << "x" << GRID_SIZE << "\n";
    out << "# start: (" << start.row << ", " << start.col << ")\n";
    out << "# goal:  (" << goal.row << ", " << goal.col << ")\n";
    out << "# 0=free, 1=obstacle\n";
    out << GRID_SIZE << "\n";
    out << start.row << " " << start.col << "\n";
    out << goal.row << " " << goal.col << "\n";

    for (int r = 0; r < GRID_SIZE; r++) {
        for (int c = 0; c < GRID_SIZE; c++) {
            out << grid[r][c];
            if (c < GRID_SIZE - 1) out << " ";
        }
        out << "\n";
    }

    out.close();
    std::cout << "网格数据已保存至: " << filepath << std::endl;
}

// ========================== 保存 PPM 图片 ==========================
void saveGridPPM(const std::vector<std::vector<int>>& grid,
                 const Point& start, const Point& goal,
                 const std::string& filepath) {
    std::ofstream out(filepath);
    if (!out.is_open()) {
        std::cerr << "错误: 无法创建文件 " << filepath << std::endl;
        return;
    }

    // PPM P3 (ASCII) header
    out << "P3\n";
    out << GRID_SIZE << " " << GRID_SIZE << "\n";
    out << "255\n";

    // 放大每个格子让图片更清晰 (pixel scale factor)
    const int SCALE = 3;  // 每个格子 3x3 像素，使 256x256 的图片不至于太小
    int img_w = GRID_SIZE * SCALE;
    int img_h = GRID_SIZE * SCALE;

    out.seekp(0);
    out << "P3\n";
    out << img_w << " " << img_h << "\n";
    out << "255\n";

    for (int r = 0; r < GRID_SIZE; r++) {
        for (int sr = 0; sr < SCALE; sr++) {
            for (int c = 0; c < GRID_SIZE; c++) {
                Color color;
                if (r == start.row && c == start.col) {
                    color = BLUE;
                } else if (r == goal.row && c == goal.col) {
                    color = RED;
                } else if (grid[r][c] == 1) {
                    color = BLACK;
                } else {
                    color = WHITE;
                }

                // 每个格子填充 SCALE 个像素
                for (int sc = 0; sc < SCALE; sc++) {
                    out << color.r << " " << color.g << " " << color.b << " ";
                }
            }
            out << "\n";
        }
    }

    out.close();
    std::cout << "图片已保存至: " << filepath << std::endl;
}

// ========================== 主函数 ==========================
int main() {
#ifdef FIXED_SEED
    std::mt19937 rng(FIXED_SEED);
    std::cout << "使用固定随机种子: " << FIXED_SEED << std::endl;
#else
    std::random_device rd;
    std::mt19937 rng(rd());
    std::cout << "使用随机设备生成种子" << std::endl;
#endif

    std::cout << "=== A* 网格地图生成 (C++) ===" << std::endl;
    std::cout << "网格大小: " << GRID_SIZE << "x" << GRID_SIZE << std::endl;
    std::cout << "障碍物占比: " << OBSTACLE_RATIO * 100 << "%" << std::endl;

    // 1. 生成网格
    std::vector<std::vector<int>> grid;
    generateGrid(grid, rng);

    int free_count = 0, obs_count = 0;
    for (const auto& row : grid) {
        for (int cell : row) {
            if (cell == 0) free_count++;
            else obs_count++;
        }
    }
    std::cout << "空闲格子: " << free_count << ", 障碍物: " << obs_count << std::endl;

    // 2. 生成起点和终点
    Point start, goal;
    generateStartGoal(grid, start, goal, rng);
    std::cout << "起点 (Start):  (" << start.row << ", " << start.col << ")" << std::endl;
    std::cout << "终点 (Goal):   (" << goal.row << ", " << goal.col << ")" << std::endl;
    std::cout << "欧几里得距离: "
              << std::hypot(start.row - goal.row, start.col - goal.col) << std::endl;

    // 3. 保存文本网格
    saveGridText(grid, start, goal, "output/grid.txt");

    // 4. 保存 PPM 图片
    saveGridPPM(grid, start, goal, "output/grid_map.ppm");

    std::cout << "\n可使用以下命令查看图片:" << std::endl;
    std::cout << "  open output/grid_map.ppm    (macOS 预览)" << std::endl;
    std::cout << "  python3 -c \"from PIL import Image; Image.open('output/grid_map.ppm').show()\"" << std::endl;

    return 0;
}
