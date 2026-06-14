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

// ========================== 数据结构 ==========================
struct Point {
    int row, col;
    bool operator==(const Point& other) const {
        return row == other.row && col == other.col;
    }
};

struct Node {
    int row, col;
    double g;  // 起点到当前节点的实际代价
    double h;  // 当前节点到终点的启发式估计代价
    double f;  // f = g + h

    bool operator>(const Node& other) const {
        return f > other.f;  // 优先队列需要小顶堆，f 越小越优先
    }
};

// ========================== A* 算法 ==========================
class AStar {
public:
    AStar(const std::vector<std::vector<int>>& grid,
          const Point& start, const Point& goal)
        : grid_(grid), start_(start), goal_(goal),
          size_(grid.size()),
          g_score_(size_, std::vector<double>(size_, INF)),
          parent_(size_, std::vector<Point>(size_, {-1, -1})),
          closed_(size_, std::vector<bool>(size_, false))
    {}

    // 获取搜索历史（展开顺序）
    const std::vector<Point>& getSearchHistory() const { return search_history_; }

    // 8 方向移动：上、下、左、右、左上、右上、左下、右下
    static const int DIRS[8][2];
    // 对应移动代价（对角线代价为 sqrt(2)）
    static const double MOVE_COST[8];

    std::vector<Point> findPath() {
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

        // 起点初始化
        g_score_[start_.row][start_.col] = 0.0;
        double h0 = heuristic(start_);
        open.push({start_.row, start_.col, 0.0, h0, h0});

        int nodes_expanded = 0;

        while (!open.empty()) {
            Node current = open.top();
            open.pop();

            // 如果已被标记为 closed，说明已经找到更优路径，跳过
            if (closed_[current.row][current.col]) continue;
            closed_[current.row][current.col] = true;
            search_history_.push_back({current.row, current.col});
            nodes_expanded++;

            // 到达终点
            if (current.row == goal_.row && current.col == goal_.col) {
                std::cout << "A* 搜索完成! 展开节点数: " << nodes_expanded << std::endl;
                std::cout << "路径代价: " << current.g << std::endl;
                return reconstructPath();
            }

            // 扩展邻居
            for (int i = 0; i < 8; i++) {
                int nr = current.row + DIRS[i][0];
                int nc = current.col + DIRS[i][1];

                // 越界检查
                if (nr < 0 || nr >= size_ || nc < 0 || nc >= size_) continue;
                // 障碍物检查
                if (grid_[nr][nc] == 1) continue;
                // 已关闭检查
                if (closed_[nr][nc]) continue;

                double new_g = current.g + MOVE_COST[i];

                // 如果找到更优路径则更新
                if (new_g < g_score_[nr][nc]) {
                    g_score_[nr][nc] = new_g;
                    parent_[nr][nc] = {current.row, current.col};
                    double h = heuristic({nr, nc});
                    open.push({nr, nc, new_g, h, new_g + h});
                }
            }
        }

        std::cout << "A* 搜索完成! 展开节点数: " << nodes_expanded << std::endl;
        std::cout << "未找到路径!" << std::endl;
        return {};  // 无路径
    }

private:
    const std::vector<std::vector<int>>& grid_;
    Point start_, goal_;
    int size_;
    std::vector<std::vector<double>> g_score_;
    std::vector<std::vector<Point>> parent_;
    std::vector<std::vector<bool>> closed_;
    std::vector<Point> search_history_;   // 展开顺序记录
    static constexpr double INF = std::numeric_limits<double>::max();

    // 欧几里得距离启发式（对 8 方向移动是可采纳的）
    double heuristic(const Point& p) const {
        double dr = p.row - goal_.row;
        double dc = p.col - goal_.col;
        return std::sqrt(dr * dr + dc * dc);
    }

    // 回溯重建路径
    std::vector<Point> reconstructPath() {
        std::vector<Point> path;
        Point cur = goal_;

        while (!(cur == start_)) {
            path.push_back(cur);
            cur = parent_[cur.row][cur.col];

            // 安全检查：防止死循环
            if (cur.row == -1 || cur.col == -1) {
                std::cerr << "错误: 路径回溯失败!" << std::endl;
                return {};
            }
        }
        path.push_back(start_);
        std::reverse(path.begin(), path.end());
        return path;
    }
};

const int AStar::DIRS[8][2] = {
    {-1,  0},  // 上
    { 1,  0},  // 下
    { 0, -1},  // 左
    { 0,  1},  // 右
    {-1, -1},  // 左上
    {-1,  1},  // 右上
    { 1, -1},  // 左下
    { 1,  1}   // 右下
};

const double AStar::MOVE_COST[8] = {
    1.0, 1.0, 1.0, 1.0,
    std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0)
};

// ========================== 颜色定义 ==========================
struct Color {
    int r, g, b;
};

const Color WHITE    = {255, 255, 255};
const Color BLACK    = {0,   0,   0  };
const Color BLUE     = {0,   0,   255};
const Color RED      = {255, 0,   0  };
const Color GREEN    = {0,   255, 0  };

// ========================== 文件读写 ==========================
struct GridData {
    std::vector<std::vector<int>> grid;
    Point start, goal;
    int size;
};

GridData readGrid(const std::string& filepath) {
    std::ifstream in(filepath);
    if (!in.is_open()) {
        std::cerr << "错误: 无法打开文件 " << filepath << std::endl;
        exit(1);
    }

    GridData data;
    std::string line;

    // 跳过注释行
    while (std::getline(in, line)) {
        if (line[0] != '#') break;
    }

    // 读取网格大小
    std::istringstream iss(line);
    iss >> data.size;

    // 读取起点
    in >> data.start.row >> data.start.col;

    // 读取终点
    in >> data.goal.row >> data.goal.col;

    // 读取网格
    data.grid.assign(data.size, std::vector<int>(data.size, 0));
    for (int r = 0; r < data.size; r++) {
        for (int c = 0; c < data.size; c++) {
            in >> data.grid[r][c];
        }
    }

    in.close();
    std::cout << "读取网格: " << data.size << "x" << data.size << std::endl;
    std::cout << "起点: (" << data.start.row << ", " << data.start.col << ")" << std::endl;
    std::cout << "终点: (" << data.goal.row << ", " << data.goal.col << ")" << std::endl;
    return data;
}

// ========================== 保存路径 ==========================
void savePath(const std::vector<Point>& path, const std::string& filepath) {
    std::ofstream out(filepath);
    if (!out.is_open()) {
        std::cerr << "错误: 无法创建文件 " << filepath << std::endl;
        return;
    }

    out << "# A* Path\n";
    out << "# steps: " << path.size() << "\n";
    out << "# format: row col\n";

    for (const auto& p : path) {
        out << p.row << " " << p.col << "\n";
    }

    out.close();
    std::cout << "路径已保存至: " << filepath << " (" << path.size() << " 步)" << std::endl;
}

// ========================== 保存搜索历史 ==========================
void saveSearchHistory(const std::vector<Point>& history,
                       const std::string& filepath) {
    std::ofstream out(filepath);
    if (!out.is_open()) {
        std::cerr << "错误: 无法创建文件 " << filepath << std::endl;
        return;
    }

    out << "# A* Search History (expansion order)\n";
    out << "# steps: " << history.size() << "\n";
    out << "# format: row col\n";

    for (const auto& p : history) {
        out << p.row << " " << p.col << "\n";
    }

    out.close();
    std::cout << "搜索历史已保存至: " << filepath
              << " (" << history.size() << " 次展开)" << std::endl;
}

// ========================== 保存 PPM 图片 ==========================
void saveResultPPM(const std::vector<std::vector<int>>& grid,
                   const Point& start, const Point& goal,
                   const std::vector<Point>& path,
                   const std::string& filepath) {
    std::ofstream out(filepath);
    if (!out.is_open()) {
        std::cerr << "错误: 无法创建文件 " << filepath << std::endl;
        return;
    }

    const int SCALE = 3;
    int img_w = grid.size() * SCALE;
    int img_h = grid.size() * SCALE;

    out << "P3\n" << img_w << " " << img_h << "\n255\n";

    // 将路径点放入查找表
    std::vector<std::vector<bool>> on_path(grid.size(),
        std::vector<bool>(grid.size(), false));
    for (const auto& p : path) {
        on_path[p.row][p.col] = true;
    }
    // 排除起点和终点，让它们保持蓝色和红色
    on_path[start.row][start.col] = false;
    on_path[goal.row][goal.col] = false;

    for (int r = 0; r < grid.size(); r++) {
        for (int sr = 0; sr < SCALE; sr++) {
            for (int c = 0; c < grid.size(); c++) {
                Color color;
                if (r == start.row && c == start.col) {
                    color = BLUE;
                } else if (r == goal.row && c == goal.col) {
                    color = RED;
                } else if (on_path[r][c]) {
                    color = GREEN;
                } else if (grid[r][c] == 1) {
                    color = BLACK;
                } else {
                    color = WHITE;
                }

                for (int sc = 0; sc < SCALE; sc++) {
                    out << color.r << " " << color.g << " " << color.b << " ";
                }
            }
            out << "\n";
        }
    }

    out.close();
    std::cout << "结果图片已保存至: " << filepath << std::endl;
}

// ========================== 统计信息 ==========================
void printPathStats(const std::vector<Point>& path) {
    if (path.empty()) return;

    std::cout << "\n=== 路径统计 ===" << std::endl;
    std::cout << "总步数: " << path.size() << std::endl;

    // 计算实际路径长度
    double total_len = 0.0;
    for (size_t i = 1; i < path.size(); i++) {
        double dr = path[i].row - path[i-1].row;
        double dc = path[i].col - path[i-1].col;
        total_len += std::sqrt(dr * dr + dc * dc);
    }
    std::cout << "路径长度: " << total_len << std::endl;

    // 直线距离
    double straight = std::hypot(
        path.back().row - path.front().row,
        path.back().col - path.front().col);
    std::cout << "直线距离: " << straight << std::endl;
    std::cout << "路径效率: " << (straight / total_len * 100.0) << "%" << std::endl;

    // 转折点统计
    int turns = 0;
    if (path.size() > 2) {
        for (size_t i = 2; i < path.size(); i++) {
            int dr1 = path[i-1].row - path[i-2].row;
            int dc1 = path[i-1].col - path[i-2].col;
            int dr2 = path[i].row - path[i-1].row;
            int dc2 = path[i].col - path[i-1].col;
            if (dr1 != dr2 || dc1 != dc2) turns++;
        }
    }
    std::cout << "转折次数: " << turns << std::endl;
}


