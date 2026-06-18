#ifndef PNC_MOTION_ASTAR_H_
#define PNC_MOTION_ASTAR_H_

#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>
#include "../../common/types.h"

/// A* 路径规划 (8方向, 欧几里得启发式)
class AStar {
public:
    AStar(const std::vector<std::vector<int>>& grid,
          const Point& start, const Point& goal);

    const std::vector<Point>& getSearchHistory() const { return search_history_; }

    static const int DIRS[8][2];
    static const double MOVE_COST[8];

    std::vector<Point> findPath();

private:
    struct Node {
        int row, col;
        double g, h, f;
        bool operator>(const Node& o) const { return f > o.f; }
    };

    std::vector<std::vector<int>> grid_;
    Point start_, goal_;
    int size_;
    std::vector<std::vector<double>> g_score_;
    std::vector<std::vector<Point>> parent_;
    std::vector<std::vector<bool>> closed_;
    std::vector<Point> search_history_;
    static constexpr double INF = std::numeric_limits<double>::max();

    double heuristic(const Point& p) const;
    std::vector<Point> reconstructPath();
};

#endif  // PNC_MOTION_ASTAR_H_
