#include "astar.h"

const int AStar::DIRS[8][2] = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1},
    {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
};

const double AStar::MOVE_COST[8] = {
    1.0, 1.0, 1.0, 1.0,
    std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0)
};

AStar::AStar(const std::vector<std::vector<int>>& grid,
             const Point& start, const Point& goal)
    : grid_(grid), start_(start), goal_(goal),
      size_(grid.size()),
      g_score_(size_, std::vector<double>(size_, INF)),
      parent_(size_, std::vector<Point>(size_, {-1, -1})),
      closed_(size_, std::vector<bool>(size_, false))
{}

std::vector<Point> AStar::findPath() {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

    g_score_[start_.row][start_.col] = 0.0;
    double h0 = heuristic(start_);
    open.push({start_.row, start_.col, 0.0, h0, h0});

    int nodes_expanded = 0;

    while (!open.empty()) {
        Node current = open.top(); open.pop();
        if (closed_[current.row][current.col]) continue;
        closed_[current.row][current.col] = true;
        search_history_.push_back({current.row, current.col});
        nodes_expanded++;

        if (current.row == goal_.row && current.col == goal_.col)
            return reconstructPath();

        for (int i = 0; i < 8; i++) {
            int nr = current.row + DIRS[i][0];
            int nc = current.col + DIRS[i][1];
            if (nr < 0 || nr >= size_ || nc < 0 || nc >= size_) continue;
            if (grid_[nr][nc] == 1) continue;
            if (closed_[nr][nc]) continue;

            double new_g = current.g + MOVE_COST[i];
            if (new_g < g_score_[nr][nc]) {
                g_score_[nr][nc] = new_g;
                parent_[nr][nc] = {current.row, current.col};
                double h = heuristic({nr, nc});
                open.push({nr, nc, new_g, h, new_g + h});
            }
        }
    }
    return {};
}

double AStar::heuristic(const Point& p) const {
    double dr = p.row - goal_.row;
    double dc = p.col - goal_.col;
    return std::sqrt(dr * dr + dc * dc);
}

std::vector<Point> AStar::reconstructPath() {
    std::vector<Point> path;
    Point cur = goal_;
    while (!(cur == start_)) {
        path.push_back(cur);
        cur = parent_[cur.row][cur.col];
        if (cur.row == -1 || cur.col == -1) return {};
    }
    path.push_back(start_);
    std::reverse(path.begin(), path.end());
    return path;
}
