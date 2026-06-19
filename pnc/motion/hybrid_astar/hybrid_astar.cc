#include "hybrid_astar.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <queue>
#include <unordered_map>

HybridAStar::HybridAStar(const std::vector<std::vector<int>>& grid)
    : grid_(grid), grid_size_(static_cast<int>(grid.size())),
      cell_size_(0.2), wheelbase_(2.68), max_steer_(0.6), arc_length_(1.5),
      num_steer_(5), theta_bins_(72), xy_bin_(0.5),
      goal_xy_tol_(2.0), goal_th_tol_(0.5)
{}

bool HybridAStar::collides(const Pose& p) const {
    double c = std::cos(p.theta), s = std::sin(p.theta);
    double hw = 0.3, fwd = 0.3, rev = 0.3;
    double crn[4][2] = {{fwd, hw}, {fwd, -hw}, {-rev, hw}, {-rev, -hw}};
    double mx = 1e9, Mx = -1e9, my = 1e9, My = -1e9;
    for (int i = 0; i < 4; i++) {
        double wx = c * crn[i][0] - s * crn[i][1] + p.x;
        double wy = s * crn[i][0] + c * crn[i][1] + p.y;
        mx = std::min(mx, wx); Mx = std::max(Mx, wx);
        my = std::min(my, wy); My = std::max(My, wy);
    }
    int cmn = std::max(0, (int)(mx / cell_size_));
    int cmx = std::min(grid_size_ - 1, (int)(Mx / cell_size_) + 1);
    int rmn = std::max(0, (int)(my / cell_size_));
    int rmx = std::min(grid_size_ - 1, (int)(My / cell_size_) + 1);
    for (int r = rmn; r <= rmx; r++)
        for (int ci = cmn; ci <= cmx; ci++) {
            if (grid_[r][ci] == 0) continue;
            double cx = ci * cell_size_ + 0.1, cy = r * cell_size_ + 0.1;
            double dx = cx - p.x, dy = cy - p.y;
            double bx = c * dx + s * dy, by = -s * dx + c * dy;
            if (bx >= -rev && bx <= fwd && by >= -hw && by <= hw) return true;
        }
    return false;
}

Pose HybridAStar::step(const Pose& from, double steer, double arc) const {
    if (std::abs(steer) < 1e-6)
        return {from.x + arc * std::cos(from.theta),
                from.y + arc * std::sin(from.theta), from.theta};
    double R = wheelbase_ / std::tan(steer), dth = arc / R;
    return {from.x + R * (std::sin(from.theta + dth) - std::sin(from.theta)),
            from.y + R * (std::cos(from.theta) - std::cos(from.theta + dth)),
            from.theta + dth};
}

bool HybridAStar::arcCollides(const Pose& from, double steer, double arc) const {
    int n = std::max(2, static_cast<int>(arc / cell_size_));
    for (int i = 0; i <= n; i++)
        if (collides(step(from, steer, arc * i / n))) return true;
    return false;
}

std::vector<Pose> HybridAStar::plan(const Pose& start, const Pose& goal) {
    if (collides(start)) return {};
    if (collides(goal)) return {};

    // ---- Dijkstra 启发式 ----
    std::vector<std::vector<double>> h2d(grid_size_,
        std::vector<double>(grid_size_, 1e9));
    {
        using Cell = std::pair<double, std::pair<int, int>>;
        std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> pq;
        int gr = (int)(goal.y / cell_size_), gc = (int)(goal.x / cell_size_);
        if (gr >= 0 && gr < grid_size_ && gc >= 0 && gc < grid_size_) {
            h2d[gr][gc] = 0; pq.push({0, {gr, gc}});
        }
        int d8[8][2] = {{-1,0},{1,0},{0,-1},{0,1},{-1,-1},{-1,1},{1,-1},{1,1}};
        double c8[8] = {1,1,1,1,1.414,1.414,1.414,1.414};
        while (!pq.empty()) {
            auto t = pq.top(); pq.pop();
            int r = t.second.first, c = t.second.second;
            if (t.first > h2d[r][c] + 1e-6) continue;
            for (int d = 0; d < 8; d++) {
                int nr = r + d8[d][0], nc = c + d8[d][1];
                if (nr < 0 || nr >= grid_size_ || nc < 0 || nc >= grid_size_)
                    continue;
                if (grid_[nr][nc] == 1) continue;
                double nd = t.first + c8[d];
                if (nd < h2d[nr][nc]) { h2d[nr][nc] = nd; pq.push({nd, {nr, nc}}); }
            }
        }
    }

    // ---- 转向角 ----
    std::vector<double> steers;
    for (int i = 0; i < num_steer_; i++)
        steers.push_back((i - num_steer_ / 2) * max_steer_ / (num_steer_ / 2));

    // ---- A* 搜索 ----
    std::priority_queue<HNode, std::vector<HNode>, std::greater<HNode>> open;
    std::unordered_map<int, double> best_g;
    std::vector<HNode> closed;

    int sr = (int)(start.y / cell_size_), sc = (int)(start.x / cell_size_);
    HNode sn = makeNode(start.x, start.y, start.theta);
    sn.g = 0;
    sn.h = h2d[sr][sc] * cell_size_;
    sn.f = sn.g + sn.h;
    sn.parent = -1;
    open.push(sn);
    best_g[sn.key()] = 0;

    while (!open.empty()) {
        HNode cur = open.top(); open.pop();
        auto it = best_g.find(cur.key());
        if (it != best_g.end() && cur.g > it->second + 1e-6) continue;
        int cur_idx = closed.size();
        closed.push_back(cur);

        double dg = std::hypot(cur.x - goal.x, cur.y - goal.y);
        double dth = std::abs(cur.theta - goal.theta);
        while (dth > M_PI) dth = 2 * M_PI - dth;
        if (dg < goal_xy_tol_ && dth < goal_th_tol_) {
            std::vector<Pose> path;
            int idx = closed.size() - 1;
            while (idx >= 0) {
                path.push_back({closed[idx].x, closed[idx].y, closed[idx].theta});
                idx = closed[idx].parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (double steer : steers) {
            Pose np = step({cur.x, cur.y, cur.theta}, steer, arc_length_);
            double map_w = grid_size_ * cell_size_;
            if (np.x < 0 || np.x >= map_w || np.y < 0 || np.y >= map_w) continue;
            if (arcCollides({cur.x, cur.y, cur.theta}, steer, arc_length_)) continue;

            double cost = arc_length_ + std::abs(steer) * arc_length_ * 0.3;
            double ng = cur.g + cost;
            int hr = (int)(np.y / cell_size_), hc = (int)(np.x / cell_size_);
            double hv = h2d[hr][hc];
            if (hv > 1e8) continue;

            HNode child = makeNode(np.x, np.y, np.theta);
            child.g = ng;
            child.parent = cur_idx;
            child.h = hv * cell_size_;
            child.f = ng + child.h;

            int k = child.key();
            auto it2 = best_g.find(k);
            if (it2 != best_g.end() && ng >= it2->second - 1e-6) continue;
            best_g[k] = ng;
            open.push(child);
        }
    }
    return {};
}
