/**
 * MPC 轨迹规划器 — 纯追踪 (Pure Pursuit) 控制器
 *
 * 给定参考路径 (x,y,θ 序列), 用运动学自行车模型 + 纯追踪
 * 生成平滑轨迹和控制序列。
 */
#include "mpc_planner.h"

#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>

// ===================================================================
//  ContinuousMap
// ===================================================================

ContinuousMap::ContinuousMap(const std::vector<std::vector<int>>& grid)
    : grid_(grid) {}  // now stored by value

int ContinuousMap::worldToRow(double y) const {
    int row = static_cast<int>(y / MPC_CELL_SIZE);
    return std::max(0, std::min(MPC_GRID_SIZE - 1, row));
}
int ContinuousMap::worldToCol(double x) const {
    int col = static_cast<int>(x / MPC_CELL_SIZE);
    return std::max(0, std::min(MPC_GRID_SIZE - 1, col));
}
double ContinuousMap::rowToWorldY(int row) const {
    return row * MPC_CELL_SIZE + MPC_CELL_SIZE / 2.0;
}
double ContinuousMap::colToWorldX(int col) const {
    return col * MPC_CELL_SIZE + MPC_CELL_SIZE / 2.0;
}
bool ContinuousMap::isOccupied(int row, int col) const {
    if (row < 0 || row >= MPC_GRID_SIZE || col < 0 || col >= MPC_GRID_SIZE)
        return true;
    return grid_[row][col] == 1;
}

bool ContinuousMap::isCollision(const Pose& pose) const {
    double c = std::cos(pose.theta), s = std::sin(pose.theta);
    double hw = MPC_CAR_WIDTH / 2.0 + MPC_CELL_SIZE;
    double fwd = MPC_WHEELBASE + 0.5 + MPC_CELL_SIZE;
    double rev = 0.5 + MPC_CELL_SIZE;
    double crn[4][2] = {{fwd, hw}, {fwd, -hw}, {-rev, hw}, {-rev, -hw}};

    double mnx = 1e9, mxx = -1e9, mny = 1e9, mxy = -1e9;
    for (int i = 0; i < 4; i++) {
        double wx = c * crn[i][0] - s * crn[i][1] + pose.x;
        double wy = s * crn[i][0] + c * crn[i][1] + pose.y;
        mnx = std::min(mnx, wx); mxx = std::max(mxx, wx);
        mny = std::min(mny, wy); mxy = std::max(mxy, wy);
    }

    int cmin = std::max(0, (int)std::floor(mnx / MPC_CELL_SIZE));
    int cmax = std::min(MPC_GRID_SIZE - 1, (int)std::ceil(mxx / MPC_CELL_SIZE));
    int rmin = std::max(0, (int)std::floor(mny / MPC_CELL_SIZE));
    int rmax = std::min(MPC_GRID_SIZE - 1, (int)std::ceil(mxy / MPC_CELL_SIZE));

    for (int r = rmin; r <= rmax; r++)
        for (int ci = cmin; ci <= cmax; ci++) {
            if (grid_[r][ci] == 0) continue;
            double cx = ci * MPC_CELL_SIZE + MPC_CELL_SIZE / 2.0;
            double cy = r * MPC_CELL_SIZE + MPC_CELL_SIZE / 2.0;
            double dx = cx - pose.x, dy = cy - pose.y;
            double bx = c * dx + s * dy;
            double by = -s * dx + c * dy;
            if (bx >= -rev && bx <= fwd && by >= -hw && by <= hw)
                return true;
        }
    return false;
}

// ===================================================================
//  MPCTrajectoryPlanner
// ===================================================================

MPCTrajectoryPlanner::MPCTrajectoryPlanner(
    const std::vector<std::vector<int>>& grid)
    : cmap_(grid), N_(30), dt_(0.1), L_(MPC_WHEELBASE), v_des_(3.0),
      max_iter_(80), step_size_(0.02),
      w_pos_(10.0), w_theta_(5.0), w_steer_(2.0),
      w_dsteer_(15.0), w_collision_(500.0), w_vel_(1.0)
{}

std::vector<Pose> MPCTrajectoryPlanner::rollout(
    const Pose& start,
    const std::vector<double>& vels,
    const std::vector<double>& steers) const {
    std::vector<Pose> traj(N_ + 1);
    traj[0] = start;
    for (int k = 0; k < N_; k++) {
        double x = traj[k].x, y = traj[k].y, theta = traj[k].theta;
        double v = vels[k], delta = steers[k];
        delta = std::max(-MPC_MAX_STEER, std::min(MPC_MAX_STEER, delta));
        if (std::abs(delta) < 1e-6) {
            traj[k+1].x = x + v * std::cos(theta) * dt_;
            traj[k+1].y = y + v * std::sin(theta) * dt_;
            traj[k+1].theta = theta;
        } else {
            double R = L_ / std::tan(delta);
            double dtheta = v * dt_ / R;
            traj[k+1].x = x + R * (std::sin(theta + dtheta) - std::sin(theta));
            traj[k+1].y = y + R * (std::cos(theta) - std::cos(theta + dtheta));
            traj[k+1].theta = theta + dtheta;
        }
    }
    return traj;
}

double MPCTrajectoryPlanner::collisionCost(const Pose& pose) const {
    double min_dist = 5.0;
    int search_radius = static_cast<int>(min_dist / MPC_CELL_SIZE) + 1;
    int cr = cmap_.worldToRow(pose.y);
    int cc = cmap_.worldToCol(pose.x);
    for (int dr = -search_radius; dr <= search_radius; dr++)
        for (int dc = -search_radius; dc <= search_radius; dc++) {
            int r = cr + dr, c = cc + dc;
            if (r < 0 || r >= MPC_GRID_SIZE || c < 0 || c >= MPC_GRID_SIZE)
                continue;
            if (cmap_.grid()[r][c] == 0) continue;
            double cx = cmap_.colToWorldX(c);
            double cy = cmap_.rowToWorldY(r);
            double dist = std::hypot(cx - pose.x, cy - pose.y);
            if (dist < min_dist) min_dist = dist;
        }
    double safe_dist = 1.5;
    if (min_dist < safe_dist) {
        double d = std::max(0.05, min_dist);
        return 1.0 / (d * d);
    }
    return 0.0;
}

double MPCTrajectoryPlanner::computeCost(
    const std::vector<Pose>& traj,
    const std::vector<double>& vels,
    const std::vector<double>& steers,
    const std::vector<Pose>& ref) const {
    double cost = 0.0;
    for (int k = 0; k <= N_; k++) {
        double dx = traj[k].x - ref[k].x;
        double dy = traj[k].y - ref[k].y;
        cost += w_pos_ * (dx * dx + dy * dy);
        double dth = traj[k].theta - ref[k].theta;
        while (dth > M_PI) dth -= 2 * M_PI;
        while (dth < -M_PI) dth += 2 * M_PI;
        cost += w_theta_ * dth * dth;
        cost += w_collision_ * collisionCost(traj[k]);
        if (k < N_) {
            cost += w_steer_ * steers[k] * steers[k];
            if (k > 0) {
                double dsteer = steers[k] - steers[k-1];
                cost += w_dsteer_ * dsteer * dsteer;
            }
            double dv = vels[k] - v_des_;
            cost += w_vel_ * dv * dv;
        }
    }
    return cost;
}

Pose MPCTrajectoryPlanner::getLookahead(
    const std::vector<Pose>& path, const Pose& current,
    double L_ahead) const {
    double best_dist = 1e9;
    size_t best_i = 0;
    for (size_t i = 0; i < path.size(); i++) {
        double d = std::hypot(path[i].x - current.x, path[i].y - current.y);
        if (d < best_dist) { best_dist = d; best_i = i; }
    }
    double cum = 0;
    for (size_t i = best_i; i + 1 < path.size(); i++) {
        double seg = std::hypot(path[i+1].x - path[i].x,
                                path[i+1].y - path[i].y);
        if (cum + seg >= L_ahead) {
            double t = (L_ahead - cum) / seg;
            Pose lp;
            lp.x = path[i].x + t * (path[i+1].x - path[i].x);
            lp.y = path[i].y + t * (path[i+1].y - path[i].y);
            lp.theta = path[i].theta;
            return lp;
        }
        cum += seg;
    }
    return path.back();
}

void MPCTrajectoryPlanner::trackWithPurePursuit(
    const Pose& start,
    const std::vector<Pose>& ref,
    std::vector<double>& out_vels,
    std::vector<double>& out_steers,
    std::vector<Pose>& out_traj) {
    out_traj.resize(N_ + 1);
    out_traj[0] = start;
    out_vels.assign(N_, v_des_);
    out_steers.assign(N_, 0.0);

    const double MAX_LAT_ACC = 2.5;
    const double MIN_LOOKAHEAD = 1.0;
    const double MAX_LOOKAHEAD = 5.0;
    const double LOOKAHEAD_RATIO = 0.35;

    Pose cur = start;

    std::vector<double> ref_arc(ref.size(), 0);
    std::vector<double> ref_kappa(ref.size(), 0);
    for (size_t i = 1; i < ref.size(); i++) {
        ref_arc[i] = ref_arc[i-1] + std::hypot(
            ref[i].x - ref[i-1].x, ref[i].y - ref[i-1].y);
        double dth = ref[i].theta - ref[i-1].theta;
        while (dth > M_PI) dth -= 2 * M_PI;
        while (dth < -M_PI) dth += 2 * M_PI;
        double ds = ref_arc[i] - ref_arc[i-1];
        ref_kappa[i] = (ds > 1e-6) ? dth / ds : 0;
    }

    auto nearestIdx = [&](const Pose& p) -> size_t {
        double best = 1e9; size_t bi = 0;
        for (size_t i = 0; i < ref.size(); i++) {
            double d = std::hypot(ref[i].x - p.x, ref[i].y - p.y);
            if (d < best) { best = d; bi = i; }
        }
        return bi;
    };

    for (int k = 0; k < N_; k++) {
        double speed = (k > 0) ? std::hypot(
            out_traj[k].x - out_traj[k-1].x,
            out_traj[k].y - out_traj[k-1].y) / dt_ : v_des_;
        double L_ahead = std::max(MIN_LOOKAHEAD,
                          std::min(MAX_LOOKAHEAD, speed * LOOKAHEAD_RATIO));

        size_t ni = nearestIdx(cur);
        double max_k = 0;
        double pa = ref_arc[ni] + L_ahead + 3.0;
        for (size_t i = ni; i < ref.size() && ref_arc[i] < pa; i++)
            max_k = std::max(max_k, std::abs(ref_kappa[i]));
        double v_lim = (max_k > 0.05)
            ? std::sqrt(MAX_LAT_ACC / max_k) * 0.75 : v_des_;

        Pose lp = getLookahead(ref, cur, L_ahead);
        double dx = lp.x - cur.x, dy = lp.y - cur.y;
        double alpha = std::atan2(dy, dx) - cur.theta;
        while (alpha > M_PI) alpha -= 2 * M_PI;
        while (alpha < -M_PI) alpha += 2 * M_PI;
        double Ld = std::hypot(dx, dy);
        if (Ld < 0.01) Ld = L_ahead;

        double d_pp = std::atan2(2.0 * L_ * std::sin(alpha), Ld);
        double d_ff = std::atan(L_ * ref_kappa[ni]);
        double delta = 0.5 * d_pp + 0.5 * d_ff;
        delta = std::max(-MPC_MAX_STEER, std::min(MPC_MAX_STEER, delta));
        double v = std::max(1.0, std::min(v_des_, v_lim));

        out_steers[k] = delta;
        out_vels[k] = v;

        if (std::abs(delta) < 1e-6) {
            out_traj[k+1].x = cur.x + v * std::cos(cur.theta) * dt_;
            out_traj[k+1].y = cur.y + v * std::sin(cur.theta) * dt_;
            out_traj[k+1].theta = cur.theta;
        } else {
            double R = L_ / std::tan(delta);
            double dtheta = v * dt_ / R;
            out_traj[k+1].x = cur.x + R
                * (std::sin(cur.theta + dtheta) - std::sin(cur.theta));
            out_traj[k+1].y = cur.y + R
                * (std::cos(cur.theta) - std::cos(cur.theta + dtheta));
            out_traj[k+1].theta = cur.theta + dtheta;
        }
        cur = out_traj[k+1];
    }
}

std::vector<Pose> MPCTrajectoryPlanner::plan(
    const std::vector<Pose>& ref_path,
    std::vector<double>& out_velocities,
    std::vector<double>& out_steers) {
    std::vector<Pose> traj;
    trackWithPurePursuit(ref_path[0], ref_path,
                         out_velocities, out_steers, traj);
    return traj;
}
