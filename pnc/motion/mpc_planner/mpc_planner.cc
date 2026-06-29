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
    : grid_(grid),
      rows_(static_cast<int>(grid.size())),
      cols_(grid.empty() ? 0 : static_cast<int>(grid[0].size())) {}

ContinuousMap::ContinuousMap(const std::vector<std::vector<int>>& grid,
                             double x_min, double y_min, double cell_size)
    : grid_(grid), x_min_(x_min), y_min_(y_min), cell_size_(cell_size),
      rows_(static_cast<int>(grid.size())),
      cols_(grid.empty() ? 0 : static_cast<int>(grid[0].size())) {}

void ContinuousMap::setOrigin(double x_min, double y_min, double cell_size) {
    x_min_ = x_min; y_min_ = y_min; cell_size_ = cell_size;
    rows_ = static_cast<int>(grid_.size());
    cols_ = grid_.empty() ? 0 : static_cast<int>(grid_[0].size());
}

int ContinuousMap::worldToRow(double y) const {
    int row = static_cast<int>((y - y_min_) / cell_size_);
    return std::max(0, std::min(rows_ - 1, row));
}
int ContinuousMap::worldToCol(double x) const {
    int col = static_cast<int>((x - x_min_) / cell_size_);
    return std::max(0, std::min(cols_ - 1, col));
}
double ContinuousMap::rowToWorldY(int row) const {
    return y_min_ + row * cell_size_ + cell_size_ / 2.0;
}
double ContinuousMap::colToWorldX(int col) const {
    return x_min_ + col * cell_size_ + cell_size_ / 2.0;
}
bool ContinuousMap::isOccupied(int row, int col) const {
    if (row < 0 || row >= rows_ || col < 0 || col >= cols_)
        return true;
    return grid_[row][col] == 1;
}

bool ContinuousMap::isCollision(const Pose& pose) const {
    double c = std::cos(pose.theta), s = std::sin(pose.theta);
    double pad = cell_size_;
    double hw = MPC_CAR_WIDTH / 2.0 + pad;
    double fwd = MPC_WHEELBASE + 0.5 + pad;
    double rev = 0.5 + pad;
    double crn[4][2] = {{fwd, hw}, {fwd, -hw}, {-rev, hw}, {-rev, -hw}};

    double mnx = 1e9, mxx = -1e9, mny = 1e9, mxy = -1e9;
    for (int i = 0; i < 4; i++) {
        double wx = c * crn[i][0] - s * crn[i][1] + pose.x;
        double wy = s * crn[i][0] + c * crn[i][1] + pose.y;
        mnx = std::min(mnx, wx); mxx = std::max(mxx, wx);
        mny = std::min(mny, wy); mxy = std::max(mxy, wy);
    }

    int cmin = std::max(0, static_cast<int>(std::floor((mnx - x_min_) / cell_size_)));
    int cmax = std::min(cols_ - 1, static_cast<int>(std::ceil((mxx - x_min_) / cell_size_)));
    int rmin = std::max(0, static_cast<int>(std::floor((mny - y_min_) / cell_size_)));
    int rmax = std::min(rows_ - 1, static_cast<int>(std::ceil((mxy - y_min_) / cell_size_)));

    for (int r = rmin; r <= rmax; r++)
        for (int ci = cmin; ci <= cmax; ci++) {
            if (grid_[r][ci] == 0) continue;
            double cx = colToWorldX(ci);
            double cy = rowToWorldY(r);
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
      max_iter_(80), step_size_(0.02), t_start_(0.0),
      w_pos_(10.0), w_theta_(5.0), w_steer_(2.0),
      w_dsteer_(15.0), w_collision_(500.0), w_vel_(1.0)
{}

void MPCTrajectoryPlanner::setGridOrigin(double x_min, double y_min,
                                         double cell_size) {
    cmap_.setOrigin(x_min, y_min, cell_size);
}

void MPCTrajectoryPlanner::setCostWeights(double w_pos, double w_theta,
    double w_steer, double w_dsteer, double w_collision, double w_vel) {
    w_pos_ = w_pos; w_theta_ = w_theta; w_steer_ = w_steer;
    w_dsteer_ = w_dsteer; w_collision_ = w_collision; w_vel_ = w_vel;
}

void MPCTrajectoryPlanner::addDynamicObstacle(
    std::shared_ptr<DynamicObstacle> obs) {
    dynamic_obstacles_.push_back(obs);
}

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

double MPCTrajectoryPlanner::collisionCost(const Pose& pose, int step_k) const {
    double min_dist = 5.0;
    double safe_dist = 1.5;
    double cost = 0.0;

    // 静态栅格扫描 (道路边界、静态障碍物)
    int search_radius = static_cast<int>(min_dist / cmap_.cellSize()) + 1;
    int cr = cmap_.worldToRow(pose.y);
    int cc = cmap_.worldToCol(pose.x);
    for (int dr = -search_radius; dr <= search_radius; dr++)
        for (int dc = -search_radius; dc <= search_radius; dc++) {
            int r = cr + dr, c = cc + dc;
            if (r < 0 || r >= cmap_.rows() || c < 0 || c >= cmap_.cols())
                continue;
            if (cmap_.grid()[r][c] == 0) continue;
            double cx = cmap_.colToWorldX(c);
            double cy = cmap_.rowToWorldY(r);
            double dist = std::hypot(cx - pose.x, cy - pose.y);
            if (dist < min_dist) min_dist = dist;
        }
    if (min_dist < safe_dist) {
        double d = std::max(0.05, min_dist);
        cost += 1.0 / (d * d);
    }

    // 动态障碍物检测: 用 step_k 预测时刻
    if (step_k >= 0 && !dynamic_obstacles_.empty()) {
        double t_k = t_start_ + step_k * dt_;
        for (auto& obs : dynamic_obstacles_) {
            Vec2d pos = obs->predict(t_k);
            double dist = std::hypot(pose.x - pos.x, pose.y - pos.y);
            if (dist < safe_dist) {
                double d = std::max(0.05, dist);
                cost += 1.0 / (d * d);
            }
        }
    }

    return cost;
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
        cost += w_collision_ * collisionCost(traj[k], k);
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

double MPCTrajectoryPlanner::computeCostPartial(
    const std::vector<Pose>& traj,
    const std::vector<double>& vels,
    const std::vector<double>& steers,
    const std::vector<Pose>& ref,
    int start_k) const {
    double cost = 0.0;
    for (int k = start_k; k <= N_; k++) {
        double dx = traj[k].x - ref[k].x;
        double dy = traj[k].y - ref[k].y;
        cost += w_pos_ * (dx * dx + dy * dy);
        double dth = traj[k].theta - ref[k].theta;
        while (dth > M_PI) dth -= 2 * M_PI;
        while (dth < -M_PI) dth += 2 * M_PI;
        cost += w_theta_ * dth * dth;
        cost += w_collision_ * collisionCost(traj[k], k);
        if (k < N_) {
            cost += w_steer_ * steers[k] * steers[k];
            if (k > start_k) {
                double dsteer = steers[k] - steers[k-1];
                cost += w_dsteer_ * dsteer * dsteer;
            }
            double dv = vels[k] - v_des_;
            cost += w_vel_ * dv * dv;
        }
    }
    return cost;
}

std::vector<Pose> MPCTrajectoryPlanner::extractRefWindow(
    const std::vector<Pose>& ref_path, const Pose& start) const {
    std::vector<Pose> window(static_cast<size_t>(N_) + 1);
    window[0] = start;

    if (ref_path.size() < 2) {
        for (int k = 1; k <= N_; k++) window[k] = start;
        return window;
    }

    // 计算 ref_path 累积弧长
    std::vector<double> arc(ref_path.size(), 0.0);
    for (size_t i = 1; i < ref_path.size(); i++)
        arc[i] = arc[i-1] + std::hypot(ref_path[i].x - ref_path[i-1].x,
                                         ref_path[i].y - ref_path[i-1].y);
    double total_arc = arc.back();
    double horizon_dist = v_des_ * N_ * dt_;

    for (int k = 1; k <= N_; k++) {
        double s_target = horizon_dist * k / N_;
        if (s_target >= total_arc || total_arc < 1e-9) {
            window[k] = ref_path.back();
            continue;
        }
        auto it = std::lower_bound(arc.begin(), arc.end(), s_target);
        size_t idx = std::min(static_cast<size_t>(it - arc.begin()),
                              ref_path.size() - 1);
        if (idx == 0) {
            window[k] = ref_path[0];
        } else {
            double s0 = arc[idx-1], s1 = arc[idx];
            double t = (s1 > s0) ? (s_target - s0) / (s1 - s0) : 0.0;
            window[k].x = ref_path[idx-1].x + t * (ref_path[idx].x - ref_path[idx-1].x);
            window[k].y = ref_path[idx-1].y + t * (ref_path[idx].y - ref_path[idx-1].y);
            window[k].theta = ref_path[idx-1].theta;
        }
    }
    return window;
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

// ===================================================================
//  运动学自行车单步传播 (内联辅助, 避免 rolloutFrom 中的重复代码)
// ===================================================================
namespace {
inline void kinStep(double& x, double& y, double& theta,
                     double v, double delta, double L, double dt) {
    delta = std::max(-MPC_MAX_STEER, std::min(MPC_MAX_STEER, delta));
    if (std::abs(delta) < 1e-6) {
        x += v * std::cos(theta) * dt;
        y += v * std::sin(theta) * dt;
    } else {
        double R = L / std::tan(delta);
        double dtheta = v * dt / R;
        x += R * (std::sin(theta + dtheta) - std::sin(theta));
        y += R * (std::cos(theta) - std::cos(theta + dtheta));
        theta += dtheta;
    }
}

// 从 traj[start_k] 出发, 用修改后的控制序列 rollout 剩余轨迹
void rolloutFrom(std::vector<Pose>& traj,
                 const std::vector<double>& vels,
                 const std::vector<double>& steers,
                 int start_k, int N, double L, double dt) {
    for (int k = start_k; k < N; k++) {
        double x = traj[k].x, y = traj[k].y, theta = traj[k].theta;
        kinStep(x, y, theta, vels[k], steers[k], L, dt);
        traj[k+1].x = x;
        traj[k+1].y = y;
        traj[k+1].theta = theta;
    }
}
} // namespace

std::vector<Pose> MPCTrajectoryPlanner::plan(
    const std::vector<Pose>& ref_path,
    std::vector<double>& out_velocities,
    std::vector<double>& out_steers,
    double t_start) {

    t_start_ = t_start;
    Pose start = ref_path[0];

    // ---- 1. 纯追踪初始解 ----
    std::vector<Pose> traj;
    trackWithPurePursuit(start, ref_path, out_velocities, out_steers, traj);

    // ---- 2. 提取参考窗口 ----
    std::vector<Pose> ref_window = extractRefWindow(ref_path, start);

    // ---- 3. 梯度下降优化 ----
    std::vector<double> vels = out_velocities;
    std::vector<double> steers = out_steers;
    std::vector<double> best_vels = vels;
    std::vector<double> best_steers = steers;
    double current_cost = computeCost(traj, vels, steers, ref_window);
    double best_cost = current_cost;

    const double grad_eps = 1e-3;
    double lr = step_size_;

    for (int iter = 0; iter < max_iter_; iter++) {
        // ---- 3a. 有限差分梯度 ----
        for (int k = 0; k < N_; k++) {
            // dCost/dSteer[k]
            {
                double orig = steers[k];
                double cost_plus = 0.0, cost_minus = 0.0;

                // 正扰动
                steers[k] = std::min(MPC_MAX_STEER, orig + grad_eps);
                {
                    std::vector<Pose> traj_pert = traj;
                    rolloutFrom(traj_pert, vels, steers, k, N_, L_, dt_);
                    cost_plus = computeCostPartial(traj_pert, vels, steers,
                                                   ref_window, k);
                }
                // 负扰动
                steers[k] = std::max(-MPC_MAX_STEER, orig - grad_eps);
                {
                    std::vector<Pose> traj_pert = traj;
                    rolloutFrom(traj_pert, vels, steers, k, N_, L_, dt_);
                    cost_minus = computeCostPartial(traj_pert, vels, steers,
                                                    ref_window, k);
                }
                double grad = (cost_plus - cost_minus) / (2.0 * grad_eps);

                // 梯度步 (仅更新控制量, 不做 rollout)
                steers[k] = orig - lr * grad;
                steers[k] = std::max(-MPC_MAX_STEER,
                           std::min(MPC_MAX_STEER, steers[k]));
            }

            // dCost/dVel[k]
            {
                double orig = vels[k];
                double cost_plus = 0.0, cost_minus = 0.0;

                // 正扰动
                vels[k] = std::min(v_des_ * 1.5, orig + grad_eps);
                {
                    std::vector<Pose> traj_pert = traj;
                    rolloutFrom(traj_pert, vels, steers, k, N_, L_, dt_);
                    cost_plus = computeCostPartial(traj_pert, vels, steers,
                                                   ref_window, k);
                }
                // 负扰动
                vels[k] = std::max(0.5, orig - grad_eps);
                {
                    std::vector<Pose> traj_pert = traj;
                    rolloutFrom(traj_pert, vels, steers, k, N_, L_, dt_);
                    cost_minus = computeCostPartial(traj_pert, vels, steers,
                                                    ref_window, k);
                }
                double grad = (cost_plus - cost_minus) / (2.0 * grad_eps);

                // 梯度步 (vel 用更小的步长)
                vels[k] = orig - lr * 0.1 * grad;
                vels[k] = std::max(0.5, std::min(v_des_ * 1.5, vels[k]));
            }
        }

        // ---- 3b. 学习率衰减 ----
        double effective_lr = lr / (1.0 + 0.1 * iter);

        // 应用衰减后的学习率重新做梯度步 (用 best 控制量作为基点)
        for (int k = 0; k < N_; k++) {
            steers[k] = best_steers[k];
            vels[k] = best_vels[k];
        }
        // 重新计算梯度并应用 effective_lr
        for (int k = 0; k < N_; k++) {
            // steer 梯度
            {
                double orig = best_steers[k];
                double cost_plus = 0.0, cost_minus = 0.0;

                steers[k] = std::min(MPC_MAX_STEER, orig + grad_eps);
                {
                    std::vector<Pose> traj_pert = traj;
                    rolloutFrom(traj_pert, vels, steers, k, N_, L_, dt_);
                    cost_plus = computeCostPartial(traj_pert, vels, steers,
                                                   ref_window, k);
                }
                steers[k] = std::max(-MPC_MAX_STEER, orig - grad_eps);
                {
                    std::vector<Pose> traj_pert = traj;
                    rolloutFrom(traj_pert, vels, steers, k, N_, L_, dt_);
                    cost_minus = computeCostPartial(traj_pert, vels, steers,
                                                    ref_window, k);
                }
                double grad = (cost_plus - cost_minus) / (2.0 * grad_eps);
                steers[k] = orig - effective_lr * grad;
                steers[k] = std::max(-MPC_MAX_STEER,
                           std::min(MPC_MAX_STEER, steers[k]));
            }

            // vel 梯度
            {
                double orig = best_vels[k];
                double cost_plus = 0.0, cost_minus = 0.0;

                vels[k] = std::min(v_des_ * 1.5, orig + grad_eps);
                {
                    std::vector<Pose> traj_pert = traj;
                    rolloutFrom(traj_pert, vels, steers, k, N_, L_, dt_);
                    cost_plus = computeCostPartial(traj_pert, vels, steers,
                                                   ref_window, k);
                }
                vels[k] = std::max(0.5, orig - grad_eps);
                {
                    std::vector<Pose> traj_pert = traj;
                    rolloutFrom(traj_pert, vels, steers, k, N_, L_, dt_);
                    cost_minus = computeCostPartial(traj_pert, vels, steers,
                                                    ref_window, k);
                }
                double grad = (cost_plus - cost_minus) / (2.0 * grad_eps);
                vels[k] = orig - effective_lr * 0.1 * grad;
                vels[k] = std::max(0.5, std::min(v_des_ * 1.5, vels[k]));
            }
        }

        // ---- 3c. 全轨迹 rollout + 代价评估 ----
        traj = rollout(start, vels, steers);
        double new_cost = computeCost(traj, vels, steers, ref_window);

        // ---- 3d. 收敛 / backtrack ----
        if (std::abs(new_cost - current_cost) < 1e-5) {
            if (new_cost < best_cost) {
                best_cost = new_cost;
                best_vels = vels; best_steers = steers;
            }
            break;
        }
        if (new_cost > current_cost) {
            lr *= 0.5;
            vels = best_vels; steers = best_steers;
        } else {
            current_cost = new_cost;
            if (new_cost < best_cost) {
                best_cost = new_cost;
                best_vels = vels; best_steers = steers;
            }
        }
    }

    out_velocities = best_vels;
    out_steers = best_steers;
    return rollout(start, out_velocities, out_steers);
}
