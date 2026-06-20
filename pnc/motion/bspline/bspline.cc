#include "bspline.h"
#include <cmath>
#include <algorithm>
#include <iostream>

BSpline::BSpline() {}

void BSpline::setParams(const BSplineParams& p) { params_ = p; }

// ===================================================================
//  Cox-de Boor 递推基函数
// ===================================================================
double BSpline::basis(int i, int k, double t,
                       const std::vector<double>& knots) const {
    if (k == 0) {
        // 右端点: 最后一跨包含 t == knots[i+1]
        if (t >= knots[i] && t < knots[i + 1]) return 1.0;
        if (t == knots.back() && i == (int)knots.size() - 2
            && t == knots[i + 1]) return 1.0;
        return 0.0;
    }
    double left = 0.0, right = 0.0;
    double denom1 = knots[i + k] - knots[i];
    double denom2 = knots[i + k + 1] - knots[i + 1];
    if (denom1 > 1e-12)
        left = (t - knots[i]) / denom1 * basis(i, k - 1, t, knots);
    if (denom2 > 1e-12)
        right = (knots[i + k + 1] - t) / denom2 * basis(i + 1, k - 1, t, knots);
    return left + right;
}

// ===================================================================
//  评估 B 样条在参数 t 处的 (x, y)
// ===================================================================
Eigen::Vector2d BSpline::eval(double t, const std::vector<double>& knots,
                                const Eigen::MatrixX2d& ctrl_pts) const {
    int n = static_cast<int>(ctrl_pts.rows());
    Eigen::Vector2d pt(0.0, 0.0);
    for (int i = 0; i < n; i++) {
        double b = basis(i, params_.degree, t, knots);
        pt(0) += b * ctrl_pts(i, 0);
        pt(1) += b * ctrl_pts(i, 1);
    }
    return pt;
}

// ===================================================================
//  周期 knot vector
// ===================================================================
std::vector<double> BSpline::periodicKnots(int n_ctrl, int degree) const {
    // Closed B-spline: uniform knots 0..n_ctrl+degree
    int m = n_ctrl + degree;  // total knots - 1
    std::vector<double> knots(m + 1);
    for (int i = 0; i <= m; i++)
        knots[i] = static_cast<double>(i);
    return knots;
}

// ===================================================================
//  点到线段投影
// ===================================================================
Vec2d BSpline::projectToSegment(double px, double py,
                                  double ax, double ay,
                                  double bx, double by) {
    double abx = bx - ax, aby = by - ay;
    double apx = px - ax, apy = py - ay;
    double ab2 = abx * abx + aby * aby;
    if (ab2 < 1e-12) return {ax, ay};
    double t = (apx * abx + apy * aby) / ab2;
    if (t < 0.0) return {ax, ay};
    if (t > 1.0) return {bx, by};
    return {ax + t * abx, ay + t * aby};
}

// ===================================================================
//  fit() — B 样条最小二乘拟合 + 走廊约束
// ===================================================================
std::vector<Pose> BSpline::fit(
    const std::vector<Pose>& ref_path,
    const std::vector<CorridorSection>& corridors) {

    int n_orig = static_cast<int>(ref_path.size());
    if (n_orig < 2) return ref_path;

    int degree  = params_.degree;
    int n_ctrl  = std::max(degree + 1, params_.num_control_points);
    bool closed = params_.closed;

    // Step 1: 沿 ref_path 等弧长采样 N 个点
    int N = n_ctrl;  // 至少与 control points 数量相同
    if (N < n_orig) N = n_orig;  // 如果原始点多，保留

    std::vector<double> cum_len(n_orig, 0.0);
    for (int i = 1; i < n_orig; i++) {
        double dx = ref_path[i].x - ref_path[i-1].x;
        double dy = ref_path[i].y - ref_path[i-1].y;
        cum_len[i] = cum_len[i-1] + std::sqrt(dx*dx + dy*dy);
    }
    double total_len = cum_len.back();
    if (total_len < 1e-6) return ref_path;

    Eigen::MatrixX2d samples(N, 2);
    for (int i = 0; i < N; i++) {
        double s = total_len * i / N;
        // 二分查找弧长所在段
        int lo = 0, hi = n_orig - 1;
        while (lo + 1 < hi) {
            int mid = (lo + hi) / 2;
            if (cum_len[mid] <= s) lo = mid;
            else hi = mid;
        }
        double seg = cum_len[hi] - cum_len[lo];
        double t = (seg > 1e-12) ? (s - cum_len[lo]) / seg : 0.0;
        samples(i, 0) = (1.0 - t) * ref_path[lo].x + t * ref_path[hi].x;
        samples(i, 1) = (1.0 - t) * ref_path[lo].y + t * ref_path[hi].y;
    }

    // Step 2: 构建 knot vector
    std::vector<double> knots;
    double t_min, t_max;
    if (closed) {
        knots = periodicKnots(n_ctrl, degree);
        t_min = knots[degree];
        t_max = knots[n_ctrl];
    } else {
        // Clamped knot vector
        int m = n_ctrl + degree;
        knots.resize(m + 1);
        for (int i = 0; i <= degree; i++)    knots[i] = 0.0;
        for (int i = degree + 1; i < n_ctrl; i++)
            knots[i] = static_cast<double>(i - degree) / (n_ctrl - degree);
        for (int i = n_ctrl; i <= m; i++)    knots[i] = 1.0;
        t_min = 0.0;
        t_max = 1.0;
    }

    // Step 3: 构建基函数矩阵 B (N x n_ctrl)
    Eigen::MatrixXd B_mat(N, n_ctrl);
    for (int i = 0; i < N; i++) {
        double param = t_min + (t_max - t_min) * i / std::max(1, N - 1);
        for (int j = 0; j < n_ctrl; j++) {
            B_mat(i, j) = basis(j, degree, param, knots);
        }
    }

    // Step 4: 最小二乘求解控制点 P: (B^T B) P = B^T S
    Eigen::MatrixXd BtB = B_mat.transpose() * B_mat;
    Eigen::MatrixXd BtS = B_mat.transpose() * samples;
    // 加微小正则化避免奇异
    BtB += Eigen::MatrixXd::Identity(n_ctrl, n_ctrl) * 1e-6;
    Eigen::MatrixX2d ctrl_pts = BtB.colPivHouseholderQr().solve(BtS);

    // Step 5: Closed-loop wrapping — 首尾 degree 个控制点均值
    if (closed) {
        int m_unique = n_ctrl - degree;
        for (int i = 0; i < degree; i++) {
            double ax = (ctrl_pts(i, 0) + ctrl_pts(m_unique + i, 0)) * 0.5;
            double ay = (ctrl_pts(i, 1) + ctrl_pts(m_unique + i, 1)) * 0.5;
            ctrl_pts(i, 0) = ax;
            ctrl_pts(i, 1) = ay;
            ctrl_pts(m_unique + i, 0) = ax;
            ctrl_pts(m_unique + i, 1) = ay;
        }
    }

    // Step 6: 走廊约束投影 (软约束: 迭代 2 次)
    if (!corridors.empty()) {
        int n_cor = static_cast<int>(corridors.size());
        for (int iter = 0; iter < 2; iter++) {
            // 密集评估
            int n_eval = std::max(200, N * 3);
            std::vector<Eigen::Vector2d> eval_pts(n_eval);
            for (int i = 0; i < n_eval; i++) {
                double param = t_min + (t_max - t_min) * i / (n_eval - 1);
                eval_pts[i] = eval(param, knots, ctrl_pts);
            }

            // 检查每个评估点是否超出走廊
            std::vector<Eigen::Vector2d> projected = eval_pts;
            bool any_violation = false;
            for (int i = 0; i < n_eval; i++) {
                double ex = eval_pts[i](0), ey = eval_pts[i](1);

                // 找最近的 corridor section
                double best_dist = 1e9;
                int best_idx = 0;
                for (int j = 0; j < n_cor; j++) {
                    double dx = ex - corridors[j].center.x;
                    double dy = ey - corridors[j].center.y;
                    double d = dx*dx + dy*dy;
                    if (d < best_dist) { best_dist = d; best_idx = j; }
                }

                const auto& sec = corridors[best_idx];
                // 左法向: (-tangent_y, tangent_x) — 使用 left 边界方向
                double nlx = sec.left.x - sec.center.x;
                double nly = sec.left.y - sec.center.y;
                double left_len = std::sqrt(nlx*nlx + nly*nly);
                double nrx = sec.right.x - sec.center.x;
                double nry = sec.right.y - sec.center.y;
                double right_len = std::sqrt(nrx*nrx + nry*nry);

                if (left_len < 1e-9 || right_len < 1e-9) continue;

                // 单位向量
                double nlux = nlx / left_len, nluy = nly / left_len;
                double nrux = nrx / right_len, nruy = nry / right_len;

                double dx_c = ex - sec.center.x;
                double dy_c = ey - sec.center.y;

                // 在左法向上的投影
                double proj_l = dx_c * nlux + dy_c * nluy;
                // 在右法向上的投影
                double proj_r = dx_c * nrux + dy_c * nruy;

                bool modified = false;
                if (proj_l > left_len) {
                    // 超出左边界 → 投影到左边界
                    double scale = left_len / std::max(proj_l, 1e-9);
                    ex = sec.center.x + dx_c * scale;
                    ey = sec.center.y + dy_c * scale;
                    modified = true;
                }
                if (proj_r > right_len) {
                    // 超出右边界 → 投影到右边界
                    double scale = right_len / std::max(proj_r, 1e-9);
                    ex = sec.center.x + dx_c * scale;
                    ey = sec.center.y + dy_c * scale;
                    modified = true;
                }
                if (modified) {
                    projected[i](0) = ex;
                    projected[i](1) = ey;
                    any_violation = true;
                }
            }

            if (!any_violation) break;

            // 用投影后的评估点作为新样本，重新拟合
            int n_s = std::min(n_eval, std::max(n_ctrl, 50));
            Eigen::MatrixX2d new_samples(n_s, 2);
            Eigen::MatrixXd new_B(n_s, n_ctrl);
            for (int i = 0; i < n_s; i++) {
                int src = i * (n_eval - 1) / std::max(1, n_s - 1);
                new_samples(i, 0) = projected[src](0);
                new_samples(i, 1) = projected[src](1);
                double param = t_min + (t_max - t_min) * i / std::max(1, n_s - 1);
                for (int j = 0; j < n_ctrl; j++)
                    new_B(i, j) = basis(j, degree, param, knots);
            }
            Eigen::MatrixXd new_BtB = new_B.transpose() * new_B;
            new_BtB += Eigen::MatrixXd::Identity(n_ctrl, n_ctrl) * 1e-6;
            Eigen::MatrixXd new_BtS = new_B.transpose() * new_samples;
            ctrl_pts = new_BtB.colPivHouseholderQr().solve(new_BtS);

            // Re-wrap for closed
            if (closed) {
                int m_unique = n_ctrl - degree;
                for (int i = 0; i < degree; i++) {
                    double ax = (ctrl_pts(i, 0) + ctrl_pts(m_unique + i, 0)) * 0.5;
                    double ay = (ctrl_pts(i, 1) + ctrl_pts(m_unique + i, 1)) * 0.5;
                    ctrl_pts(i, 0) = ax; ctrl_pts(i, 1) = ay;
                    ctrl_pts(m_unique + i, 0) = ax;
                    ctrl_pts(m_unique + i, 1) = ay;
                }
            }
        }
    }

    // Step 7: 密集评估 + 计算朝向
    int n_out = std::max(100, N * 2);
    std::vector<Pose> result;
    for (int i = 0; i < n_out; i++) {
        double param = t_min + (t_max - t_min) * i / n_out;
        Eigen::Vector2d pt = eval(param, knots, ctrl_pts);
        result.push_back({pt(0), pt(1), 0.0});
    }

    // 计算朝向 (有限差分)
    for (size_t i = 0; i < result.size(); i++) {
        size_t nxt = (i + 1) % result.size();
        double dx = result[nxt].x - result[i].x;
        double dy = result[nxt].y - result[i].y;
        result[i].theta = std::atan2(dy, dx);
    }

    return result;
}

// ===================================================================
//  resample() — 等弧长重采样
// ===================================================================
std::vector<Pose> BSpline::resample(const std::vector<Pose>& path) {
    if (path.size() < 2) return path;

    int n = static_cast<int>(path.size());
    double spacing = params_.resample_spacing;
    bool closed = params_.closed;

    // 累积弧长
    std::vector<double> arc(n, 0.0);
    for (int i = 1; i < n; i++) {
        double dx = path[i].x - path[i-1].x;
        double dy = path[i].y - path[i-1].y;
        arc[i] = arc[i-1] + std::sqrt(dx*dx + dy*dy);
    }
    double total = arc.back();

    // 等弧长采样
    std::vector<Pose> result;
    int n_out = closed
        ? static_cast<int>(total / spacing)
        : static_cast<int>(total / spacing) + 1;
    if (n_out < 2) n_out = 2;

    for (int i = 0; i < n_out; i++) {
        double s = i * spacing;
        if (closed) s = std::fmod(s, total);

        if (s <= 0.0) {
            result.push_back(path[0]);
            continue;
        }
        if (s >= total) {
            result.push_back(path.back());
            continue;
        }

        // 二分查找
        int lo = 0, hi = n - 1;
        while (lo + 1 < hi) {
            int mid = (lo + hi) / 2;
            if (arc[mid] <= s) lo = mid;
            else hi = mid;
        }
        double seg = arc[hi] - arc[lo];
        double t = (seg > 1e-12) ? (s - arc[lo]) / seg : 0.0;
        result.push_back({
            (1.0 - t) * path[lo].x + t * path[hi].x,
            (1.0 - t) * path[lo].y + t * path[hi].y,
            path[lo].theta
        });
    }

    // 闭环绕组: 首尾一致
    if (closed && !result.empty() && result.size() > 1) {
        result.back().x = result[0].x;
        result.back().y = result[0].y;
        result.back().theta = result[0].theta;
    }

    // 重新计算朝向
    for (size_t i = 0; i < result.size(); i++) {
        size_t nxt = (i + 1) % result.size();
        double dx = result[nxt].x - result[i].x;
        double dy = result[nxt].y - result[i].y;
        result[i].theta = std::atan2(dy, dx);
    }

    return result;
}
