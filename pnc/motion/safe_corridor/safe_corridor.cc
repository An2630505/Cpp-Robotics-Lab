#include "safe_corridor.h"
#include <cmath>
#include <algorithm>
#include <utility>

SafeCorridor::SafeCorridor() {}

// ===================================================================
//  build() — 基于占用栅格逐 cell 扫描构建安全走廊
// ===================================================================
std::vector<CorridorSection> SafeCorridor::build(
    const std::vector<Pose>& ref_path,
    const std::vector<std::vector<int>>& grid,
    double x_min, double y_min,
    double cell_size, int cols, int rows) {

    std::vector<CorridorSection> result;
    if (ref_path.size() < 2 || grid.empty()) return result;

    // 世界坐标 → 栅格索引
    auto worldToGrid = [&](double wx, double wy) -> std::pair<int,int> {
        int c = static_cast<int>((wx - x_min) / cell_size);
        int r = static_cast<int>((wy - y_min) / cell_size);
        return {r, c};
    };

    // Step 1: 沿 ref_path 等弧长采样
    int n = static_cast<int>(ref_path.size());

    std::vector<double> cum_len(n, 0.0);
    for (int i = 1; i < n; i++) {
        double dx = ref_path[i].x - ref_path[i-1].x;
        double dy = ref_path[i].y - ref_path[i-1].y;
        cum_len[i] = cum_len[i-1] + std::sqrt(dx*dx + dy*dy);
    }
    double total_len = cum_len.back();
    if (total_len < 1e-6) return result;

    int num_samples = std::max(2, static_cast<int>(total_len / sample_interval_) + 1);

    auto interpolate = [&](double s) -> Pose {
        if (s <= 0.0) return ref_path[0];
        if (s >= total_len) return ref_path.back();
        size_t lo = 0, hi = n - 1;
        while (lo + 1 < hi) {
            size_t mid = (lo + hi) / 2;
            if (cum_len[mid] <= s) lo = mid;
            else hi = mid;
        }
        double seg = cum_len[hi] - cum_len[lo];
        double t = (seg > 1e-12) ? (s - cum_len[lo]) / seg : 0.0;
        return {
            (1.0 - t) * ref_path[lo].x + t * ref_path[hi].x,
            (1.0 - t) * ref_path[lo].y + t * ref_path[hi].y,
            ref_path[lo].theta
        };
    };

    // 最大扫描步数 (防止死循环)
    double max_scan_dist = std::max(
        (x_min + cols * cell_size) - x_min,
        (y_min + rows * cell_size) - y_min);
    int max_steps = static_cast<int>(max_scan_dist / cell_size) + 10;

    for (int i = 0; i < num_samples; i++) {
        double s = (total_len * i) / (num_samples - 1);
        Pose pt = interpolate(s);

        // Step 2: 计算切线方向
        double ds = std::max(0.5, sample_interval_ * 0.5);
        Pose pt_fwd = interpolate(std::min(total_len, s + ds));
        Pose pt_bwd = interpolate(std::max(0.0, s - ds));
        double tx = pt_fwd.x - pt_bwd.x;
        double ty = pt_fwd.y - pt_bwd.y;
        double tnorm = std::sqrt(tx*tx + ty*ty);
        if (tnorm < 1e-9) {
            tx = std::cos(pt.theta);
            ty = std::sin(pt.theta);
        } else {
            tx /= tnorm; ty /= tnorm;
        }

        // 左法向 (+90°) 和右法向 (-90°)
        Vec2d n_left  = { -ty,  tx };
        Vec2d n_right = {  ty, -tx };

        // Step 3: 沿法向逐 cell 扫描
        auto scan = [&](const Vec2d& dir) -> double {
            for (int step = 1; step < max_steps; step++) {
                double wx = pt.x + dir.x * step * cell_size;
                double wy = pt.y + dir.y * step * cell_size;
                auto [r, c] = worldToGrid(wx, wy);
                if (r < 0 || r >= rows || c < 0 || c >= cols) {
                    return (step - 1) * cell_size;
                }
                if (grid[r][c] == 1) {
                    return (step - 1) * cell_size;
                }
            }
            return (max_steps - 1) * cell_size;
        };

        double d_left  = scan(n_left);
        double d_right = scan(n_right);

        // Step 4: 减安全边距
        d_left  = std::max(0.0, d_left  - margin_);
        d_right = std::max(0.0, d_right - margin_);

        // Step 5: 存储截面
        CorridorSection sec;
        sec.center = { pt.x, pt.y };
        sec.left   = { pt.x + n_left.x  * d_left,
                       pt.y + n_left.y  * d_left };
        sec.right  = { pt.x + n_right.x * d_right,
                       pt.y + n_right.y * d_right };
        result.push_back(sec);
    }

    return result;
}
