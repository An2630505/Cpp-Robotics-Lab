#include "safe_corridor.h"
#include <cmath>
#include <algorithm>

SafeCorridor::SafeCorridor() {}

// ===================================================================
//  射线-线段交点
// ===================================================================
bool SafeCorridor::raySegIntersect(const Vec2d& orig, const Vec2d& dir,
                                    const Vec2d& a, const Vec2d& b,
                                    double& t) {
    double seg_x = b.x - a.x, seg_y = b.y - a.y;
    double cross = dir.x * seg_y - dir.y * seg_x;
    if (std::abs(cross) < 1e-12) return false;  // 平行
    double u = ((orig.x - a.x) * dir.y - (orig.y - a.y) * dir.x) / (-cross);
    if (u < 0.0 || u > 1.0) return false;
    t = ((a.x - orig.x) * seg_y - (a.y - orig.y) * seg_x) / (-cross);
    return t >= 0.0;
}

// ===================================================================
//  射线与多边形的最短正方向交距
// ===================================================================
double SafeCorridor::rayPolygonDist(const Vec2d& orig, const Vec2d& dir,
                                     const std::vector<Vec2d>& poly) {
    if (poly.size() < 2) return 1e9;
    double best = 1e9;
    int n = static_cast<int>(poly.size());
    for (int i = 0; i < n; i++) {
        const Vec2d& a = poly[i];
        const Vec2d& b = poly[(i + 1) % n];
        double t = 1e9;
        if (raySegIntersect(orig, dir, a, b, t) && t < best)
            best = t;
    }
    return best;
}

// ===================================================================
//  build() — 构建安全走廊
// ===================================================================
std::vector<CorridorSection> SafeCorridor::build(
    const std::vector<Pose>& ref_path,
    const std::vector<Vec2d>& outer,
    const std::vector<std::vector<Vec2d>>& holes) {

    std::vector<CorridorSection> result;
    if (ref_path.size() < 2) return result;

    // Step 1: 沿 ref_path 等弧长采样
    int n = static_cast<int>(ref_path.size());

    // 累积弧长
    std::vector<double> cum_len(n, 0.0);
    for (int i = 1; i < n; i++) {
        double dx = ref_path[i].x - ref_path[i-1].x;
        double dy = ref_path[i].y - ref_path[i-1].y;
        cum_len[i] = cum_len[i-1] + std::sqrt(dx*dx + dy*dy);
    }
    double total_len = cum_len.back();
    if (total_len < 1e-6) return result;

    int num_samples = std::max(2, static_cast<int>(total_len / sample_interval_) + 1);

    // 在每个采样点计算切线、法向、走廊边界
    auto interpolate = [&](double s) -> Pose {
        if (s <= 0.0) return ref_path[0];
        if (s >= total_len) return ref_path.back();
        // 二分查找弧长所在段
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
            ref_path[lo].theta  // 朝向同 lo 点
        };
    };

    for (int i = 0; i < num_samples; i++) {
        double s = (total_len * i) / (num_samples - 1);
        Pose pt = interpolate(s);

        // Step 2: 计算切线方向 (用前后点差分)
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

        // 左法向 (+90°)
        Vec2d n_left  = { -ty,  tx };
        Vec2d n_right = {  ty, -tx };

        // Step 3: 沿 ±法向射线求交点距离
        Vec2d orig = { pt.x, pt.y };
        double d_left  = rayPolygonDist(orig, n_left,  outer);
        double d_right = rayPolygonDist(orig, n_right, outer);

        // 孔洞: 如果射线穿过孔洞, 取更近的交点
        for (auto& hole : holes) {
            double hl = rayPolygonDist(orig, n_left,  hole);
            double hr = rayPolygonDist(orig, n_right, hole);
            if (hl < d_left)  d_left  = hl;
            if (hr < d_right) d_right = hr;
        }

        // Step 4: 减去安全边距
        d_left  = std::max(0.0, d_left  - margin_);
        d_right = std::max(0.0, d_right - margin_);

        // Step 5: 存储截面
        CorridorSection sec;
        sec.center = orig;
        sec.left   = { orig.x + n_left.x  * d_left,
                        orig.y + n_left.y  * d_left };
        sec.right  = { orig.x + n_right.x * d_right,
                        orig.y + n_right.y * d_right };
        result.push_back(sec);
    }

    return result;
}
