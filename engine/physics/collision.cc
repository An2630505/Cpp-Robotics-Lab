#include "collision.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace engine {
namespace physics {

// ============================================================================
// 辅助: 将局部坐标多边形变换到世界坐标系
// ============================================================================

std::vector<Vec2d> transform_polygon(const Polygon& poly, const Pose& pose) {
    std::vector<Vec2d> world_verts;
    world_verts.reserve(poly.vertices.size());
    for (const auto& v : poly.vertices) {
        world_verts.push_back(pose.transform(v));
    }
    return world_verts;
}

// ============================================================================
// 辅助: 计算多边形在给定轴上的投影区间 [min, max]
// ============================================================================

static void project_polygon(const std::vector<Vec2d>& verts,
                            const Vec2d& axis,
                            double& out_min, double& out_max) {
    out_min = std::numeric_limits<double>::max();
    out_max = std::numeric_limits<double>::lowest();
    for (const auto& v : verts) {
        double proj = v.dot(axis);
        if (proj < out_min) out_min = proj;
        if (proj > out_max) out_max = proj;
    }
}

// ============================================================================
// 辅助: 获取凸多边形的边法向列表 (单位向量, 朝外)
// ============================================================================

static std::vector<Vec2d> get_edge_normals(const std::vector<Vec2d>& verts) {
    std::vector<Vec2d> normals;
    int n = static_cast<int>(verts.size());
    normals.reserve(n);
    for (int i = 0; i < n; ++i) {
        const auto& p0 = verts[i];
        const auto& p1 = verts[(i + 1) % n];
        Vec2d edge = p1 - p0;
        // 边法向 (左旋90°), 归一化
        Vec2d nrm = edge.perp().normalized();
        // 确保法向朝外: 对于凸多边形 (CCW), 边法向 = edge 左旋 即朝外
        normals.push_back(nrm);
    }
    return normals;
}

// ============================================================================
// 辅助: 找多边形上沿给定方向的最远点 (support point)
// ============================================================================

static Vec2d support_point(const std::vector<Vec2d>& verts, const Vec2d& dir) {
    double best_dot = -std::numeric_limits<double>::max();
    Vec2d best_vert;
    for (const auto& v : verts) {
        double d = v.dot(dir);
        if (d > best_dot) {
            best_dot = d;
            best_vert = v;
        }
    }
    return best_vert;
}

// ============================================================================
// SAT 碰撞检测
// ============================================================================

CollisionResult sat_collision(const Polygon& poly_a, const Pose& pose_a,
                               const Polygon& poly_b, const Pose& pose_b) {
    CollisionResult result;
    result.collides = false;

    auto verts_a = transform_polygon(poly_a, pose_a);
    auto verts_b = transform_polygon(poly_b, pose_b);

    double min_overlap = std::numeric_limits<double>::max();
    Vec2d min_axis;

    // 收集所有分离轴: 两个多边形的边法向
    auto normals_a = get_edge_normals(verts_a);
    auto normals_b = get_edge_normals(verts_b);

    // 在每条轴上检查投影重叠
    auto check_axis = [&](const Vec2d& axis) {
        double min_a, max_a, min_b, max_b;
        project_polygon(verts_a, axis, min_a, max_a);
        project_polygon(verts_b, axis, min_b, max_b);

        // 检查分离
        if (max_a < min_b || max_b < min_a) {
            result.collides = false;
            return false;  // 找到分离轴, 无碰撞
        }

        // 计算重叠量
        double overlap = std::min(max_a - min_b, max_b - min_a);
        if (overlap < min_overlap) {
            min_overlap = overlap;
            min_axis = axis;
        }
        return true;  // 继续检查
    };

    for (const auto& n : normals_a) {
        if (!check_axis(n)) return result;
    }
    for (const auto& n : normals_b) {
        if (!check_axis(n)) return result;
    }

    // 所有轴都重叠 → 碰撞
    result.collides = true;
    result.penetration = min_overlap;

    // 确定法向方向: 从 A 指向 B
    // 用两多边形质心连线的投影方向来确定
    Vec2d center_a{0.0, 0.0}, center_b{0.0, 0.0};
    for (auto& v : verts_a) { center_a.x += v.x; center_a.y += v.y; }
    for (auto& v : verts_b) { center_b.x += v.x; center_b.y += v.y; }
    double inv_n_a = 1.0 / static_cast<double>(verts_a.size());
    double inv_n_b = 1.0 / static_cast<double>(verts_b.size());
    center_a.x *= inv_n_a; center_a.y *= inv_n_a;
    center_b.x *= inv_n_b; center_b.y *= inv_n_b;

    Vec2d ab = center_b - center_a;  // 从 A 到 B
    if (ab.dot(min_axis) >= 0.0) {
        result.normal = min_axis;
    } else {
        result.normal = min_axis * -1.0;
    }

    // 接触点: 取两多边形沿法向的支撑点中点 (近似)
    Vec2d support_a = support_point(verts_a, result.normal);
    Vec2d support_b = support_point(verts_b, result.normal * -1.0);
    result.contact_point = (support_a + support_b) * 0.5;

    return result;
}

// ============================================================================
// 弹性碰撞响应
// ============================================================================

void resolve_elastic_collision(EntityState& state_a,
                                EntityState& state_b,
                                const CollisionResult& result) {
    if (!result.collides) return;

    Vec2d n = result.normal;  // 从 A 指向 B
    double n_len = n.length();
    if (n_len < 1e-12) return;
    n = n / n_len;  // 确保单位向量

    // ---- 速度响应 (弹性碰撞, 动量守恒) ----
    // 碰撞点处两实体的速度 (简化为质心速度, 无角速度耦合)
    Vec2d v_a = {state_a.vel.vx, state_a.vel.vy};
    Vec2d v_b = {state_b.vel.vx, state_b.vel.vy};

    // 沿法向的相对速度
    double vrel_n = (v_a - v_b).dot(n);

    // 如果已经在分离 (vrel_n < 0: A 相对 B 沿法向远离), 不施加冲量
    if (vrel_n <= 0.0) {
        // 但仍需要位置修正
    } else {
        double m_a = state_a.is_static ? std::numeric_limits<double>::infinity() : state_a.mass;
        double m_b = state_b.is_static ? std::numeric_limits<double>::infinity() : state_b.mass;

        if (std::isinf(m_a) && std::isinf(m_b)) {
            // 两个都是静态, 不处理
            return;
        }

        double inv_m_a = state_a.is_static ? 0.0 : (1.0 / m_a);
        double inv_m_b = state_b.is_static ? 0.0 : (1.0 / m_b);
        double inv_total = inv_m_a + inv_m_b;

        // 完全弹性碰撞: 恢复系数 e = 1.0
        // 冲量大小: J = -(1+e) * vrel_n / (1/m_a + 1/m_b)
        double J = -(2.0 * vrel_n) / inv_total;

        // 施加冲量
        Vec2d impulse = n * J;
        if (!state_a.is_static) {
            state_a.vel.vx += impulse.x / m_a;
            state_a.vel.vy += impulse.y / m_a;
        }
        if (!state_b.is_static) {
            state_b.vel.vx -= impulse.x / m_b;
            state_b.vel.vy -= impulse.y / m_b;
        }
    }

    // ---- 位置修正: 分离穿透 ----
    double pen = result.penetration;
    if (pen > 0.0) {
        double inv_m_a = state_a.is_static ? 0.0 : (1.0 / state_a.mass);
        double inv_m_b = state_b.is_static ? 0.0 : (1.0 / state_b.mass);
        double inv_total = inv_m_a + inv_m_b;

        if (inv_total >= 1e-12) {
            double ratio_a = inv_m_a / inv_total;
            double ratio_b = inv_m_b / inv_total;

            Vec2d correction = n * pen;
            if (!state_a.is_static) {
                state_a.pose.x -= correction.x * ratio_a;
                state_a.pose.y -= correction.y * ratio_a;
            }
            if (!state_b.is_static) {
                state_b.pose.x += correction.x * ratio_b;
                state_b.pose.y += correction.y * ratio_b;
            }
        }
    }

}

}  // namespace physics
}  // namespace engine
