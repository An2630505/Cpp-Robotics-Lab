#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace engine {
namespace physics {

// ============================================================================
// 2D 向量
// ============================================================================
struct Vec2d {
    double x = 0.0;
    double y = 0.0;

    Vec2d() = default;
    Vec2d(double x_, double y_) : x(x_), y(y_) {}

    Vec2d operator+(const Vec2d& o) const { return {x + o.x, y + o.y}; }
    Vec2d operator-(const Vec2d& o) const { return {x - o.x, y - o.y}; }
    Vec2d operator*(double s) const { return {x * s, y * s}; }
    Vec2d operator/(double s) const { return {x / s, y / s}; }
    Vec2d& operator+=(const Vec2d& o) { x += o.x; y += o.y; return *this; }
    Vec2d& operator-=(const Vec2d& o) { x -= o.x; y -= o.y; return *this; }

    double dot(const Vec2d& o) const { return x * o.x + y * o.y; }
    double cross(const Vec2d& o) const { return x * o.y - y * o.x; }
    double length() const { return std::sqrt(x * x + y * y); }
    double length_sq() const { return x * x + y * y; }
    Vec2d normalized() const {
        double l = length();
        return l > 1e-12 ? Vec2d{x / l, y / l} : Vec2d{0.0, 0.0};
    }
    Vec2d perp() const { return {-y, x}; }  // 左旋90°
};

// ============================================================================
// 2D 位姿
// ============================================================================
struct Pose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;  // 朝向角 (弧度)

    Pose() = default;
    Pose(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}

    /// 将局部坐标点变换到世界坐标系
    Vec2d transform(const Vec2d& local) const {
        double c = std::cos(theta);
        double s = std::sin(theta);
        return {x + local.x * c - local.y * s,
                y + local.x * s + local.y * c};
    }

    /// 将世界坐标系向量旋转到局部坐标系
    Vec2d inverse_rotate(const Vec2d& world_vec) const {
        double c = std::cos(theta);
        double s = std::sin(theta);
        return { world_vec.x * c + world_vec.y * s,
                -world_vec.x * s + world_vec.y * c};
    }
};

// ============================================================================
// 速度状态 (世界坐标系)
// ============================================================================
struct Velocity {
    double vx = 0.0;    // 世界系 x 方向线速度
    double vy = 0.0;    // 世界系 y 方向线速度
    double omega = 0.0; // 角速度 (rad/s)

    Velocity() = default;
    Velocity(double vx_, double vy_, double omega_) : vx(vx_), vy(vy_), omega(omega_) {}
};

// ============================================================================
// 凸多边形 (顶点逆时针排列, 实体局部坐标系)
// ============================================================================
struct Polygon {
    std::vector<Vec2d> vertices;

    Polygon() = default;
    explicit Polygon(std::vector<Vec2d> verts) : vertices(std::move(verts)) {}

    /// 计算多边形面积 (Shoelace 公式)
    double area() const {
        double a = 0.0;
        int n = static_cast<int>(vertices.size());
        if (n < 3) return 0.0;
        for (int i = 0; i < n; ++i) {
            const auto& p0 = vertices[i];
            const auto& p1 = vertices[(i + 1) % n];
            a += p0.x * p1.y - p1.x * p0.y;
        }
        return std::abs(a) * 0.5;
    }

    /// 生成轴对齐矩形 (AABB)
    static Polygon aabb(double half_w, double half_h) {
        return Polygon({
            {-half_w, -half_h},
            { half_w, -half_h},
            { half_w,  half_h},
            {-half_w,  half_h}});
    }

    /// 生成有向矩形 (车辆形状: forward/backward/half_width)
    static Polygon vehicle(double half_width, double forward, double backward) {
        return Polygon({
            { forward,  half_width},
            { forward, -half_width},
            {-backward, -half_width},
            {-backward,  half_width}});
    }
};

// ============================================================================
// 控制指令
// ============================================================================
struct ControlInput {
    double steer = 0.0;  // 方向盘转角 (rad), 或直接角速度 (SimpleModel)
    double ax = 0.0;     // 纵向加速度 (m/s²)

    ControlInput() = default;
    ControlInput(double s, double a) : steer(s), ax(a) {}
};

// ============================================================================
// 实体状态
// ============================================================================
struct EntityState {
    int64_t id = 0;
    Pose pose;
    Velocity vel;
    Polygon geometry;   // 局部坐标系下的凸多边形
    double mass = 0.0;  // 0 = 自动从 geometry.area() 计算; 显式设置则使用该值
    bool is_static = false;

    EntityState() = default;
};

// ============================================================================
// 碰撞检测结果
// ============================================================================
struct CollisionResult {
    bool collides = false;
    Vec2d normal;          // 碰撞法向 (从实体A指向实体B), 世界坐标系
    double penetration = 0.0;  // 穿透深度 (m)
    Vec2d contact_point;       // 接触点 (世界坐标系, 近似)

    CollisionResult() = default;
};

// ============================================================================
// 碰撞事件
// ============================================================================
struct CollisionEvent {
    int64_t entity_a;
    int64_t entity_b;
    CollisionResult result;
};

}  // namespace physics
}  // namespace engine
