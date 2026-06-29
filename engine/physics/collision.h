#pragma once

#include "types.h"

namespace engine {
namespace physics {

// ============================================================================
// SAT (Separating Axis Theorem) 凸多边形碰撞检测
// ============================================================================

/// 将局部坐标多边形变换到世界坐标系
std::vector<Vec2d> transform_polygon(const Polygon& poly, const Pose& pose);

/// SAT 碰撞检测 (两凸多边形)
/// @return CollisionResult; 若 collides==false, normal/penetration 无意义
/// @note 法向从实体A指向实体B
CollisionResult sat_collision(const Polygon& poly_a, const Pose& pose_a,
                               const Polygon& poly_b, const Pose& pose_b);

// ============================================================================
// 弹性碰撞响应
// ============================================================================

/// 对一对实体施加完全弹性碰撞响应 (动量守恒)
/// @param state_a  实体A状态 (含质量、速度) — 会被修改
/// @param state_b  实体B状态 (含质量、速度) — 会被修改
/// @param result   SAT 碰撞检测结果 (法向从A指向B)
/// @note  静态实体质量视为∞, 只反弹对方
/// @note  位置修正: 将两实体沿法向分离, 消除穿透
void resolve_elastic_collision(EntityState& state_a,
                                EntityState& state_b,
                                const CollisionResult& result);

}  // namespace physics
}  // namespace engine
