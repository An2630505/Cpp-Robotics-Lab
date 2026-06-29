#include "physics_world.h"

#include <algorithm>
#include <stdexcept>

#include "collision.h"

namespace engine {
namespace physics {

// ============================================================================
// 实体管理
// ============================================================================

int64_t PhysicsWorld::add_entity(EntityState state,
                                  std::shared_ptr<MotionModel> model) {
    int64_t id = next_id_++;
    state.id = id;

    // 自动计算质量 = 面积 × 密度(1.0), 除非已显式设置
    if (state.mass <= 0.0 && !state.is_static) {
        state.mass = state.geometry.area();
    }

    entities_[id] = {std::move(state), std::move(model), ControlInput{}};
    return id;
}

void PhysicsWorld::remove_entity(int64_t id) {
    entities_.erase(id);
}

const EntityState* PhysicsWorld::get_entity_state(int64_t id) const {
    auto it = entities_.find(id);
    if (it != entities_.end()) {
        return &it->second.state;
    }
    return nullptr;
}

std::vector<int64_t> PhysicsWorld::get_all_entity_ids() const {
    std::vector<int64_t> ids;
    ids.reserve(entities_.size());
    for (const auto& [id, _] : entities_) {
        ids.push_back(id);
    }
    return ids;
}

std::vector<EntityState> PhysicsWorld::get_all_states() const {
    std::vector<EntityState> states;
    states.reserve(entities_.size());
    for (const auto& [_, entry] : entities_) {
        states.push_back(entry.state);
    }
    return states;
}

// ============================================================================
// 控制接口
// ============================================================================

void PhysicsWorld::apply_control(int64_t id, const ControlInput& cmd) {
    auto it = entities_.find(id);
    if (it != entities_.end()) {
        it->second.pending_cmd = cmd;
    }
}

// ============================================================================
// 步进
// ============================================================================

void PhysicsWorld::step(double dt) {
    last_collisions_.clear();

    // 1. 运动模型计算 + 积分
    integrate_entities(dt);

    // 2. 碰撞检测 + 响应
    detect_and_resolve_collisions();
}

void PhysicsWorld::integrate_entities(double dt) {
    for (auto& [id, entry] : entities_) {
        if (entry.state.is_static) continue;  // 静态实体不积分
        if (!entry.model) continue;           // 无模型则不动

        // 运动模型计算新速度
        Velocity new_vel = entry.model->step(
            entry.state.pose, entry.state.vel, entry.pending_cmd, dt);

        // 欧拉积分: 更新位姿
        entry.state.pose.x += new_vel.vx * dt;
        entry.state.pose.y += new_vel.vy * dt;
        entry.state.pose.theta += new_vel.omega * dt;

        // 更新速度
        entry.state.vel = new_vel;
    }
}

void PhysicsWorld::detect_and_resolve_collisions() {
    // 收集所有实体 ID (需要随机访问)
    std::vector<int64_t> ids;
    ids.reserve(entities_.size());
    for (const auto& [id, _] : entities_) {
        ids.push_back(id);
    }

    int n = static_cast<int>(ids.size());
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            auto& entry_a = entities_[ids[i]];
            auto& entry_b = entities_[ids[j]];

            // 两个静态实体之间不检测
            if (entry_a.state.is_static && entry_b.state.is_static) continue;

            // SAT 碰撞检测
            auto result = sat_collision(
                entry_a.state.geometry, entry_a.state.pose,
                entry_b.state.geometry, entry_b.state.pose);

            if (result.collides) {
                // 记录事件
                last_collisions_.push_back(
                    {ids[i], ids[j], result});

                // 弹性碰撞响应 (修改速度和位置)
                resolve_elastic_collision(entry_a.state, entry_b.state, result);
            }
        }
    }
}

}  // namespace physics
}  // namespace engine
