#pragma once

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "types.h"
#include "motion_model.h"

namespace engine {
namespace physics {

// ============================================================================
// 物理世界
//
// 职责:
//   - 管理实体 (增删查)
//   - 接收控制指令
//   - 按固定步长推进仿真 (积分 + 碰撞检测/响应)
//   - 提供碰撞事件查询
// ============================================================================
class PhysicsWorld {
public:
    PhysicsWorld() = default;
    ~PhysicsWorld() = default;

    // ---- 实体管理 ----

    /// 添加实体, 返回实体 ID
    int64_t add_entity(EntityState state,
                       std::shared_ptr<MotionModel> model);

    /// 移除实体
    void remove_entity(int64_t id);

    /// 获取实体状态 (只读)
    const EntityState* get_entity_state(int64_t id) const;

    /// 获取所有实体 ID
    std::vector<int64_t> get_all_entity_ids() const;

    /// 获取所有实体状态
    std::vector<EntityState> get_all_states() const;

    /// 获取实体数量
    size_t entity_count() const { return entities_.size(); }

    // ---- 控制接口 ----

    /// 下发控制指令 (下一帧 step 时生效)
    void apply_control(int64_t id, const ControlInput& cmd);

    // ---- 步进 ----

    /// 推进仿真一步
    /// 1. 对所有非静态实体执行运动模型计算
    /// 2. 对所有非静态实体执行运动积分 (pose += v * dt)
    /// 3. 检测所有实体对之间的碰撞
    /// 4. 施加弹性碰撞响应 (速度 + 位置修正)
    /// @param dt 时间步长 (s)
    void step(double dt);

    // ---- 查询 ----

    /// 获取最近一次 step 产生的碰撞事件
    const std::vector<CollisionEvent>& get_collisions() const {
        return last_collisions_;
    }

private:
    struct EntityEntry {
        EntityState state;
        std::shared_ptr<MotionModel> model;
        ControlInput pending_cmd;  // 本帧待执行的控制
    };

    std::unordered_map<int64_t, EntityEntry> entities_;
    int64_t next_id_ = 1;

    std::vector<CollisionEvent> last_collisions_;

    // 内部方法
    void integrate_entities(double dt);
    void detect_and_resolve_collisions();
};

}  // namespace physics
}  // namespace engine
