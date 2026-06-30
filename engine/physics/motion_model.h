#pragma once

#include "types.h"

namespace engine {
namespace physics {

// ============================================================================
// 运动模型抽象基类
// ============================================================================
class MotionModel {
public:
    virtual ~MotionModel() = default;

    /// 根据当前状态和控制指令, 计算下一时刻的速度
    /// @param current_pose  当前位姿 (用于方向分解)
    /// @param current_vel   当前速度 (世界坐标系)
    /// @param cmd           控制指令
    /// @param dt            时间步长
    /// @return 新的速度 (世界坐标系)
    virtual Velocity step(const Pose& current_pose,
                          const Velocity& current_vel,
                          const ControlInput& cmd,
                          double dt) = 0;
};

// ============================================================================
// 运动学自行车模型
//
// 控制: steer (前轮转角) + ax (纵向加速度)
// 动力学: omega = v_lon * tan(steer) / L
// 速度方向始终与车头朝向一致 (无侧滑假设)
// ============================================================================
class BicycleModel : public MotionModel {
public:
    explicit BicycleModel(double wheelbase);

    Velocity step(const Pose& current_pose,
                  const Velocity& current_vel,
                  const ControlInput& cmd,
                  double dt) override;

    double wheelbase() const { return wheelbase_; }

    /// 侧向阻尼系数 (模拟轮胎侧偏刚度, 默认 5.0, 0=无阻尼/冰面)
    void set_lat_damping(double d) { lat_damping_ = d; }
    double lat_damping() const { return lat_damping_; }

private:
    double wheelbase_;      // 轴距 L (m)
    double lat_damping_ = 5.0;  // 侧向速度衰减率 (1/s)
};

// ============================================================================
// 简单运动模型
//
// 控制: steer → 直接作为角速度 omega
//       ax    → 沿车头朝向的纵向加速度
// 保持世界坐标系下的侧向速度分量 (碰撞后可侧滑)
// ============================================================================
class SimpleModel : public MotionModel {
public:
    SimpleModel() = default;

    Velocity step(const Pose& current_pose,
                  const Velocity& current_vel,
                  const ControlInput& cmd,
                  double dt) override;
};

}  // namespace physics
}  // namespace engine
