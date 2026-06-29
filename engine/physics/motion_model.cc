#include "motion_model.h"

#include <algorithm>
#include <cmath>

namespace engine {
namespace physics {

// ============================================================================
// BicycleModel
// ============================================================================

BicycleModel::BicycleModel(double wheelbase)
    : wheelbase_(wheelbase) {}

Velocity BicycleModel::step(const Pose& current_pose,
                             const Velocity& current_vel,
                             const ControlInput& cmd,
                             double dt) {
    double c = std::cos(current_pose.theta);
    double s = std::sin(current_pose.theta);

    // 将世界系速度分解为纵向 (沿车头) 和侧向 (垂直车头)
    double v_lon =  current_vel.vx * c + current_vel.vy * s;
    double v_lat = -current_vel.vx * s + current_vel.vy * c;

    // 控制指令只影响纵向: 沿车头方向施加加速度
    double v_lon_new = v_lon + cmd.ax * dt;

    // 自行车模型: omega = v_lon * tan(steer) / L
    double omega_new = v_lon * std::tan(cmd.steer) / wheelbase_;

    // 重组世界系速度: 纵向沿车头 + 侧向保留 (碰撞产生的侧滑不丢失)
    double vx_new = v_lon_new * c - v_lat * s;
    double vy_new = v_lon_new * s + v_lat * c;

    return {vx_new, vy_new, omega_new};
}

// ============================================================================
// SimpleModel
// ============================================================================

Velocity SimpleModel::step(const Pose& current_pose,
                            const Velocity& current_vel,
                            const ControlInput& cmd,
                            double dt) {
    double c = std::cos(current_pose.theta);
    double s = std::sin(current_pose.theta);

    // 沿车头方向施加纵向加速度; 侧向速度保持 (允许碰撞后侧滑)
    double vx_new = current_vel.vx + cmd.ax * c * dt;
    double vy_new = current_vel.vy + cmd.ax * s * dt;

    // steer 直接作为角速度
    return {vx_new, vy_new, cmd.steer};
}

}  // namespace physics
}  // namespace engine
