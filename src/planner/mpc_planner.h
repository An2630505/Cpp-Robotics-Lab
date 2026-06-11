#pragma once

#include <vector>
#include <cmath>

// ========================== 地图 & 车辆参数 ==========================
const double CELL_SIZE        = 0.2;
const int    GRID_SIZE        = 256;
const double WHEELBASE        = 2.68;
const double CAR_WIDTH        = 1.8;
const double REAR_OVERHANG    = 0.5;
const double FRONT_OVERHANG   = 0.5;
const double CAR_LENGTH       = REAR_OVERHANG + WHEELBASE + FRONT_OVERHANG;
const double COLLISION_MARGIN = CELL_SIZE;
const double MAX_STEER        = 0.6;

// ========================== 位姿 ==========================
struct Pose {
    double x, y, theta;
};

// ========================== 连续地图 (碰撞检测) ==========================
class ContinuousMap {
public:
    ContinuousMap(const std::vector<std::vector<int>>& grid);
    int  worldToRow(double y) const;
    int  worldToCol(double x) const;
    double rowToWorldY(int row) const;
    double colToWorldX(int col) const;
    bool isOccupied(int row, int col) const;
    bool isCollision(const Pose& pose) const;
    const std::vector<std::vector<int>>& grid() const { return grid_; }
    double mapWidth()  const { return GRID_SIZE * CELL_SIZE; }
    double mapHeight() const { return GRID_SIZE * CELL_SIZE; }
private:
    const std::vector<std::vector<int>>& grid_;
};

// ========================== MPC 轨迹规划器 ==========================
class MPCTrajectoryPlanner {
public:
    MPCTrajectoryPlanner(const std::vector<std::vector<int>>& grid);

    std::vector<Pose> plan(const std::vector<Pose>& ref_path,
                           std::vector<double>& out_velocities,
                           std::vector<double>& out_steers);

    void setHorizon(int N)          { N_ = N; }
    void setDt(double dt)           { dt_ = dt; }
    void setDesiredSpeed(double v)  { v_des_ = v; }
    void setMaxIter(int it)         { max_iter_ = it; }
    void setWeightPos(double w)     { w_pos_ = w; }
    void setWeightSteer(double w)   { w_steer_ = w; }
    void setWeightDSteer(double w)  { w_dsteer_ = w; }
    void setWeightCollision(double w) { w_collision_ = w; }
    void setWeightVel(double w)     { w_vel_ = w; }

private:
    int    N_, max_iter_;
    double dt_, L_, v_des_, step_size_;
    double w_pos_, w_theta_, w_steer_, w_dsteer_, w_collision_, w_vel_;

    const ContinuousMap cmap_;

    std::vector<Pose> rollout(const Pose& start,
                              const std::vector<double>& vels,
                              const std::vector<double>& steers) const;

    double computeCost(const std::vector<Pose>& traj,
                       const std::vector<double>& vels,
                       const std::vector<double>& steers,
                       const std::vector<Pose>& ref) const;

    double collisionCost(const Pose& pose) const;

    // 纯追踪控制器: 模拟车辆跟踪参考路径
    void trackWithPurePursuit(const Pose& start,
                              const std::vector<Pose>& ref,
                              std::vector<double>& out_vels,
                              std::vector<double>& out_steers,
                              std::vector<Pose>& out_traj);

    // 在参考路径上找前视点
    Pose getLookahead(const std::vector<Pose>& path,
                      const Pose& current, double L_ahead) const;
};
