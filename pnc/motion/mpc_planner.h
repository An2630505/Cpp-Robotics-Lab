#ifndef PNC_MOTION_MPC_PLANNER_H_
#define PNC_MOTION_MPC_PLANNER_H_

#include <vector>
#include "../common/types.h"

const double MPC_CELL_SIZE    = 0.2;
const int    MPC_GRID_SIZE    = 256;
const double MPC_WHEELBASE    = 2.68;
const double MPC_CAR_WIDTH    = 1.8;
const double MPC_MAX_STEER    = 0.6;

class ContinuousMap {
public:
    ContinuousMap(const std::vector<std::vector<int>>& grid);
    int worldToRow(double y) const;
    int worldToCol(double x) const;
    double rowToWorldY(int row) const;
    double colToWorldX(int col) const;
    bool isOccupied(int row, int col) const;
    bool isCollision(const Pose& pose) const;
    const std::vector<std::vector<int>>& grid() const { return grid_; }
private:
    std::vector<std::vector<int>> grid_;
};

class MPCTrajectoryPlanner {
public:
    MPCTrajectoryPlanner(const std::vector<std::vector<int>>& grid);
    std::vector<Pose> plan(const std::vector<Pose>& ref_path,
                           std::vector<double>& out_velocities,
                           std::vector<double>& out_steers);
    void setHorizon(int N)          { N_ = N; }
    void setDt(double dt)           { dt_ = dt; }
    void setDesiredSpeed(double v)  { v_des_ = v; }
private:
    int N_, max_iter_;
    double dt_, L_, v_des_, step_size_;
    double w_pos_, w_theta_, w_steer_, w_dsteer_, w_collision_, w_vel_;
    ContinuousMap cmap_;
    std::vector<Pose> rollout(const Pose& start,
                              const std::vector<double>& vels,
                              const std::vector<double>& steers) const;
    double computeCost(const std::vector<Pose>& traj,
                       const std::vector<double>& vels,
                       const std::vector<double>& steers,
                       const std::vector<Pose>& ref) const;
    double collisionCost(const Pose& pose) const;
    Pose getLookahead(const std::vector<Pose>& path,
                      const Pose& current, double L_ahead) const;
    void trackWithPurePursuit(const Pose& start,
                              const std::vector<Pose>& ref,
                              std::vector<double>& out_vels,
                              std::vector<double>& out_steers,
                              std::vector<Pose>& out_traj);
};

#endif  // PNC_MOTION_MPC_PLANNER_H_
