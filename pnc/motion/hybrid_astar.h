#ifndef PNC_MOTION_HYBRID_ASTAR_H_
#define PNC_MOTION_HYBRID_ASTAR_H_

#include <vector>
#include "../common/types.h"

// ========================== 内部搜索节点 ==========================
struct HNode {
    double x, y, theta, g, h, f;
    int parent, xy_bins, theta_bins;
    double xy_bin;

    int key() const;
    bool operator>(const HNode& o) const { return f > o.f; }
};

inline int HNode::key() const {
    int ix = std::max(0, std::min(xy_bins - 1,
        static_cast<int>(x / xy_bin)));
    int iy = std::max(0, std::min(xy_bins - 1,
        static_cast<int>(y / xy_bin)));
    double th = theta;
    while (th < 0) th += 2 * M_PI;
    while (th >= 2 * M_PI) th -= 2 * M_PI;
    int ith = static_cast<int>(th / (2.0 * M_PI / theta_bins)) % theta_bins;
    return (iy * xy_bins + ix) * theta_bins + ith;
}

/// Hybrid A* — 运动学约束的连续路径规划
class HybridAStar {
public:
    HybridAStar(const std::vector<std::vector<int>>& grid);

    void setWheelbase(double w)  { wheelbase_ = w; }
    void setMaxSteer(double s)   { max_steer_ = s; }
    void setNumSteer(int n)      { num_steer_ = n; }
    void setArcLength(double a)  { arc_length_ = a; }
    void setCellSize(double c)   { cell_size_ = c; }
    void setGoalXYTol(double t)  { goal_xy_tol_ = t; }
    void setGoalThTol(double t)  { goal_th_tol_ = t; }
    void setThetaBins(int b)     { theta_bins_ = b; }
    void setXYBin(double b)      { xy_bin_ = b; }

    std::vector<Pose> plan(const Pose& start, const Pose& goal);

private:
    std::vector<std::vector<int>> grid_;
    int grid_size_;
    double cell_size_, wheelbase_, max_steer_, arc_length_;
    int num_steer_, theta_bins_;
    double xy_bin_, goal_xy_tol_, goal_th_tol_;

    Pose step(const Pose& from, double steer, double arc) const;
    bool collides(const Pose& p) const;
    bool arcCollides(const Pose& from, double steer, double arc) const;
    HNode makeNode(double x, double y, double theta) const {
        return {x, y, theta, 0, 0, 0, -1,
                static_cast<int>(grid_size_ * cell_size_ / xy_bin_) + 1,
                theta_bins_, xy_bin_};
    }
};

#endif  // PNC_MOTION_HYBRID_ASTAR_H_
