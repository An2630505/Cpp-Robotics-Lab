/**
 * MPC 轨迹规划器 — 完整的连续轨迹生成管线
 *
 * 管线流程:
 *   1. 加载 Occupancy Grid 地图
 *   2. 膨胀障碍物 (为车辆留出安全距离)
 *   3. 在膨胀地图上运行离散 A* → 安全参考路径
 *   4. 纯追踪 (Pure Pursuit) 控制器模拟车辆跟踪参考路径
 *   5. 输出: (x, y, θ, v, δ) 序列 + PPM 对比图
 *
 * 输出格式: output/mpc_trajectory.txt
 *   每行: x(m) y(m) θ(rad) v(m/s) δ(rad)
 */

#include "mpc_planner.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <cfloat>

// ===================================================================
//  ContinuousMap — 连续空间碰撞检测
// ===================================================================
//  将离散 Occupancy Grid 封装为连续坐标系的地图。
//  世界坐标系: x 向右, y 向下, 原点在 (0,0) = grid[0][0]
//  每格 CELL_SIZE = 0.2m
// ===================================================================

ContinuousMap::ContinuousMap(const std::vector<std::vector<int>>& grid)
    : grid_(grid) {}

// 世界坐标 → 网格行号 (y 向下增大, row 也向下增大)
int ContinuousMap::worldToRow(double y) const {
    int row = static_cast<int>(y / CELL_SIZE);
    return std::max(0, std::min(GRID_SIZE - 1, row));
}

// 世界坐标 → 网格列号 (x 向右增大, col 也向右增大)
int ContinuousMap::worldToCol(double x) const {
    int col = static_cast<int>(x / CELL_SIZE);
    return std::max(0, std::min(GRID_SIZE - 1, col));
}

// 网格 → 世界 (返回格子中心坐标)
double ContinuousMap::rowToWorldY(int row) const { return row * CELL_SIZE + CELL_SIZE / 2.0; }
double ContinuousMap::colToWorldX(int col) const { return col * CELL_SIZE + CELL_SIZE / 2.0; }

// 单格查询: 越界或 obstacle → true
bool ContinuousMap::isOccupied(int row, int col) const {
    if (row < 0 || row >= GRID_SIZE || col < 0 || col >= GRID_SIZE) return true;
    return grid_[row][col] == 1;
}

/**
 * 车辆矩形碰撞检测
 *
 * 原理:
 *   1. 计算车辆四角在车体坐标系下的坐标
 *      车体坐标系: x 向前(车头), y 向左
 *      参考点为后轴中心
 *   2. 用旋转矩阵 R(θ) 变换到世界坐标系
 *   3. 取轴对齐包围盒 (AABB), 映射到网格索引范围
 *   4. 遍历 AABB 内每个障碍物格子:
 *      将格子中心变换回车体坐标系 (R(-θ)),
 *      判断是否在车辆矩形内
 *
 * @return true=碰撞, false=安全
 */
bool ContinuousMap::isCollision(const Pose& pose) const {
    double c = std::cos(pose.theta), s = std::sin(pose.theta);

    // 车辆半宽 + 碰撞裕度
    double hw = CAR_WIDTH / 2.0 + COLLISION_MARGIN;
    // 前向延伸 (后轴 → 车头 + margin)
    double fwd = WHEELBASE + FRONT_OVERHANG + COLLISION_MARGIN;
    // 后向延伸 (后轴 → 车尾 + margin)
    double rev = REAR_OVERHANG + COLLISION_MARGIN;

    // 车体坐标系下的四角: 前左, 前右, 后左, 后右
    double crn[4][2] = {{fwd, hw}, {fwd, -hw}, {-rev, hw}, {-rev, -hw}};

    // 变换到世界坐标系, 同时收集 AABB
    double mnx = 1e9, mxx = -1e9, mny = 1e9, mxy = -1e9;
    for (int i = 0; i < 4; i++) {
        double wx = c * crn[i][0] - s * crn[i][1] + pose.x;
        double wy = s * crn[i][0] + c * crn[i][1] + pose.y;
        mnx = std::min(mnx, wx); mxx = std::max(mxx, wx);
        mny = std::min(mny, wy); mxy = std::max(mxy, wy);
    }

    // AABB → 网格索引范围
    int cmin = std::max(0, (int)std::floor(mnx / CELL_SIZE));
    int cmax = std::min(GRID_SIZE - 1, (int)std::ceil(mxx / CELL_SIZE));
    int rmin = std::max(0, (int)std::floor(mny / CELL_SIZE));
    int rmax = std::min(GRID_SIZE - 1, (int)std::ceil(mxy / CELL_SIZE));

    // 遍历 AABB 内的障碍物格子, 判断是否在车身矩形内
    for (int r = rmin; r <= rmax; r++) {
        for (int ci = cmin; ci <= cmax; ci++) {
            if (grid_[r][ci] == 0) continue;  // 空闲, 跳过

            // 格子中心世界坐标
            double cx = ci * CELL_SIZE + CELL_SIZE / 2.0;
            double cy = r  * CELL_SIZE + CELL_SIZE / 2.0;
            // 变换到车体坐标系: 先平移, 再乘 R(-θ)
            double dx = cx - pose.x, dy = cy - pose.y;
            double bx =  c * dx + s * dy;   // cos(-θ)=cos(θ), sin(-θ)=-sin(θ)
            double by = -s * dx + c * dy;

            // 检查是否在车辆矩形内部
            if (bx >= -rev && bx <= fwd && by >= -hw && by <= hw)
                return true;  // 碰撞!
        }
    }
    return false;
}

// ===================================================================
//  MPCTrajectoryPlanner — 构造 & 默认参数
// ===================================================================
//  默认值:
//    N_ = 30 步, dt_ = 0.1s → 预测 3s
//    v_des_ = 3.0 m/s
//    权重: 位置跟踪 10, 朝向 5, 转向平滑 2, 转向变化 15, 碰撞 500
// ===================================================================

MPCTrajectoryPlanner::MPCTrajectoryPlanner(
    const std::vector<std::vector<int>>& grid)
    : cmap_(grid),
      N_(30), dt_(0.1), L_(WHEELBASE), v_des_(3.0),
      max_iter_(80), step_size_(0.02),
      w_pos_(10.0), w_theta_(5.0), w_steer_(2.0),
      w_dsteer_(15.0), w_collision_(500.0), w_vel_(1.0)
{}

// ===================================================================
//  运动学前向仿真 (rollout)
// ===================================================================
//  给定起始位姿 + 速度/转向序列, 用自行车模型前向积分 N 步。
//
//  自行车运动学 (参考点 = 后轴中心):
//    直线 (δ≈0):
//      x_{k+1} = x_k + v·cos(θ)·dt
//      y_{k+1} = y_k + v·sin(θ)·dt
//      θ_{k+1} = θ_k
//    圆弧 (δ≠0):
//      R = L / tan(δ)                    — 转弯半径
//      dθ = v·dt / R                     — 角度增量
//      x_{k+1} = x_k + R·[sin(θ+dθ) - sin(θ)]
//      y_{k+1} = y_k + R·[cos(θ) - cos(θ+dθ)]
//      θ_{k+1} = θ_k + dθ
// ===================================================================

std::vector<Pose> MPCTrajectoryPlanner::rollout(
    const Pose& start,
    const std::vector<double>& vels,
    const std::vector<double>& steers) const
{
    std::vector<Pose> traj(N_ + 1);  // N 步仿真 → N+1 个状态
    traj[0] = start;

    for (int k = 0; k < N_; k++) {
        double x = traj[k].x, y = traj[k].y, theta = traj[k].theta;
        double v = vels[k], delta = steers[k];

        // 转向角物理限幅
        delta = std::max(-MAX_STEER, std::min(MAX_STEER, delta));

        if (std::abs(delta) < 1e-6) {
            // --- 直线运动 ---
            traj[k+1].x = x + v * std::cos(theta) * dt_;
            traj[k+1].y = y + v * std::sin(theta) * dt_;
            traj[k+1].theta = theta;
        } else {
            // --- 圆弧运动 ---
            double R = L_ / std::tan(delta);          // 转弯半径
            double dtheta = v * dt_ / R;               // 本步转角
            traj[k+1].x = x + R * (std::sin(theta + dtheta) - std::sin(theta));
            traj[k+1].y = y + R * (std::cos(theta) - std::cos(theta + dtheta));
            traj[k+1].theta = theta + dtheta;
        }
    }
    return traj;
}

// ===================================================================
//  碰撞代价 — 人工势场法
// ===================================================================
//  在车辆周围搜索最近障碍物, 距离 < safe_dist 时返回 1/d² 代价。
//  用于梯度优化时"推开"轨迹远离障碍物。
//  (当前纯追踪方案不依赖此函数, 保留供扩展)
// ===================================================================

double MPCTrajectoryPlanner::collisionCost(const Pose& pose) const {
    double min_dist = 5.0;  // 搜索半径 5m
    int search_radius = static_cast<int>(min_dist / CELL_SIZE) + 1;

    int cr = cmap_.worldToRow(pose.y);
    int cc = cmap_.worldToCol(pose.x);

    // 搜索周围格子, 找最近障碍物
    for (int dr = -search_radius; dr <= search_radius; dr++) {
        for (int dc = -search_radius; dc <= search_radius; dc++) {
            int r = cr + dr, c = cc + dc;
            if (r < 0 || r >= GRID_SIZE || c < 0 || c >= GRID_SIZE) continue;
            if (cmap_.grid()[r][c] == 0) continue;

            double cx = cmap_.colToWorldX(c);
            double cy = cmap_.rowToWorldY(r);
            double dist = std::hypot(cx - pose.x, cy - pose.y);
            if (dist < min_dist) min_dist = dist;
        }
    }

    // 平方反比势场: cost = 1/(d+ε)²
    double safe_dist = 1.5;
    if (min_dist < safe_dist) {
        double d = std::max(0.05, min_dist);  // 避免除零
        return 1.0 / (d * d);
    }
    return 0.0;
}

// ===================================================================
//  优化总代价
// ===================================================================
//  J = Σ( w_pos·||p-p_ref||²                位置跟踪
//       + w_theta·(θ-θ_ref)²                朝向跟踪
//       + w_collision·collisionCost(p)       碰撞势场
//       + w_steer·δ_k²                       转向幅度惩罚
//       + w_dsteer·(δ_k-δ_{k-1})²           转向平滑惩罚
//       + w_vel·(v_k-v_des)²                速度跟踪惩罚 )
//  (保留供梯度优化扩展用)
// ===================================================================

double MPCTrajectoryPlanner::computeCost(
    const std::vector<Pose>& traj,
    const std::vector<double>& vels,
    const std::vector<double>& steers,
    const std::vector<Pose>& ref) const
{
    double cost = 0.0;

    for (int k = 0; k <= N_; k++) {
        // 位置误差 (欧几里得距离平方)
        double dx = traj[k].x - ref[k].x;
        double dy = traj[k].y - ref[k].y;
        cost += w_pos_ * (dx*dx + dy*dy);

        // 朝向误差 (归一化到 [-π, π])
        double dth = traj[k].theta - ref[k].theta;
        while (dth >  M_PI) dth -= 2 * M_PI;
        while (dth < -M_PI) dth += 2 * M_PI;
        cost += w_theta_ * dth * dth;

        // 碰撞势场
        cost += w_collision_ * collisionCost(traj[k]);

        if (k < N_) {
            // 转向幅度惩罚 (鼓励小转向)
            cost += w_steer_ * steers[k] * steers[k];

            // 转向变化率惩罚 (鼓励平滑转向)
            if (k > 0) {
                double dsteer = steers[k] - steers[k-1];
                cost += w_dsteer_ * dsteer * dsteer;
            }

            // 速度偏离惩罚
            double dv = vels[k] - v_des_;
            cost += w_vel_ * dv * dv;
        }
    }
    return cost;
}

// ===================================================================
//  纯追踪控制器 (Pure Pursuit) — 核心算法
// ===================================================================
//  纯追踪是最简单的模型预测控制: 用运动学模型预测,
//  用几何关系直接计算转向角, 不需要迭代优化。
//
//  前视点: 在参考路径上, 距离当前车辆 L_ahead 的点
//  转向律:  δ = atan( 2·L·sin(α) / Ld )
//     L  = 轴距, α = 车辆朝向与前视方向的夹角, Ld = 前视距离
//  曲率限速: v_max = sqrt(a_lat_max / |κ|)
//     κ = tan(δ) / L  (当前曲率)
// ===================================================================

/**
 * 在参考路径上找前视点
 * @param path     参考路径 (x,y,θ 序列)
 * @param current  车辆当前位置
 * @param L_ahead  前视距离 (m)
 * @return         前视点位姿
 */
Pose MPCTrajectoryPlanner::getLookahead(
    const std::vector<Pose>& path, const Pose& current, double L_ahead) const
{
    // 找到参考路径上离车辆最近的点
    double best_dist = 1e9;
    size_t best_i = 0;
    for (size_t i = 0; i < path.size(); i++) {
        double d = std::hypot(path[i].x - current.x, path[i].y - current.y);
        if (d < best_dist) { best_dist = d; best_i = i; }
    }

    // 从最近点沿路径向前, 累计弧长直到 ≥ L_ahead
    double cum = 0;
    for (size_t i = best_i; i + 1 < path.size(); i++) {
        double seg = std::hypot(path[i+1].x - path[i].x,
                                 path[i+1].y - path[i].y);
        if (cum + seg >= L_ahead) {
            // 前视点落在当前线段内部, 线性插值
            double t = (L_ahead - cum) / seg;
            Pose lp;
            lp.x = path[i].x + t * (path[i+1].x - path[i].x);
            lp.y = path[i].y + t * (path[i+1].y - path[i].y);
            lp.theta = path[i].theta;
            return lp;
        }
        cum += seg;
    }
    // 路径终点作为前视点
    return path.back();
}

/**
 * 纯追踪轨迹生成
 *
 * 对每个时间步 k = 0..N-1:
 *   1. 找前视点 (距当前位姿 L_ahead 沿参考路径)
 *   2. 计算 α = 前视方向 - 车头朝向
 *   3. 纯追踪转向: δ = atan(2·L·sin(α) / Ld)
 *   4. 曲率限速: v = min(v_des, sqrt(a_max / |κ|))
 *   5. 自行车模型前向积分一步
 */
void MPCTrajectoryPlanner::trackWithPurePursuit(
    const Pose& start,
    const std::vector<Pose>& ref,
    std::vector<double>& out_vels,
    std::vector<double>& out_steers,
    std::vector<Pose>& out_traj)
{
    out_traj.resize(N_ + 1);
    out_traj[0] = start;
    out_vels.assign(N_, v_des_);
    out_steers.assign(N_, 0.0);

    const double L_AHEAD     = 3.0;   // 前视距离 (m)
    const double MAX_LAT_ACC = 3.0;   // 最大侧向加速度 (m/s²)

    Pose cur = start;

    for (int k = 0; k < N_; k++) {
        // ---- 步骤 1: 纯追踪转向 ----
        Pose lp = getLookahead(ref, cur, L_AHEAD);
        double dx = lp.x - cur.x;
        double dy = lp.y - cur.y;

        // α = 前视方向角 - 当前朝向 (车辆需要转的角度)
        double alpha = std::atan2(dy, dx) - cur.theta;
        while (alpha >  M_PI) alpha -= 2 * M_PI;   // 归一化
        while (alpha < -M_PI) alpha += 2 * M_PI;

        double Ld = std::hypot(dx, dy);
        if (Ld < 0.01) Ld = L_AHEAD;  // 防止除零

        // 纯追踪核心公式
        double delta = std::atan2(2.0 * L_ * std::sin(alpha), Ld);
        delta = std::max(-MAX_STEER, std::min(MAX_STEER, delta));

        // ---- 步骤 2: 曲率限速 ----
        double curvature = std::abs(std::tan(delta) / L_);
        double v_max = curvature > 1e-6
            ? std::sqrt(MAX_LAT_ACC / curvature)   // 侧向加速度约束
            : v_des_;                                // 直线无限制
        double v = std::min(v_des_, v_max * 0.8);    // 80% 安全系数

        out_steers[k] = delta;
        out_vels[k]   = v;

        // ---- 步骤 3: 自行车模型前向积分一步 ----
        if (std::abs(delta) < 1e-6) {
            // 直线
            out_traj[k+1].x = cur.x + v * std::cos(cur.theta) * dt_;
            out_traj[k+1].y = cur.y + v * std::sin(cur.theta) * dt_;
            out_traj[k+1].theta = cur.theta;
        } else {
            // 圆弧
            double R = L_ / std::tan(delta);
            double dtheta = v * dt_ / R;
            out_traj[k+1].x = cur.x + R * (std::sin(cur.theta + dtheta) - std::sin(cur.theta));
            out_traj[k+1].y = cur.y + R * (std::cos(cur.theta) - std::cos(cur.theta + dtheta));
            out_traj[k+1].theta = cur.theta + dtheta;
        }
        cur = out_traj[k+1];
    }
    std::cout << "  纯追踪完成: " << N_ << " 步" << std::endl;
}

// ===================================================================
//  plan() — MPC 规划入口
// ===================================================================
//  调用纯追踪控制器生成轨迹, 输出统计信息。
//  @param ref_path        参考路径 (x,y,θ 序列, 起点到终点)
//  @param out_velocities  输出: 速度序列 v[0..N-1] (m/s)
//  @param out_steers      输出: 转向序列 δ[0..N-1] (rad)
//  @return                规划的轨迹 (N+1 个位姿)
// ===================================================================

std::vector<Pose> MPCTrajectoryPlanner::plan(
    const std::vector<Pose>& ref_path,
    std::vector<double>& out_velocities,
    std::vector<double>& out_steers)
{
    std::cout << "\n=== MPC 轨迹规划 (纯追踪) ===" << std::endl;
    std::cout << "时域: N=" << N_ << " dt=" << dt_
              << "s → 预测 " << N_ * dt_ << "s" << std::endl;
    std::cout << "前视距离: 3.0m | 期望速度: " << v_des_ << "m/s" << std::endl;

    std::vector<Pose> traj;
    trackWithPurePursuit(ref_path[0], ref_path, out_velocities, out_steers, traj);

    // ---- 统计信息 ----
    double total_len = 0, max_steer = 0;
    for (int k = 1; k <= N_; k++)
        total_len += std::hypot(traj[k].x - traj[k-1].x, traj[k].y - traj[k-1].y);
    for (int k = 0; k < N_; k++)
        max_steer = std::max(max_steer, std::abs(out_steers[k]));

    // 终点误差: 轨迹终点与参考路径终点的欧几里得距离
    double end_err = std::hypot(traj.back().x - ref_path.back().x,
                                 traj.back().y - ref_path.back().y);

    std::cout << "\n=== 规划结果 ===" << std::endl;
    std::cout << "轨迹长度: " << total_len << "m" << std::endl;
    std::cout << "终点误差: " << end_err << "m" << std::endl;
    std::cout << "最大转向: " << (max_steer * 180 / M_PI) << "°" << std::endl;
    std::cout << "速度范围: " << *std::min_element(out_velocities.begin(), out_velocities.end())
              << " ~ " << *std::max_element(out_velocities.begin(), out_velocities.end())
              << " m/s" << std::endl;

    return traj;
}

// ===================================================================
//  工具函数: 文件读写
// ===================================================================

// 读取 Hybrid A* 格式路径 (x y theta 每行)
std::vector<Pose> readHybridPath(const std::string& filepath) {
    std::vector<Pose> path;
    std::ifstream in(filepath);
    std::string line;
    while (std::getline(in, line)) {
        if (line[0] == '#') continue;  // 跳过注释
        double x, y, th;
        std::istringstream iss(line);
        if (iss >> x >> y >> th) path.push_back({x, y, th});
    }
    return path;
}

/**
 * 离散 A* 路径 → 连续位姿序列
 *
 * 转换规则:
 *   - 世界 x = col * CELL_SIZE + CELL_SIZE/2  (格子中心)
 *   - 世界 y = row * CELL_SIZE + CELL_SIZE/2
 *   - θ = 相邻两点连线方向 (用前后向量的平均平滑)
 */
std::vector<Pose> discreteToHybridPath(const std::string& filepath) {
    // 读取 (row, col) 序列
    std::vector<std::pair<int,int>> cells;
    std::ifstream in(filepath);
    std::string line;
    while (std::getline(in, line)) {
        if (line[0] == '#') continue;
        int r, c;
        std::istringstream iss(line);
        if (iss >> r >> c) cells.push_back({r, c});
    }
    if (cells.empty()) return {};

    // 转换为世界坐标
    std::vector<Pose> path;
    for (size_t i = 0; i < cells.size(); i++) {
        Pose p;
        p.x = cells[i].second * CELL_SIZE + CELL_SIZE / 2.0;  // col → x
        p.y = cells[i].first  * CELL_SIZE + CELL_SIZE / 2.0;  // row → y
        if (i + 1 < cells.size())
            // 朝向 = 从当前点指向下一个点的方向
            p.theta = std::atan2(cells[i+1].first - cells[i].first,
                                 cells[i+1].second - cells[i].second);
        else if (i > 0)
            p.theta = path.back().theta;  // 最后一点保持上一朝向
        else
            p.theta = 0;
        path.push_back(p);
    }

    // 朝向平滑: 每点取前后方向的角度平均 (消除离散路径的锯齿)
    for (size_t i = 1; i + 1 < path.size(); i++) {
        double th_prev = std::atan2(path[i].y - path[i-1].y,
                                     path[i].x - path[i-1].x);
        double th_next = std::atan2(path[i+1].y - path[i].y,
                                     path[i+1].x - path[i].x);
        // 角度平均: atan2(Σsin, Σcos) 正确处理了 ±π 环绕
        double th_avg = std::atan2(std::sin(th_prev) + std::sin(th_next),
                                    std::cos(th_prev) + std::cos(th_next));
        path[i].theta = th_avg;
    }
    return path;
}

// 网格文件数据结构
struct GridData2 {
    std::vector<std::vector<int>> grid;
    int start_row, start_col, goal_row, goal_col, size;
};

// 读取 grid.txt (与 generate_grid.cpp 输出格式兼容)
GridData2 readGrid2(const std::string& fp) {
    std::ifstream in(fp);
    GridData2 d; std::string line;
    // 跳过注释行 (以 # 开头)
    while (std::getline(in, line)) if (line[0] != '#') break;
    // 第 1 个非注释行: grid size
    std::istringstream iss(line); iss >> d.size;
    // 第 2-3 行: start (row col), goal (row col)
    in >> d.start_row >> d.start_col >> d.goal_row >> d.goal_col;
    // 剩余: 网格矩阵 (空格分隔的 0/1)
    d.grid.assign(d.size, std::vector<int>(d.size, 0));
    for (int r = 0; r < d.size; r++)
        for (int c = 0; c < d.size; c++)
            in >> d.grid[r][c];
    return d;
}

// ===================================================================
//  PPM 图片输出 — 参考路径 vs MPC 轨迹对比
// ===================================================================
//  颜色含义:
//    灰色  = 参考路径 (膨胀地图上的 A* 结果)
//    绿色  = MPC 纯追踪轨迹
//    蓝色  = 起点
//    红色  = 终点
//    黑色  = 障碍物 (原始地图)
//    白色  = 空闲区域
//  输出: 3x 放大的 PPM P3 格式图片
// ===================================================================

void saveMPCPPM(const std::vector<std::vector<int>>& grid,
                const Pose& start, const Pose& goal,
                const std::vector<Pose>& ref_path,
                const std::vector<Pose>& mpc_traj,
                const std::string& filepath) {
    std::ofstream out(filepath);
    const int S = 3;  // 放大倍率 (每格 → S×S 像素)
    int w = GRID_SIZE * S, h = GRID_SIZE * S;
    out << "P3\n" << w << " " << h << "\n255\n";

    // 栅格化路径: 将连续坐标映射到网格, 标记每个格子上的路径类型
    auto mark = [](const Pose& p, std::vector<std::vector<char>>& m, char c) {
        int r = std::max(0, std::min(GRID_SIZE - 1, (int)(p.y / CELL_SIZE)));
        int cl = std::max(0, std::min(GRID_SIZE - 1, (int)(p.x / CELL_SIZE)));
        m[r][cl] = c;
    };

    std::vector<std::vector<char>> overlay(GRID_SIZE,
        std::vector<char>(GRID_SIZE, 0));       // 0 = 无路径
    for (const auto& p : ref_path) mark(p, overlay, 'H');   // H = 参考路径 (Hybrid)
    for (const auto& p : mpc_traj) mark(p, overlay, 'M');   // M = MPC 轨迹

    // 起点终点 (映射到网格)
    int sr = std::max(0, std::min(GRID_SIZE - 1, (int)(start.y / CELL_SIZE)));
    int sc = std::max(0, std::min(GRID_SIZE - 1, (int)(start.x / CELL_SIZE)));
    int gr = std::max(0, std::min(GRID_SIZE - 1, (int)(goal.y  / CELL_SIZE)));
    int gc = std::max(0, std::min(GRID_SIZE - 1, (int)(goal.x  / CELL_SIZE)));

    // 逐像素输出
    for (int r = 0; r < GRID_SIZE; r++) {
        for (int sy = 0; sy < S; sy++) {           // 纵向放大
            for (int c = 0; c < GRID_SIZE; c++) {
                int R = 255, G = 255, B = 255;     // 默认白色 (空闲)
                if (r == sr && c == sc)         { R = 0;   G = 0;   B = 255; } // 蓝 = 起点
                else if (r == gr && c == gc)     { R = 255; G = 0;   B = 0;   } // 红 = 终点
                else if (overlay[r][c] == 'M')   { R = 0;   G = 255; B = 0;   } // 绿 = MPC 轨迹
                else if (overlay[r][c] == 'H')   { R = 180; G = 180; B = 180; } // 灰 = 参考路径
                else if (grid[r][c] == 1)        { R = 0;   G = 0;   B = 0;   } // 黑 = 障碍物
                for (int sx = 0; sx < S; sx++)     // 横向放大
                    out << R << " " << G << " " << B << " ";
            }
            out << "\n";
        }
    }
    out.close();
}

// ===================================================================
//  main — MPC 轨迹规划完整管线
// ===================================================================
//
//  管线步骤:
//   1. 读取原始 Occupancy Grid 地图
//   2. 膨胀障碍物 (DILATE_CELLS 格)  → 车辆安全走廊
//   3. 在膨胀地图上运行离散 A*    → 安全参考路径 (grid 坐标)
//   4. 转换为世界坐标             → (x, y, θ) 序列
//   5. 纯追踪控制器模拟跟踪       → 平滑轨迹 + 控制序列
//   6. 输出: mpc_trajectory.txt + mpc_result.ppm
//
//  为什么膨胀障碍物?
//    离散 A* 路径对"点"是安全的, 但车辆有体积 (3.68m × 1.8m)。
//    膨胀后路径距离障碍物边缘至少 DILATE_CELLS × CELL_SIZE = 0.6m,
//    加上纯追踪的平滑偏离, 实际轨迹仍保持在安全区域内。
// ===================================================================

int main() {
    std::cout << "=== MPC 轨迹规划 ===" << std::endl;

    // ===== 第 1 步: 加载原始地图 =====
    auto gd = readGrid2("output/grid.txt");

    // ===== 第 2 步: 膨胀障碍物 =====
    // 车辆宽 1.8m + margin 0.2m = 2.0m 安全宽度
    // 半径方向需要 ≥1.0m = 5 cells, 取 3 cells 作为最小缓冲区
    const int DILATE_CELLS = 3;  // 每侧膨胀 3 格 = 0.6m
    auto dilated = gd.grid;
    for (int r = 0; r < gd.size; r++)
        for (int c = 0; c < gd.size; c++)
            if (gd.grid[r][c] == 1)        // 每个障碍物格子
                for (int dr = -DILATE_CELLS; dr <= DILATE_CELLS; dr++)   // 膨胀到周围
                    for (int dc = -DILATE_CELLS; dc <= DILATE_CELLS; dc++) {
                        int nr = r + dr, nc = c + dc;
                        if (nr >= 0 && nr < gd.size && nc >= 0 && nc < gd.size)
                            dilated[nr][nc] = 1;
                    }

    // 确保起/终点不被膨胀吞没 (各清理 8×8 格区域)
    for (int dr = -8; dr <= 8; dr++)
        for (int dc = -8; dc <= 8; dc++) {
            int r1 = std::max(0, std::min(gd.size - 1, gd.start_row + dr));
            int c1 = std::max(0, std::min(gd.size - 1, gd.start_col + dc));
            int r2 = std::max(0, std::min(gd.size - 1, gd.goal_row  + dr));
            int c2 = std::max(0, std::min(gd.size - 1, gd.goal_col  + dc));
            dilated[r1][c1] = 0;  // 起点区域清空
            dilated[r2][c2] = 0;  // 终点区域清空
        }

    // ===== 第 3 步: 膨胀地图上运行离散 A* =====
    // 使用 8 方向搜索, 与 astar.cpp 一致
    std::vector<std::pair<int,int>> safe_cells;  // 存储 (row, col) 序列
    {
        // 离散 A* 节点 (内嵌实现, 不依赖 astar.cpp)
        struct DP {
            int r, c;
            double g, h, f;                     // g=实际代价, h=启发式, f=g+h
            bool operator>(const DP& o) const { return f > o.f; }  // 小顶堆
        };

        const double INF = 1e9;
        std::vector<std::vector<double>> gs(gd.size,
            std::vector<double>(gd.size, INF));                // g_score 数组
        std::vector<std::vector<std::pair<int,int>>> parent(gd.size,
            std::vector<std::pair<int,int>>(gd.size, {-1, -1})); // parent 链
        std::priority_queue<DP, std::vector<DP>, std::greater<DP>> pq;  // open set

        // 起点初始化
        gs[gd.start_row][gd.start_col] = 0;
        pq.push({gd.start_row, gd.start_col, 0, 0, 0});

        // 8 方向移动: 上下左右 + 四对角
        int dirs[8][2] = {{-1,0},{1,0},{0,-1},{0,1},{-1,-1},{-1,1},{1,-1},{1,1}};
        double costs[8] = {1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414};  // 对角=√2

        bool found = false;
        while (!pq.empty()) {
            auto cur = pq.top(); pq.pop();                       // pop f 最小节点
            if (cur.r == gd.goal_row && cur.c == gd.goal_col)    // 到达终点
                { found = true; break; }
            if (cur.g > gs[cur.r][cur.c] + 1e-6) continue;      // 已有更优路径

            // 扩展 8 个邻居
            for (int d = 0; d < 8; d++) {
                int nr = cur.r + dirs[d][0], nc = cur.c + dirs[d][1];
                if (nr < 0 || nr >= gd.size || nc < 0 || nc >= gd.size) continue;
                if (dilated[nr][nc] == 1) continue;             // 障碍 (膨胀后)
                double ng = cur.g + costs[d];
                if (ng < gs[nr][nc]) {                          // 找到更优路径
                    gs[nr][nc] = ng;
                    parent[nr][nc] = {cur.r, cur.c};
                    double h = std::hypot(nr - gd.goal_row, nc - gd.goal_col); // 欧几里得启发式
                    pq.push({nr, nc, ng, h, ng + h});
                }
            }
        }

        // 回溯重建路径 (goal → start, 然后翻转)
        if (found) {
            int r = gd.goal_row, c = gd.goal_col;
            while (r != -1 && c != -1) {
                safe_cells.push_back({r, c});
                auto p = parent[r][c];
                r = p.first; c = p.second;
            }
            std::reverse(safe_cells.begin(), safe_cells.end());
            std::cout << "膨胀地图 A*: " << safe_cells.size() << " 步" << std::endl;
        }
    }

    if (safe_cells.empty()) {
        std::cerr << "膨胀后无可行路径! 尝试减小 DILATE_CELLS" << std::endl;
        return 1;
    }

    // ===== 第 4 步: 转换到世界坐标 =====
    // grid (row, col) → world (x, y, θ)
    std::vector<Pose> ref_path;
    for (size_t i = 0; i < safe_cells.size(); i++) {
        Pose p;
        p.x = safe_cells[i].second * CELL_SIZE + CELL_SIZE / 2.0;
        p.y = safe_cells[i].first  * CELL_SIZE + CELL_SIZE / 2.0;
        if (i + 1 < safe_cells.size())
            p.theta = std::atan2(safe_cells[i+1].first  - safe_cells[i].first,
                                 safe_cells[i+1].second - safe_cells[i].second);
        else if (i > 0)
            p.theta = ref_path.back().theta;
        ref_path.push_back(p);
    }

    // ===== 第 5 步: 纯追踪 MPC 轨迹生成 =====
    double total_path_len = 0;
    for (size_t i = 1; i < ref_path.size(); i++)
        total_path_len += std::hypot(ref_path[i].x - ref_path[i-1].x,
                                      ref_path[i].y - ref_path[i-1].y);

    const double V_DES = 3.0;          // 期望速度 (m/s)
    MPCTrajectoryPlanner planner(dilated);
    double plan_dt = 0.2;              // 控制周期 0.2s (5Hz)
    // N = 路径总时间 / dt, 至少 20 步
    int plan_N = std::max(20, (int)(total_path_len / (V_DES * plan_dt)) + 5);
    planner.setDt(plan_dt);
    planner.setHorizon(plan_N);
    planner.setDesiredSpeed(V_DES);

    std::cout << "路径全长: " << total_path_len << "m → N=" << plan_N
              << " dt=" << plan_dt << "s" << std::endl;

    std::vector<double> velocities, steers;
    auto mpc_traj = planner.plan(ref_path, velocities, steers);

    // ===== 第 6 步: 保存输出文件 =====

    // 6a. 轨迹文本: (x, y, θ, v, δ) 每行
    {
        std::ofstream out("output/mpc_trajectory.txt");
        out << "# MPC Trajectory\n# N=" << mpc_traj.size()
            << "\n# x y theta v delta\n";
        for (size_t k = 0; k < mpc_traj.size(); k++) {
            double v = (k < velocities.size()) ? velocities[k] : 0.0;
            double s = (k < steers.size())     ? steers[k]     : 0.0;
            out << mpc_traj[k].x << " " << mpc_traj[k].y << " "
                << mpc_traj[k].theta << " " << v << " " << s << "\n";
        }
        out.close();
        std::cout << "MPC 轨迹已保存: output/mpc_trajectory.txt" << std::endl;
    }

    // 6b. PPM 对比图
    Pose start_pose = { gd.start_col * CELL_SIZE + CELL_SIZE / 2.0,
                        gd.start_row * CELL_SIZE + CELL_SIZE / 2.0, 0.0 };
    Pose goal_pose  = { gd.goal_col  * CELL_SIZE + CELL_SIZE / 2.0,
                        gd.goal_row  * CELL_SIZE + CELL_SIZE / 2.0, 0.0 };
    saveMPCPPM(gd.grid, start_pose, goal_pose, ref_path, mpc_traj,
               "output/mpc_result.ppm");
    std::cout << "对比图已保存: output/mpc_result.ppm" << std::endl;
    std::cout << "  (灰色=参考路径, 绿色=MPC 平滑轨迹)" << std::endl;

    // ===== 控制量统计 =====
    double max_s = 0, avg_v = 0;
    for (double s : steers) max_s = std::max(max_s, std::abs(s));
    for (double v : velocities) avg_v += v;
    avg_v /= velocities.size();
    std::cout << "\n控制统计:" << std::endl;
    std::cout << "  平均速度: " << avg_v << " m/s" << std::endl;
    std::cout << "  最大转向: " << (max_s * 180 / M_PI) << "°" << std::endl;

    return 0;
}
