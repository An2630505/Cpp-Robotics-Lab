#ifndef ___PATH_H___
#define ___PATH_H___

#include <Eigen/Dense>
#include <string>

class Path {
public:
    virtual ~Path() = default;

    // 给定弧长 s，返回 [x, y, psi_ref, kappa]
    virtual Eigen::VectorXd getState(float s) = 0;

    // 给定车辆位置 [x, y, psi]，找到最近路径点
    // 输出: s(弧长), e_y(侧向误差), e_psi(航向误差), kappa(曲率)
    virtual void findNearest(const Eigen::VectorXd &pos, float &s,
                             float &e_y, float &e_psi, float &kappa) = 0;

    // 生成 MPC 预测时域内 N+1 步的参考轨迹（每行为 [e_y, de_y, e_psi, de_psi]）
    // 误差坐标系下参考始终为零，留出接口便于未来扩展
    virtual Eigen::MatrixXd getReferenceTrajectory(float s_start, int N, float dt, float Vx)
    {
        return Eigen::MatrixXd::Zero(N + 1, 4);
    }

    // 返回 REF 头信息字符串（供 Python 画图解析）
    virtual std::string getRefString(float dt, float Vx) const = 0;
};

#endif
