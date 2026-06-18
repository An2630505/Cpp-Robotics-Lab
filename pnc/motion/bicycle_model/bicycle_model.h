#ifndef PNC_MOTION_BICYCLE_MODEL_H_
#define PNC_MOTION_BICYCLE_MODEL_H_

#include <Eigen/Dense>
#include "../../control/kf/kf.h"

/// 自行车模型 — 车辆横向动力学被控对象
class BicycleModel {
public:
    int nx, ny, nu;
    float t;

    Eigen::VectorXd x;  // 当前状态 [e1, de1, e2, de2]
    Eigen::VectorXd y;  // 当前输出

    // 系统矩阵
    Eigen::MatrixXd A, B1, B2, C, D;

    KF kf;

    BicycleModel();
    BicycleModel(Eigen::MatrixXd A, Eigen::MatrixXd B1, Eigen::MatrixXd B2,
                 Eigen::MatrixXd C, Eigen::MatrixXd D);

    Eigen::VectorXd Init(Eigen::VectorXd x0,
                         Eigen::VectorXd u0 = Eigen::VectorXd::Zero(1));

    /// 单步运行: x_{k+1} = A*x_k + B1*u + B2*w (欧拉积分)
    Eigen::VectorXd step(float dt, float w,
                         const Eigen::VectorXd& u = Eigen::VectorXd::Zero(1));

    float getTime();
};

#endif  // PNC_MOTION_BICYCLE_MODEL_H_
