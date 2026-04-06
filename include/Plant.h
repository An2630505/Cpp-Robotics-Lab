#ifndef ___PLANT_H___
#define ___PLANT_H___

#include <Eigen/Dense>
#include <vector>
#include <iostream>

class Plant
{
public:
    // 构造函数
    Plant();

    Plant(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C, Eigen::MatrixXd D);

    Eigen::VectorXd Init(Eigen::VectorXd x0, Eigen::VectorXd u0 = Eigen::VectorXd::Zero(2));

    // 单步运行
    Eigen::VectorXd step( float dt, const Eigen::VectorXd& u = Eigen::VectorXd::Zero(2), bool f = true);

    float getTime();

private:

    //高斯噪声
    Eigen::VectorXd sampleGaussian(const Eigen::MatrixXd &cov);
    Eigen::VectorXd generateFrictionDisturbance(const Eigen::VectorXd &x, float mu, std::vector<int> vel_indices, float noise_std ); 

    // ===== 系统维度 =====
    int nx; // 状态维度
    int ny; // 输出维度
    int nu; // 输入维度

    // ===== 状态变量 =====
    Eigen::VectorXd x;   // 当前状态
    Eigen::VectorXd y;   // 当前输出

    Eigen::VectorXd w;   // 过程噪声
    Eigen::VectorXd h;   // 过程噪声

    Eigen::VectorXd x0;  // 初始状态
    Eigen::VectorXd u0;  // 初始输入
    Eigen::VectorXd y0;  // 初始输出

    // ===== 系统矩阵 =====
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd C;
    Eigen::MatrixXd D;

    // ===== 噪声 =====
    Eigen::MatrixXd Q; // 过程噪声协方差
    Eigen::MatrixXd R; // 观测噪声协方差

    // ===== 时间 =====
    double t;
};

#endif