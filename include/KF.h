#ifndef ___KF_H___
#define ___KF_H___

#include <Eigen/Dense>

class KF {
public:
    
    Eigen::VectorXd x_hat;      // 状态向量先验估计值
    Eigen::VectorXd x_post;     // 状态向量后验估计值
    Eigen::VectorXd y_hat;      // 观测向量先验估计值
    Eigen::VectorXd y_meas;     // 观测向量测量值
    Eigen::VectorXd y_post;     // 观测向量后验估计值

    void init(Eigen::MatrixXd A, Eigen::MatrixXd B,Eigen::MatrixXd C,  Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::VectorXd x0);

    // 完成卡尔曼滤波的预测步骤
    // TODO
    // 1. 预测状态向量x_hat_
    // 2. 预测协方差矩阵P_
    void predict(Eigen::VectorXd u);

    // 完成卡尔曼滤波的修正步骤
    // 输入：测量值measurement_
    // TODO
    // 1. 计算卡尔曼增益；
    // 2. 利用测量值修正预测值，更新状态量；
    // 3. 更新协方差矩阵；
    void correct(Eigen::VectorXd measurement_);

    Eigen::VectorXd update( Eigen::VectorXd measurement_, Eigen::VectorXd u = Eigen::VectorXd::Zero(2));

private:

    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd K_;

    Eigen::VectorXd x_hat_;
    int n_;

};

#endif

