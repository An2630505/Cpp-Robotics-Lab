#ifndef PNC_CONTROL_KF_H_
#define PNC_CONTROL_KF_H_

#include <Eigen/Dense>

class KF {
public:
    Eigen::VectorXd x_hat;   // 状态向量先验估计值
    Eigen::VectorXd x_post;  // 状态向量后验估计值
    Eigen::VectorXd y_hat;   // 观测向量先验估计值
    Eigen::VectorXd y_meas;  // 观测向量测量值
    Eigen::VectorXd y_post;  // 观测向量后验估计值

    void init(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C,
              Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::MatrixXd R,
              Eigen::VectorXd x0);

    void predict(Eigen::VectorXd u);
    void correct(Eigen::VectorXd measurement);
    Eigen::VectorXd update(Eigen::VectorXd measurement,
                           Eigen::VectorXd u = Eigen::VectorXd::Zero(2));

private:
    Eigen::MatrixXd A_, B_, H_, Q_, R_, P_, K_;
    Eigen::VectorXd x_hat_;
    int n_;
};

#endif  // PNC_CONTROL_KF_H_
