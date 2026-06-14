#include "lqr.h"

#include <cmath>
#include <limits>

void LQR::Init(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C,
               Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::MatrixXd S) {
    this->A_ = A;
    this->B_ = B;
    this->C_ = C;
    this->Q_ = Q;
    this->R_ = R;
    this->S_ = S;
    this->K_ = this->solve();
}

Eigen::VectorXd LQR::run(Eigen::VectorXd y_ref, Eigen::VectorXd x_obs) {
    Eigen::VectorXd x_ref = this->C_.completeOrthogonalDecomposition().solve(y_ref);
    Eigen::VectorXd u = this->K_ * (x_ref - x_obs);
    return u;
}

Eigen::MatrixXd LQR::solve() {
    int nx = this->A_.rows();  // 状态维度
    int nu = this->B_.cols();  // 输入维度
    Eigen::MatrixXd P0 = this->S_;  // 终端代价矩阵 S 为 P0
    int N_max = 100;
    Eigen::MatrixXd P(nx, nx * N_max);
    P.setZero();
    P.block(0, 0, nx, nx) = P0;
    Eigen::MatrixXd P_k_1 = P0;
    Eigen::MatrixXd P_k = P0;
    float tollimit = 1e-6f;
    float inf = std::numeric_limits<float>::infinity();
    float diff = inf;
    Eigen::MatrixXd F_k = Eigen::MatrixXd::Identity(nu, nx);

    for (int i = 0; i < N_max; i++) {
        if (diff < tollimit) {
            return F_k;
        }
        Eigen::MatrixXd F_pre = F_k;
        F_k = (this->R_ + this->B_.transpose() * P_k_1 * this->B_).inverse()
            * this->B_.transpose() * P_k_1 * this->A_;
        P_k = (this->A_ - this->B_ * F_k).transpose() * P_k * (this->A_ - this->B_ * F_k)
            + F_k.transpose() * this->R_ * F_k + this->Q_;
        P.block(0, nx * (i + 1), P.rows(), P_k.cols()) = P_k;
        P_k_1 = P_k;
        diff = (F_k - F_pre).cwiseAbs().maxCoeff();
    }
    return F_k;
}
