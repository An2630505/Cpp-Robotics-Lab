#include "mpc.h"

#include <cmath>

void MPC::Init(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C,
               Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::MatrixXd S, int N) {
    this->A_ = A;
    this->B_ = B;
    this->C_ = C;
    this->Q_ = Q;
    this->R_ = R;
    this->S_ = S;
    this->n_ = N;
    this->MPC_Matrices();
}

void MPC::MPC_Matrices() {
    nx = this->A_.rows();  // 状态维度
    nu = this->B_.cols();  // 输入维度

    // 创建 M，大小为 (N+1)*nx × nx
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero((this->n_ + 1) * nx, nx);
    M.block(0, 0, nx, nx) = Eigen::MatrixXd::Identity(nx, nx);

    Eigen::MatrixXd G = Eigen::MatrixXd::Zero((this->n_ + 1) * nx, this->n_ * nu);

    Eigen::MatrixXd tmp = Eigen::MatrixXd::Identity(nx, nx);

    for (int i = 0; i < this->n_; i++) {
        int row_start = (i + 1) * nx;

        Eigen::MatrixXd G_prev = G.block(i * nx, 0, nx, this->n_ * nu);

        Eigen::MatrixXd left_block = tmp * this->B_;
        Eigen::MatrixXd right_block = G_prev.leftCols((this->n_ - 1) * nu);

        Eigen::MatrixXd G_row(nx, this->n_ * nu);
        G_row << left_block, right_block;

        G.block(row_start, 0, nx, this->n_ * nu) = G_row;

        tmp = this->A_ * tmp;

        M.block(row_start, 0, nx, nx) = tmp;
    }

    // Q_bar 和 R_bar
    Eigen::MatrixXd Q_bar = blockDiagRepeat(this->Q_, this->n_);

    Eigen::MatrixXd Q_bar_full = Eigen::MatrixXd::Zero(
        Q_bar.rows() + this->S_.rows(),
        Q_bar.cols() + this->S_.cols());
    Q_bar_full.block(0, 0, Q_bar.rows(), Q_bar.cols()) = Q_bar;
    Q_bar_full.block(Q_bar.rows(), Q_bar.cols(),
                     this->S_.rows(), this->S_.cols()) = this->S_;

    Eigen::MatrixXd R_bar = this->blockDiagRepeat(this->R_, this->n_);

    // 计算 G, E, H
    Eigen::MatrixXd F = M.transpose() * Q_bar_full * M;

    this->E_ = G.transpose() * Q_bar_full * M;

    this->H_ = G.transpose() * Q_bar_full * G + R_bar;
}

Eigen::MatrixXd MPC::blockDiagRepeat(const Eigen::MatrixXd& A, int N) {
    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(N * A.rows(), N * A.cols());
    for (int i = 0; i < N; ++i) {
        result.block(i * A.rows(), i * A.cols(), A.rows(), A.cols()) = A;
    }
    return result;
}

Eigen::VectorXd MPC::predict(Eigen::VectorXd y_ref, Eigen::VectorXd x_obs) {
    Eigen::VectorXd x_ref = this->C_.completeOrthogonalDecomposition().solve(y_ref);

    Eigen::VectorXd f = this->E_ * (x_obs - x_ref);  // (N*p) × 1

    // 解 QP（无约束快速版）
    Eigen::VectorXd U_k = -this->H_.ldlt().solve(f);

    // 取第一个控制输入
    Eigen::VectorXd u_k = U_k.head(nu);

    return u_k;
}
