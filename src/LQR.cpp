#include <iostream>
#include <cmath>

#include "LQR.h"



void LQR::Init(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::MatrixXd S)
{
    this->A_ = A;
    this->B_ = B;
    this->Q_ = Q;
    this->R_ = R;
    this->S_ = S;

    // ！！！！初始化时需要求解控制律增益
    this->K_ = this->solve();
}

Eigen::VectorXd LQR::run(Eigen::VectorXd y_ref, Eigen::VectorXd y_obs)
{
    // Eigen::VectorXd x_est = this->C_.inverse() * y_obs;
    // Eigen::VectorXd x_ref = this->C_.inverse() * y_ref;
    Eigen::VectorXd u = this->K_ * (y_ref - y_obs);
    return u;
}

 Eigen::MatrixXd LQR::solve()
 {
    int nx= this->A_.rows();//状态维度
    int nu= this->B_.cols();//输入维度
    Eigen::MatrixXd P0= this->S_;//终端代价矩阵S为P0
    int N_max= 100;//最大迭代次数
    Eigen::MatrixXd P(nx, nx*N_max);//P存放每次迭代后的终端代价函数矩阵
    P.setZero();
    P.block(0, 0, nx, nx) = P0;
    Eigen::MatrixXd P_k_1= P0;//上一时刻P
    Eigen::MatrixXd P_k= P0;
    float tollimit= 1e-6;//稳态误差阈值
    float inf = std::numeric_limits<float>::infinity();
    float diff = inf;//初始误差
    Eigen::MatrixXd F_k = Eigen::MatrixXd::Identity(nu, nx);//初始增益为无穷大
    for (int i = 0; i < N_max; i++)
    {
        if(diff < tollimit)
        {
            return F_k;
            break;
        }
        else
        {
            Eigen::MatrixXd F_pre = F_k;
            //计算F[N-k]
            F_k = (this->R_ + this->B_.transpose() * P_k_1 * this->B_).inverse()*  this->B_.transpose() * P_k_1 * this->A_;
            //计算P[k]
            P_k = (this->A_ - this->B_ * F_k).transpose() * P_k * (this->A_ - this->B_ * F_k) 
                    + F_k.transpose() * this->R_ * F_k + this->Q_;
            //将P[k]存储到P矩阵中
            P.block(0, nu*(i+1) - nu + 1, P.rows(), P_k.cols()) = P_k; 
            //更新P[k-1]
            P_k_1 = P_k;
            diff = (F_k - F_pre).cwiseAbs().maxCoeff();
        }
    }
    return F_k;
 }