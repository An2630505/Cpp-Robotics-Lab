#include <iostream>
#include "KF.h"



void KF::init(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C, Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::MatrixXd R,Eigen::VectorXd x0)
{   
    this->n_ = x0.size();
    this->A_ = A;
    this->B_ = B;
    this->H_ = C;
    this->P_ = P;
    this->Q_ = Q;
    this->R_ = R;

    this->x_hat_ = x0;
    
}

    // 完成卡尔曼滤波的预测步骤
    // TODO
    // 1. 预测状态向量x_hat_
    // 2. 预测协方差矩阵P_
void KF::predict(Eigen::VectorXd u)
{
    //u_ = u;
    this->x_hat_ = this->A_ * this->x_hat_ + this->B_ * u ;
    this ->P_=this->A_ * this->P_ * this->A_.transpose() + this->Q_;
}

    // 完成卡尔曼滤波的修正步骤
    // 输入：测量值measurement_
    // TODO
    // 1. 计算卡尔曼增益；
    // 2. 利用测量值修正预测值，更新状态量；
    // 3. 更新协方差矩阵；
void KF::correct(Eigen::VectorXd measurement_)
{
    this->K_ = this->P_ * this->H_.transpose() * (this->H_ * this->P_ * this->H_.transpose() + this->R_).inverse();
    this->x_hat_ = this->x_hat_ + this->K_ * (measurement_ - this->H_ * this->x_hat_);
    this->P_ = (Eigen::MatrixXd::Identity(this->n_, this->n_) - this->K_ * this->H_) * this->P_;
}

Eigen::VectorXd KF::update(Eigen::VectorXd u,Eigen::VectorXd measurement_)
{ 
    KF::predict(u);
    KF::correct(measurement_);
    return this->x_hat_;
}
