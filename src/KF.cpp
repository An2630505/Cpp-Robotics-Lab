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
    x_hat = x0;      
    x_post = x0;     
    y_hat = C * x0;      // 观测向量先验估计值
    y_meas = y_hat;     // 观测向量测量值
    y_post = y_hat;     // 观测向量后验估计值
    
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
    
    // 先验估计过程
    y_hat = H_ * x_hat_;
    x_hat = x_hat_;
}

void KF::correct(Eigen::VectorXd measurement_)
{
    this->K_ = this->P_ * this->H_.transpose() * (this->H_ * this->P_ * this->H_.transpose() + this->R_).inverse();
    this->x_hat_ = this->x_hat_ + this->K_ * (measurement_ - this->H_ * this->x_hat_);
    this->P_ = (Eigen::MatrixXd::Identity(this->n_, this->n_) - this->K_ * this->H_) * this->P_;

    // 后验估计修正过程
    x_post = x_hat_;
    y_meas = measurement_;
    y_post = H_ * x_post;
}

Eigen::VectorXd KF::update(Eigen::VectorXd measurement_, Eigen::VectorXd u)
{ 
    KF::predict(u);
    KF::correct(measurement_);
    return x_post;
}
