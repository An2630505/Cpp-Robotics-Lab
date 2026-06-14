#include "kf.h"

void KF::init(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C,
              Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::MatrixXd R,
              Eigen::VectorXd x0) {
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
    y_hat = C * x0;
    y_meas = y_hat;
    y_post = y_hat;
}

void KF::predict(Eigen::VectorXd u) {
    this->x_hat_ = this->A_ * this->x_hat_ + this->B_ * u;
    this->P_ = this->A_ * this->P_ * this->A_.transpose() + this->Q_;
    y_hat = H_ * x_hat_;
    x_hat = x_hat_;
}

void KF::correct(Eigen::VectorXd measurement) {
    this->K_ = this->P_ * this->H_.transpose()
             * (this->H_ * this->P_ * this->H_.transpose() + this->R_).inverse();
    this->x_hat_ = this->x_hat_ + this->K_ * (measurement - this->H_ * this->x_hat_);
    this->P_ = (Eigen::MatrixXd::Identity(this->n_, this->n_)
                - this->K_ * this->H_) * this->P_;
    x_post = x_hat_;
    y_meas = measurement;
    y_post = H_ * x_post;
}

Eigen::VectorXd KF::update(Eigen::VectorXd measurement, Eigen::VectorXd u) {
    predict(u);
    correct(measurement);
    return x_post;
}
