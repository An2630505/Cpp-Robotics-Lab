#include "bicycle_model.h"

BicycleModel::BicycleModel() {}

BicycleModel::BicycleModel(Eigen::MatrixXd A, Eigen::MatrixXd B1, Eigen::MatrixXd B2,
                           Eigen::MatrixXd C, Eigen::MatrixXd D) {
    this->t = 0.0;
    this->nx = A.rows();
    this->ny = C.rows();
    this->nu = B1.cols();
    this->A = A;
    this->B1 = B1;
    this->B2 = B2;
    this->C = C;
    this->D = D;
    this->x = Eigen::VectorXd::Zero(nx);
    this->y = Eigen::VectorXd::Zero(ny);
}

Eigen::VectorXd BicycleModel::Init(Eigen::VectorXd x0, Eigen::VectorXd u0) {
    this->t = 0.0;
    this->x = x0;
    this->y = this->C * this->x + this->D * u0;
    return this->y;
}

Eigen::VectorXd BicycleModel::step(float dt, float w, const Eigen::VectorXd& u) {
    Eigen::VectorXd dot_x = this->A * this->x + this->B1 * u + this->B2 * w;
    this->x = this->x + dot_x * dt;
    this->y = this->C * this->x + this->D * u;
    this->t += dt;
    return this->y;
}

float BicycleModel::getTime() {
    return this->t;
}
