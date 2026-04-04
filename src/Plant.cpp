#include <iostream>
#include <random>
#include "Plant.h"

using namespace Eigen;
// using namespace std;

Plant::Plant()
{
    nx = 6;
    ny = 2;
    nu = 2;

    x = Eigen::VectorXd::Zero(nx);
    y = Eigen::VectorXd::Zero(ny);

    x0 = Eigen::VectorXd::Zero(nx);
    y0 = Eigen::VectorXd::Zero(ny);

    A = Eigen::MatrixXd::Zero(nx, nx);
    B = Eigen::MatrixXd::Zero(nx, nu);
    C = Eigen::MatrixXd::Zero(ny, nx);
    D = Eigen::MatrixXd::Zero(ny, nu);

    Q = Eigen::MatrixXd::Identity(nx, nx) * 0.00;
    R = Eigen::MatrixXd::Identity(ny, ny) * 0.00;

    t = 0.0;
}

Plant::Plant(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C, Eigen::MatrixXd D)
{
    this->t = 0.0;
    this->nx = A.rows();
    this->ny = C.rows();
    this->nu = B.cols();

    this->A = A;
    this->B = B;
    this->C = C;
    this->D = D;

    this->Q = Eigen::MatrixXd::Identity(nx, nx) * 0.00001f;
    this->R = Eigen::MatrixXd::Identity(ny, ny) * 0.001f;

    this->x = Eigen::VectorXd::Zero(nx);
    this->y = Eigen::VectorXd::Zero(ny);

    this->x0 = Eigen::VectorXd::Zero(nx);
    this->y0 = Eigen::VectorXd::Zero(ny);
    this->u0 = Eigen::VectorXd::Zero(nu);

    this->y = this->C * this->x0 + this->D * this->u0;
}

Eigen::VectorXd Plant::Init(Eigen::VectorXd x0, Eigen::VectorXd u0)
{
    this->t = 0.0;
    this->x0 = x0;
    this->u0 = u0;
    this->x = this->x0;
    this->y = this->C * this->x + this->D * this->u0;

    return this->y;
}

Eigen::VectorXd Plant::step(const Eigen::VectorXd& u, float dt)
{
    // 噪声
    this->w = this->sampleGaussian(this->Q);
    this->h = this->sampleGaussian(this->R);
    // 状态更新
    this->x = this->A * this->x + this->B * u + this->w;

    // 输出
    this->y = this->C * this->x + this->D * u + this->h;

    // 更新时间
    this->t += dt;

    return this->y;

}


float Plant::getTime()
{
    return this->t;
}

Eigen::VectorXd Plant::sampleGaussian(const Eigen::MatrixXd &cov) 
{
    int n = cov.rows();
    Eigen::VectorXd z(n);
    static std::default_random_engine gen;
    static std::normal_distribution<double> dist(0.0, 1.0);

    for (int i = 0; i < n; ++i) z(i) = dist(gen);

    Eigen::MatrixXd L = cov.llt().matrixL();
    return L * z;
}