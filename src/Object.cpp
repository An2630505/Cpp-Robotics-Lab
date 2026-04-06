#include <random>
#include "Object.h"

using namespace Eigen;
// using namespace std;

Object::Object()
{

}

Object::Object(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C, Eigen::MatrixXd D)
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

Eigen::VectorXd Object::Init(Eigen::VectorXd x0, Eigen::VectorXd u0)
{
    this->t = 0.0;
    this->x0 = x0;
    this->u0 = u0;
    this->x = this->x0;
    this->y = this->C * this->x + this->D * this->u0;

    return this->y;
}

Eigen::VectorXd Object::step(float dt , const Eigen::VectorXd& u)
{

    // 状态更新
    this->x = this->A * this->x + this->B * u;
    // 输出
    this->y = this->C * this->x + this->D * u;

    // 更新时间
    this->t += dt;

    return this->y;

}


float Object::getTime()
{
    return this->t;
}

Eigen::VectorXd Object::sampleGaussian(const Eigen::MatrixXd &cov) 
{
    int n = cov.rows();
    Eigen::VectorXd z(n);
    static std::default_random_engine gen;
    static std::normal_distribution<double> dist(0.0, 1.0);

    for (int i = 0; i < n; ++i) z(i) = dist(gen);

    Eigen::MatrixXd L = cov.llt().matrixL();
    return L * z;
}

// 函数：生成摩擦扰动
// x: 当前状态向量
// mu: 摩擦系数（库仑摩擦）
// vel_indices: 速度所在状态索引（如 ẋ,ẏ 对应 0,1）
// noise_std: 可选随机扰动标准差（默认为0）
Eigen::VectorXd Object::generateFrictionDisturbance(
    const Eigen::VectorXd &x,
    float mu,
    std::vector<int> vel_indices, 
    float noise_std
) 
{
    int n = x.size();
    Eigen::VectorXd w = Eigen::VectorXd::Zero(n);

    // 随机数生成器
    static std::default_random_engine gen;
    std::normal_distribution<float> dist(0.0, noise_std);

    for(int idx : vel_indices){
        float sign_v = (x(idx) > 0) ? 1.0 : ((x(idx) < 0) ? -1.0 : 0.0);
        w(idx) = -mu * sign_v + dist(gen); // 库仑摩擦 + 随机噪声
    }

    return w;
}