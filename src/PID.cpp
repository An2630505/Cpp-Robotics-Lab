#include <iostream>
#include "PID.h"


// TODO 
// 1. 创建一个PID类，属性包含 kp, ki, kd, last_error, error, int_error;
// 2. 方法包含：1. init(); 2. 根据参数重载的PID
// 3. 增量式位置式，实现增量式PID
// ddl：2026.03.30


PID::PID(int n)//n维度
{
    this->n = n;
    this->init();
}

PID::PID(int n, Eigen::VectorXd kp, Eigen::VectorXd ki, Eigen::VectorXd kd)
{
    this->n = n;
    this->init();
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

// 初始化
void PID::init()
{
    this->kp = Eigen::VectorXd::Zero(this->n);
    this->ki = Eigen::VectorXd::Zero(this->n);
    this->kd = Eigen::VectorXd::Zero(this->n);
    this->output = Eigen::VectorXd::Zero(this->n);
    this->min_output = Eigen::VectorXd::Zero(this->n);    
    this->max_output = Eigen::VectorXd::Zero(this->n);
    this->last_error = Eigen::VectorXd::Zero(this->n);
    this->error = Eigen::VectorXd::Zero(this->n);
    this->int_error = Eigen::VectorXd::Zero(this->n);
    this->prev_error = Eigen::VectorXd::Zero(this->n);
}

// 参数更新
// =========================
void PID::setParam(Eigen::VectorXd kp, Eigen::VectorXd ki, Eigen::VectorXd kd,  Eigen::VectorXd min_out, Eigen::VectorXd max_out)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;

    this->min_output = min_out;
    this->max_output = max_out;
}

// 输出限幅
// =========================
Eigen::VectorXd PID::limit(Eigen::VectorXd val, Eigen::VectorXd min_val, Eigen::VectorXd max_val)
{
    return val.array().min(max_val.array()).max(min_val.array());
}

// =========================
// 1️⃣ 位置式 PID
// u(k) = kp*e + ki*∑e + kd*(e(k)-e(k-1))
// =========================
Eigen::VectorXd PID::positionPID(Eigen::VectorXd target, Eigen::VectorXd current)
{
    this->error = target - current;
    this->int_error += this->error;

    Eigen::VectorXd output(this->n);
    output = this->kp.array() * this->error.array()
                    + this->ki.array() * this->int_error.array()
                    + this->kd.array() * (this->error - this->last_error).array();

    this->min_output = -10.0f * Eigen::VectorXd::Ones(this->n);
    this->max_output =  10.0f * Eigen::VectorXd::Ones(this->n);
    // 输出限幅
    output = limit(output, min_output, max_output);

    this->last_error = this->error;

    return output;
}

// =========================
// 2️⃣ 增量式 PID（重点）
// Δu = kp*(e(k)-e(k-1))
//    + ki*e(k)
//    + kd*(e(k)-2e(k-1)+e(k-2))
// =========================
Eigen::VectorXd PID::incrementalPID(Eigen::VectorXd target, Eigen::VectorXd current)
{
    this->error = target - current;

    Eigen::VectorXd delta_u(this->n);
    delta_u  = this->kp.array() * (this->error - this->last_error).array()
                    + this->ki.array() * this->error.array()
                    + this->kd.array() * (this->error - 2 * this->last_error + this->prev_error).array();

    // 更新历史误差
    this->prev_error = this->last_error;
    this->last_error = this->error;

    this->output += delta_u;

    this->min_output = -1000.0f * Eigen::VectorXd::Ones(this->n);
    this->max_output =  1000.0f * Eigen::VectorXd::Ones(this->n);
    this->output = limit(this->output, this->min_output, this->max_output);

    return this->output;
}





