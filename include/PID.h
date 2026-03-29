#ifndef ___PID_H___
#define ___PID_H___

#include <Eigen/Dense>

// TODO
// 1. 后续PID和LQR函数要统一，统一成一样的格式
// 2. 对外不暴露PID实现方法
class PID
{
private:

    int n;//控制量维度
    // Eigen::VectorXd kp, ki, kd;
    Eigen::VectorXd kp, ki, kd;

    Eigen::VectorXd error;
    Eigen::VectorXd last_error;
    Eigen::VectorXd prev_error;
    Eigen::VectorXd int_error;

    Eigen::VectorXd output;
    Eigen::VectorXd max_output;
    Eigen::VectorXd min_output;

public:
    // 构造函数
    PID(int n);
    PID(int n, Eigen::VectorXd kp, Eigen::VectorXd ki, Eigen::VectorXd kd);

    // 初始化
    void init();

    // 参数设置
    void setParam(Eigen::VectorXd kp, Eigen::VectorXd ki, Eigen::VectorXd kd, Eigen::VectorXd min_out, Eigen::VectorXd max_out);

    // 位置式PID
    Eigen::VectorXd positionPID(Eigen::VectorXd target, Eigen::VectorXd current);
    
    // 核心算法（增量式PID）
    Eigen::VectorXd incrementalPID(Eigen::VectorXd target, Eigen::VectorXd current);

    // 限幅函数
    Eigen::VectorXd limit(Eigen::VectorXd val, Eigen::VectorXd min_val, Eigen::VectorXd max_val);

};

#endif