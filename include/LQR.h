#ifndef ___LQR_H___
#define ___LQR_H___

#include <Eigen/Dense>
 
// TODO
// 1. 需要完成下面三个函数
// 2. 还需要内置一个观测器干啥的，将y观测量转换为x状态量
class LQR{

public: 
    // Init 函数是需要输入建立的数学模型，并将控制律增益求解出来
    // 该函数要在系统循环前调用
    void Init(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::MatrixXd S);

    // run 函数是在系统循环的过程中调用
    // 将Init函数中求解的控制率增益，与状态反馈相乘，得到当前的控制量
    Eigen::VectorXd run(Eigen::VectorXd y_ref, Eigen::VectorXd y_obs);

private:
    // 需要在这个函数中完成LQR增益的求解
    Eigen::MatrixXd solve();

    // 私有属性的变量以后命名在最后面加一个下划线
    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    Eigen::MatrixXd S_;
    Eigen::MatrixXd K_;


};

#endif