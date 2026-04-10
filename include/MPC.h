#ifndef ___MPC_H___
#define ___MPC_H___

#include <Eigen/Dense>

class MPC {
public:
    
    

    void Init(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::MatrixXd S, int N);

    void MPC_Matrices();

    Eigen::VectorXd predict(Eigen::VectorXd y_ref, Eigen::VectorXd x_obs);

    Eigen::MatrixXd blockDiagRepeat(const Eigen::MatrixXd& A, int N);


private:

    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Eigen::MatrixXd C_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    Eigen::MatrixXd S_;
    Eigen::MatrixXd E_;
    Eigen::MatrixXd H_;

    int n_;

    int nx; // 状态维度
    int ny; // 输出维度
    int nu; // 输入维度


};

#endif

