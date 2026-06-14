#ifndef PNC_CONTROL_MPC_H_
#define PNC_CONTROL_MPC_H_

#include <Eigen/Dense>

class MPC {
public:
    void Init(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C,
              Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::MatrixXd S, int N);

    Eigen::VectorXd predict(Eigen::VectorXd y_ref, Eigen::VectorXd x_obs);

private:
    void MPC_Matrices();
    Eigen::MatrixXd blockDiagRepeat(const Eigen::MatrixXd& A, int N);

    Eigen::MatrixXd A_, B_, C_;
    Eigen::MatrixXd Q_, R_, S_;
    Eigen::MatrixXd E_, H_;
    int n_;
    int nx, ny, nu;
};

#endif  // PNC_CONTROL_MPC_H_
