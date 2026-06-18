#ifndef PNC_CONTROL_LQR_H_
#define PNC_CONTROL_LQR_H_

#include <Eigen/Dense>

class LQR {
public:
    void Init(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C,
              Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::MatrixXd S);

    Eigen::VectorXd run(Eigen::VectorXd y_ref, Eigen::VectorXd x_obs);

private:
    Eigen::MatrixXd solve();

    Eigen::MatrixXd A_, B_, C_, Q_, R_, S_, K_;
};

#endif  // PNC_CONTROL_LQR_H_
