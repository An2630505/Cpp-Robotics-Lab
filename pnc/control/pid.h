#ifndef PNC_CONTROL_PID_H_
#define PNC_CONTROL_PID_H_

#include <Eigen/Dense>

class PID {
public:
    PID(int n);
    PID(int n, Eigen::VectorXd kp, Eigen::VectorXd ki, Eigen::VectorXd kd);

    void init();
    void setParam(Eigen::VectorXd kp, Eigen::VectorXd ki, Eigen::VectorXd kd,
                  Eigen::VectorXd min_out, Eigen::VectorXd max_out);

    Eigen::VectorXd positionPID(Eigen::VectorXd target, Eigen::VectorXd current);
    Eigen::VectorXd incrementalPID(Eigen::VectorXd target, Eigen::VectorXd current);
    Eigen::VectorXd limit(Eigen::VectorXd val, Eigen::VectorXd min_val, Eigen::VectorXd max_val);

private:
    int n;
    Eigen::VectorXd kp, ki, kd;
    Eigen::VectorXd error, last_error, prev_error, int_error;
    Eigen::VectorXd output, max_output, min_output;
};

#endif  // PNC_CONTROL_PID_H_
