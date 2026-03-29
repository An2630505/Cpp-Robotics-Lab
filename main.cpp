#include <iostream>
#include <fstream>

#include "main.h"
#include "PID.h"
#include "LQR.h"
#include "Plant.h"

using namespace Eigen;
using namespace std;

int main() {
    
        // 打开文件用于写入
    ofstream outfile("output.txt");
    if (!outfile.is_open()) {
        cerr << "Failed to open output file!" << endl;
        return 1;
    }

    outfile << "Step, y[0], y[1], u[0], u[1]" << endl;

    float dt = 0.1f;

    MatrixXd A(6, 6);
    MatrixXd B(6, 2);
    MatrixXd C(2, 6);
    MatrixXd D(2, 2);

    MatrixXd Q(6, 6);
    MatrixXd R(2, 2);
    MatrixXd S(6, 6);

    //x,dx,ddx,y,dy,ddy
    A << 1, dt, 0.5f*dt*dt, 0, 0, 0,
        0, 1, dt, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, dt, 0.5f*dt*dt,
        0, 0, 0, 0, 1, dt,
        0, 0, 0, 0, 0, 1;

    B << 0, 0,
        0, 0,
        1, 0,
        0, 0,
        0, 0,
        0, 1;

    C << 1, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0;

    D << 0, 0,
         0, 0;

    Q = MatrixXd::Identity(6, 6) * 0.01;
    S = MatrixXd::Identity(6, 6) * 0.01;
    R = MatrixXd::Identity(2, 2) * 0.01;

    Eigen::VectorXd u(2);
    u << 0.0, 0.0;

    Plant plant = Plant(A, B, C, D);


    Eigen::VectorXd x0(6);

    x0 << 1, 0, 0, 1, 0.1, 0.0;

    Eigen::VectorXd target_y(2);

    target_y << 0, 0;
    
    Eigen::VectorXd y = plant.Init(x0, u);

    int nu = B.cols();

    Eigen::VectorXd kp(nu);
    Eigen::VectorXd ki(nu);
    Eigen::VectorXd kd(nu);

    kp << 50, 50;
    ki << 0, 0;
    kd << 0, 0;


    PID pid(nu, kp, ki, kd);

    // LQR lqr;

    // lqr.Init(A, B, Q, R, S);

    for (int i = 0; i < 10000; i++)
    {
        u = pid.incrementalPID(target_y, y);
        // lqr是状态反馈，但是我们这里只有观测值
        // 那你还要有一个观测器
        // u = lqr.run(target_y, y);

        y = plant.step(u, dt);
        
        std::cout << "step= " << i << "      y = " << y.transpose() <<  "      u = " << u.transpose() << std::endl;
    
        outfile << i << ", " << y[0] << ", " << y[1] << ", " << u[0] << ", " << u[1] << endl;
    }

    outfile.close();

    return 0;
}