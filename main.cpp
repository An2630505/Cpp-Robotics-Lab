#include <iostream>
#include <fstream>

#include "main.h"
#include "PID.h"
#include "LQR.h"
#include "Plant.h"
#include "KF.h"

using namespace Eigen;
using namespace std;



int main() {
    
        // 打开文件用于写入
    ofstream outfile("output/output.txt");
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

    MatrixXd P_kf(6, 6);
    MatrixXd R_kf(2, 2);

    //x,dx,ddx,y,dy,ddy
    A << 1, dt, 0.5f*dt*dt, 0, 0, 0,
        0, 1, dt, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, dt, 0.5f*dt*dt,
        0, 0, 0, 0, 1, dt,
        0, 0, 0, 0, 0, 0;

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

    P_kf = MatrixXd::Identity(6, 6) * 1;
    MatrixXd Q_kf(6, 6); 
    Q_kf << 1*dt*dt*dt*dt, 1*dt*dt*dt, 0, 0, 0, 0,
        1*dt*dt*dt, 1*dt*dt*dt*dt, dt, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1*dt*dt*dt*dt, 1*dt*dt*dt, 0,
        0, 0, 0, 1*dt*dt*dt, 1*dt*dt*dt*dt, 0,
        0, 0, 0, 0, 0, 1;
    
    R_kf = MatrixXd::Identity(2, 2) * 10;

    Eigen::VectorXd u(2);
    u << 0.0, 0.0;

    Plant plant = Plant(A, B, C, D);


    Eigen::VectorXd x0(6);

    x0 << 1, 0, 0, 1, 0.1, 0.0;

    Eigen::VectorXd x_hat(6);

    Eigen::VectorXd target_y(2);

    target_y << 0, 0;
    
    Eigen::VectorXd y = plant.Init(x0, u);

    int nu = B.cols();

    Eigen::VectorXd kp(nu);
    Eigen::VectorXd ki(nu);
    Eigen::VectorXd kd(nu);

    kp << 0.01f, 0.01f;
    ki << 0, 0.0;
    kd << 1.0f, 1.0f;


    PID pid(nu, kp, ki, kd);

    KF kf;
    kf.init(A, B, C, P_kf, Q_kf, R_kf, x0);

    // LQR lqr;

    // lqr.Init(A, B, Q, R, S);

    for (int i = 0; i < 300; i++)
    {
        //u = pid.incrementalPID(target_y, y);
        // u = pid.positionPID(target_y, y);

        // u = lqr.run(target_y, y);

        x_hat = kf.update(u, y);



        y = plant.step(u, dt);

        y = y + (rand() / (double)RAND_MAX) * 0.01 * Eigen::VectorXd::Ones(2);
        
        //std::cout << "step= " << i << "      y = " << y.transpose() <<  "      u = " << u.transpose() << std::endl;
        std::cout.precision(2);
        std::cout << "step= " << i << "      x = " << x_hat.transpose() << "      y = " << y.transpose() <<  "      u = " << u.transpose() << std::endl;
    
        outfile << i << ", " << y[0] << ", " << y[1] << ", " << u[0] << ", " << u[1] << endl;
    }

    outfile.close();

    return 0;
}