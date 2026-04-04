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

    outfile << "Step, y_meas[0], y_meas[1], y_filt[0], y_filt[1], u[0], u[1]" << endl;

    float dt = 0.1f;

    //系统矩阵
    MatrixXd A(6, 6);
    MatrixXd B(6, 2);
    MatrixXd C(2, 6);
    MatrixXd D(2, 2);

    //LQR状态权重矩阵、输入权重矩阵和终端权重矩阵
    MatrixXd Q(6, 6);
    MatrixXd R(2, 2);
    MatrixXd S(6, 6);

    //卡尔曼滤波器参数
    MatrixXd P_kf(6, 6);
    MatrixXd Q_kf(6, 6); 
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

    // Q = MatrixXd::Identity(6, 6) * 0.01;
    Q = Eigen::MatrixXd::Zero(6,6); // 赋值
    Q(0,0) = 0.1;
    Q(3,3) = 0.1;
    S = MatrixXd::Identity(6, 6) * 0.01;
    R = MatrixXd::Identity(2, 2) * 0.01;

    float sigma = 0.01f;
    // 估计测量噪声协方差
    R_kf = 10.0f*(sigma * sigma / 3.0) * Eigen::MatrixXd::Identity(2,2);
    P_kf = MatrixXd::Identity(6, 6) * 1;
    // 估计过程噪声协方差矩阵
    Q_kf << 1*dt*dt*dt*dt, 1*dt*dt*dt, 0, 0, 0, 0,
        1*dt*dt*dt, 1*dt*dt*dt*dt, dt, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1*dt*dt*dt*dt, 1*dt*dt*dt, 0,
        0, 0, 0, 1*dt*dt*dt, 1*dt*dt*dt*dt, 0,
        0, 0, 0, 0, 0, 1;

    Eigen::VectorXd u(2);
    u << 0.0, 0.0;

    //真实系统
    Plant plant = Plant(A, B, C, D);

    //状态初始值
    Eigen::VectorXd x0(6);

    x0 << 1, 0, 0, 1, 0.1, 0.0;

    //状态估计值
    Eigen::VectorXd x_hat(6);

    //目标值
    Eigen::VectorXd target_y(2);

    target_y << 0, 0;
    //初始化系统
    Eigen::VectorXd y = plant.Init(x0, u);

    //系统输入维数
    int nu = B.cols();

    //PID控制器参数
    Eigen::VectorXd kp(nu);
    Eigen::VectorXd ki(nu);
    Eigen::VectorXd kd(nu);

    kp << 2.78f, 2.78f;
    ki << 0.0f, 0.0f;
    kd << 0.0f, 0.0f;

    //PID控制器
    PID pid(nu, kp, ki, kd);
    //卡尔曼滤波器
    KF kf;
    kf.init(A, B, C, P_kf, Q_kf, R_kf, x0);
    //LQR控制器 
    LQR lqr;
    lqr.Init(A, B, C, Q, R, S);

    Eigen::VectorXd y_meas(2);// 滤波前的测量值（带噪声）
    Eigen::VectorXd y_filt(2);// 滤波后的估计值（通过观测矩阵得到位置）

    for (int i = 0; i < 5000; i++)
    {
        // u = pid.incrementalPID(target_y, y);
        // u = pid.positionPID(target_y, y);
        
        x_hat = kf.update(u, y);

        // 记录滤波前的测量值（带噪声）和滤波后的估计值
        y_meas = y;  // 滤波前的测量值（带噪声）
        y_filt = C * x_hat;  // 滤波后的估计值（通过观测矩阵得到位置）
        u = lqr.run(target_y, x_hat);
        // u = pid.incrementalPID(target_y, y_filt);
        // u = pid.positionPID(target_y, y_filt);

        y = plant.step(u, dt);

   


        //std::cout << "step= " << i << "      x = " << x_hat.transpose() << "      y = " << y.transpose() <<  "      u = " << u.transpose() << std::endl;
        std::cout.precision(2);
        std::cout << "step= " << i << "      x = " << x_hat.transpose() << "      y = " << y.transpose() <<  "      u = " << u.transpose() << std::endl;
    
        outfile << i << ", " << y_meas[0] << ", " << y_meas[1] << ", " << y_filt[0] << ", " << y_filt[1] << ", " << u[0] << ", " << u[1] << endl;
    }

    outfile.close();

    return 0;
}