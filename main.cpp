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
    MatrixXd C(4, 6);
    MatrixXd D(4, 2);

    //LQR状态权重矩阵、输入权重矩阵和终端权重矩阵
    MatrixXd Q(6, 6);
    MatrixXd R(2, 2);
    MatrixXd S(6, 6);

    //卡尔曼滤波器参数
    MatrixXd P_kf(6, 6);
    MatrixXd Q_kf(6, 6); 
    MatrixXd R_kf(4, 4); 

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
        0, 0, 0, 1, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0;

    D << 0, 0,
        0, 0,
        0, 0,
        0, 0;

    // Q = MatrixXd::Identity(6, 6) * 0.01;
    Q = Eigen::MatrixXd::Zero(6,6); // 赋值
    Q(0,0) = 0.1;
    Q(3,3) = 0.1;
    S = MatrixXd::Identity(6, 6) * 0.01;
    R = MatrixXd::Identity(2, 2) * 0.01;

    float sigma = 0.01f;
    // 估计测量噪声协方差
    R_kf = 0.0f*(sigma * sigma / 3.0) * Eigen::MatrixXd::Identity(4,4);
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

    MatrixXd obj_A(4, 4);
    MatrixXd obj_C(4, 4);
    float w=0.1;
    //x,y,dx,dy
    obj_A << 1, 0, dt, 0,
        0, 1, 0, dt, 
        0, 0, cos(w*dt),-sin(w*dt),
        0, 0, sin(w*dt), cos(w*dt);
    obj_C << 1, 0, 0, 0,
        0, 1, 0, 0, 
        0, 0, 1, 0,
        0, 0, 0, 1;

    Plant object_A = Plant(obj_A, Eigen::MatrixXd::Zero(4,2), obj_C , Eigen::MatrixXd::Zero(4,2));

    //目标状态初始值
    Eigen::VectorXd obj_x0(4);
    obj_x0 << 0, 1, 1, 0;
    //目标状态估计值
    Eigen::VectorXd obj_x_hat(4);
    //初始化目标系统
    Eigen::VectorXd obj_y = object_A.Init(obj_x0);
    Eigen::VectorXd obj_y_meas(4);// 滤波前的测量值（带噪声）
    Eigen::VectorXd obj_y_filt(4);// 滤波后的估计值（通过观测矩阵得到位置）


    MatrixXd obj_P_kf(4, 4);
    MatrixXd obj_Q_kf(4, 4); 
    MatrixXd obj_R_kf(4, 4); 
    // float sigma = 0.01f;
    // 估计测量噪声协方差
    obj_R_kf = 10.0f*(sigma * sigma / 3.0) * Eigen::MatrixXd::Identity(4,4);
    obj_P_kf = MatrixXd::Identity(4, 4) * 1;
    // 估计过程噪声协方差矩阵
    obj_Q_kf << 1*dt*dt*dt*dt,0, 1*dt*dt*dt,0,
        0 ,1*dt*dt*dt*dt, 0, 1*dt*dt*dt,
        1*dt*dt*dt , 0, 1*dt*dt*dt*dt, 0,
        0, 1*dt*dt*dt ,  0, 1*dt*dt*dt*dt;
    //卡尔曼滤波器
    KF obj_kf;
    obj_kf.init(obj_A, Eigen::MatrixXd::Zero(4,2), obj_C , obj_P_kf, obj_Q_kf, obj_R_kf, obj_x0);

    //状态初始值
    Eigen::VectorXd x0(6);

    x0 << 1, 0, 0, 1, 0, 0;

    //状态估计值
    Eigen::VectorXd x_hat(6);

    //目标值
    Eigen::VectorXd target_y(4);

    // target_y << 0, 0;
    //初始化系统
    Eigen::VectorXd y = plant.Init(x0, u);

    //系统输入维数
    int nu = B.cols();

    // //PID控制器参数
    // Eigen::VectorXd kp(nu);
    // Eigen::VectorXd ki(nu);
    // Eigen::VectorXd kd(nu);

    // kp << 2.78f, 2.78f;
    // ki << 0.0f, 0.0f;
    // kd << 0.0f, 0.0f;

    // //PID控制器
    // PID pid(nu, kp, ki, kd);
    //卡尔曼滤波器
    KF kf;
    kf.init(A, B, C, P_kf, Q_kf, R_kf, x0);
    // //LQR控制器 
    LQR lqr;
    lqr.Init(A, B, C, Q, R, S);

    Eigen::VectorXd y_meas(4);// 滤波前的测量值（带噪声）
    Eigen::VectorXd y_filt(4);// 滤波后的估计值（通过观测矩阵得到位置）

    for (int i = 0; i < 800; i++)
    {
        // u = pid.incrementalPID(target_y, y);
        // u = pid.positionPID(target_y, y);

        obj_x_hat = obj_kf.update(obj_y,Eigen::VectorXd::Zero(2));
        // // 记录滤波前的测量值（带噪声）和滤波后的估计值
        obj_y_meas = obj_y;  // 滤波前的测量值（带噪声）
        obj_y_filt = obj_C * obj_x_hat;  // 滤波后的估计值（通过观测矩阵得到位置）
        obj_y = object_A.step(dt,Eigen::VectorXd::Zero(2), false);
        
        x_hat = kf.update(y, u);
        // 记录滤波前的测量值（带噪声）和滤波后的估计值
        y_meas = y;  // 滤波前的测量值（带噪声）
        y_filt = C * x_hat;  // 滤波后的估计值（通过观测矩阵得到位置）
        u = lqr.run(obj_y_filt, x_hat);
        y = plant.step(dt,u);


    //     outfile << i << ", " << y_meas[0] << ", " << y_meas[1] << ", " << y_filt[0] << ", " << y_filt[1] << ", " << u[0] << ", " << u[1] << endl;
    std::cout.precision(2);
    std::cout << "step= " << i << "      x = " << x_hat.transpose() << "      y = " << y.transpose() << std::endl;

    outfile << i << ", " << y_filt[0] << ", " << y_filt[1] << ", "<< y_filt[2] << ", " << y_filt[3] << ", " << obj_y_filt[0] << ", " << obj_y_filt[1] << ", "<< obj_y_filt[2] << ", " << obj_y_filt[3] << endl;
   
    }

    outfile.close();

    return 0;
}