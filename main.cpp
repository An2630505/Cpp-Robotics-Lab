#include <iostream>
#include <fstream>
#include <iomanip>

#include "main.h"
#include "PID.h"
#include "LQR.h"
#include "MPC.h"
#include "Plant.h"
#include "Object.h"
#include "KF.h"

using namespace Eigen;
using namespace std;

// 目标轨迹角速度（圆周运动）
const float w = 0.1f;

Plant plantInit(float dt, Eigen::VectorXd u) {
    // 被控对象的初始化函数
    //系统矩阵
    MatrixXd A(6, 6);
    MatrixXd B(6, 2);
    MatrixXd C(4, 6);
    MatrixXd D(4, 2);

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

    // 卡尔曼初始化
    //卡尔曼滤波器参数
    MatrixXd P_kf(6, 6);
    MatrixXd Q_kf(6, 6); 
    MatrixXd R_kf(4, 4); 

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

    Plant plant = Plant(A, B, C, D);

    //状态初始值
    Eigen::VectorXd x0(6);

    x0 << 20, 0, 0, 1, 0, 0;
    plant.y = plant.Init(x0, u);  

    // 被控对象的 KF 初始化
    plant.kf.init(A, B, C, P_kf, Q_kf, R_kf, x0);

    return plant;
}

LQR LQRInit(Plant &plant) {
    // LQR 控制器的初始化函数
    //LQR状态权重矩阵、输入权重矩阵和终端权重矩阵
    MatrixXd Q(6, 6);
    MatrixXd R(2, 2);
    MatrixXd S(6, 6);

    // Q = MatrixXd::Identity(6, 6) * 0.01;
    Q = Eigen::MatrixXd::Zero(6,6); // 赋值
    Q(0,0) = 0.1;
    Q(3,3) = 0.1;
    S = MatrixXd::Identity(6, 6) * 0.01;
    R = MatrixXd::Identity(2, 2) * 0.01;

    // //LQR控制器 
    LQR lqr;
    lqr.Init(plant.A, plant.B, plant.C, Q, R, S);

    return lqr;
}

MPC MPCInit(Plant &plant, int N) {
    // MPC控制器的初始化函数
    //MPC状态权重矩阵、输入权重矩阵和终端权重矩阵
    MatrixXd Q(6, 6);
    MatrixXd R(2, 2);
    MatrixXd S(6, 6);

    Q = Eigen::MatrixXd::Zero(6,6); // 赋值
    Q(0,0) = 10;
    Q(1,1) = 10;
    Q(3,3) = 10;
    Q(4,4) = 10;
    S = MatrixXd::Identity(6, 6) * 0.1;
    R = MatrixXd::Identity(2, 2) * 0.1;

    // //LQR控制器 
    MPC mpc;
    mpc.Init(plant.A, plant.B, plant.C, plant.Q, R, S, N);

    return mpc;
}



Object objectInit(float dt) {
    // 初始化参考目标
    MatrixXd A(4, 4);
    MatrixXd C(4, 4);
    //x,y,dx,dy
    A << 1, 0, dt, 0,
        0, 1, 0, dt, 
        0, 0, cos(w*dt),-sin(w*dt),
        0, 0, sin(w*dt), cos(w*dt);
    C << 1, 0, 0, 0,
        0, 1, 0, 0, 
        0, 0, 1, 0,
        0, 0, 0, 1;

    Object object = Object(A, Eigen::MatrixXd::Zero(4,2), C , Eigen::MatrixXd::Zero(4,2));

    //目标状态初始值
    Eigen::VectorXd obj_x0(4);
    obj_x0 << 0, 1, 1, 0;
    //目标状态估计值
    Eigen::VectorXd obj_x_hat(4);
    //初始化目标系统
    Eigen::VectorXd obj_y = object.Init(obj_x0);
    Eigen::VectorXd obj_y_meas(4);// 滤波前的测量值（带噪声）
    Eigen::VectorXd obj_y_filt(4);// 滤波后的估计值（通过观测矩阵得到位置）

    MatrixXd obj_P_kf(4, 4);
    MatrixXd obj_Q_kf(4, 4); 
    MatrixXd obj_R_kf(4, 4); 
    float sigma = 0.01f;
    // 估计测量噪声协方差
    obj_R_kf = 10.0f*(sigma * sigma / 3.0) * Eigen::MatrixXd::Identity(4,4);
    obj_P_kf = MatrixXd::Identity(4, 4) * 1;
    // 估计过程噪声协方差矩阵
    obj_Q_kf << 1*dt*dt*dt*dt,0, 1*dt*dt*dt,0,
        0 ,1*dt*dt*dt*dt, 0, 1*dt*dt*dt,
        1*dt*dt*dt , 0, 1*dt*dt*dt*dt, 0,
        0, 1*dt*dt*dt ,  0, 1*dt*dt*dt*dt;
    object.kf.init(A, Eigen::MatrixXd::Zero(4,2), C , obj_P_kf, obj_Q_kf, obj_R_kf, obj_x0);
    return object;
}

PID PIDInit(const Plant &plant) {
    int nu = plant.B.cols();  // 控制输入维度为2

    Eigen::VectorXd kp(nu);
    Eigen::VectorXd ki(nu);
    Eigen::VectorXd kd(nu);

    kp << 3.1f, 3.1f;  // x, y 方向的P参数（系统对称，增益一致）
    ki << 0.0f, 0.0f;     // I参数
    kd << 5.0f, 5.0f;     // D参数

    PID pid(nu, kp, ki, kd);
    return pid;
}

// 前馈控制：计算跟踪圆周轨迹所需的向心加速度
// u_ff_x = -w * vy_ref,  u_ff_y = w * vx_ref
Eigen::VectorXd computeFeedforward(Eigen::VectorXd y_ref, float w) {
    Eigen::VectorXd u_ff(2);
    u_ff << -w * y_ref[3],   // ax = -w * vy_ref
             w * y_ref[2];    // ay =  w * vx_ref
    return u_ff;
}


int main() {
    
    // 打开文件用于写入
    ofstream outfile("output/output.txt");
    if (!outfile.is_open()) {
        cerr << "Failed to open output file!" << endl;
        return 1;
    }

    outfile << "Step, plant.y[0], plant.y[1], plant.y[2], plant.y[3], " 
            << "object.y[0], object.y[1], object.y[2], object.y[3]" << std::endl;

    float dt = 0.1f;

    // 控制量初始化
    Eigen::VectorXd u(2);
    Eigen::VectorXd u_fb(2);
    Eigen::VectorXd u_ff(2);
    u << 0.0, 0.0;

    // 被控对象初始化
    Plant plant = plantInit(dt, u);

    int N = 50;
    // 控制器初始化
    LQR lqr = LQRInit(plant);
    PID pid = PIDInit(plant);
    MPC mpc = MPCInit(plant,N);
   
    // 跟踪目标初始化
    Object object = objectInit(dt);

    //目标值
    Eigen::VectorXd target_y(4);
    target_y << 0.0, 0.0, 0.0, 0.0;

    for (int i = 0; i < 800; i++)
    {
        // 跟踪目标单步运动
        object.y = object.step(dt, Eigen::VectorXd::Zero(2));
        // 目标的卡尔曼滤波更新（传感器测量过程）
        object.kf.x_post = object.kf.update(object.y, Eigen::VectorXd::Zero(2));
        // 被控对象自身状态感知的卡尔曼滤波更新
        plant.kf.x_post = plant.kf.update(plant.y, u);
        
        // 提取位置信息(前两个元素: x, y)用于PID控制
        Eigen::VectorXd target_pos(2);
        Eigen::VectorXd current_pos(2);
        target_pos << object.kf.y_post[0], object.kf.y_post[1];   // 目标位置 x, y
        current_pos << plant.kf.y_post[0], plant.kf.y_post[1];     // 当前位置 x, y
        
        // 控制器输出（反馈 + 前馈）
        // u_fb = lqr.run(object.kf.y_post, plant.kf.x_post);
        // u_fb = pid.positionPID(target_pos, current_pos);
        u_fb = mpc.predict(object.kf.y_post, plant.kf.x_post);
        u_ff = computeFeedforward(object.kf.y_post, w);
        u = u_fb + u_ff;
        
        // 被控对象运行
        //该死
        plant.y = plant.step(dt, u, false);

        // debug输出
        std::cout << std::fixed << std::setprecision(2) << "step= " << i << ", "
                  << "u= " << u[0] << ", " << u[1] << ", "
                  << "plant = " << plant.y[0] << ", " << plant.y[1] << ", " << plant.y[2] << ", " << plant.y[3] << ", "
                //   << "plant = " << plant.kf.y_post[0] << ", " << plant.kf.y_post[1] << ", " << plant.kf.y_post[2] << ", " << plant.kf.y_post[3] << ", "
                  << "object = " << object.kf.y_post[0] << ", " << object.kf.y_post[1] << std::endl;

        outfile << i << ", " 
                << plant.kf.y_post[0] << ", " << plant.kf.y_post[1] << ", "<< plant.kf.y_post[2] << ", " << plant.kf.y_post[3] << ", " 
                << object.kf.y_post[0] << ", " << object.kf.y_post[1] << ", "<< object.kf.y_post[2] << ", " << object.kf.y_post[3] << endl;
    
    }

    outfile.close();

    return 0;
}