#include <iostream>
#include <fstream>
#include <iomanip>

#include "main.h"
#include "PID.h"
#include "LQR.h"
#include "MPC.h"
#include "Plant.h"
#include "Plant_car.h"
#include "Object.h"
#include "KF.h"
#include "Path.h"
#include "CirclePath.h"
#include "SlalomPath.h"
#include "StraightPath.h"
#include "ComplexPath.h"


using namespace Eigen;
using namespace std;

// 车辆参数
const float PI = 3.14159265358979323846f;

const float m=1573.0f;
const float Iz=2873.0f;
const float lf=1.1;
const float lr=1.58;
const float C_af=80000.0f;
const float C_ar=80000.0f;

const float Vx=10.0f;

// Plant plantInit(float dt, Eigen::VectorXd u) {
//     // 被控对象的初始化函数
//     //系统矩阵
//     MatrixXd A(6, 6);
//     MatrixXd B(6, 2);
//     MatrixXd C(4, 6);
//     MatrixXd D(4, 2);

//     //x,dx,ddx,y,dy,ddy
//     A << 1, dt, 0.5f*dt*dt, 0, 0, 0,
//         0, 1, dt, 0, 0, 0,
//         0, 0, 0, 0, 0, 0,
//         0, 0, 0, 1, dt, 0.5f*dt*dt,
//         0, 0, 0, 0, 1, dt,
//         0, 0, 0, 0, 0, 0;

//     B << 0, 0,
//         0, 0,
//         1, 0,
//         0, 0,
//         0, 0,
//         0, 1;

//     C << 1, 0, 0, 0, 0, 0,
//         0, 0, 0, 1, 0, 0,
//         0, 1, 0, 0, 0, 0,
//         0, 0, 0, 0, 1, 0;

//     D << 0, 0,
//         0, 0,
//         0, 0,
//         0, 0;

//     // 卡尔曼初始化
//     //卡尔曼滤波器参数
//     MatrixXd P_kf(6, 6);
//     MatrixXd Q_kf(6, 6); 
//     MatrixXd R_kf(4, 4); 

//     float sigma = 0.01f;
//     // 估计测量噪声协方差
//     R_kf = 0.0f*(sigma * sigma / 3.0) * Eigen::MatrixXd::Identity(4,4);
//     P_kf = MatrixXd::Identity(6, 6) * 1;
//     // 估计过程噪声协方差矩阵
//     Q_kf << 1*dt*dt*dt*dt, 1*dt*dt*dt, 0, 0, 0, 0,
//         1*dt*dt*dt, 1*dt*dt*dt*dt, dt, 0, 0, 0,
//         0, 0, 1, 0, 0, 0,
//         0, 0, 0, 1*dt*dt*dt*dt, 1*dt*dt*dt, 0,
//         0, 0, 0, 1*dt*dt*dt, 1*dt*dt*dt*dt, 0,
//         0, 0, 0, 0, 0, 1;

//     Plant plant = Plant(A, B, C, D);

//     //状态初始值
//     Eigen::VectorXd x0(6);

//     x0 << 20, 0, 0, 1, 0, 0;
//     plant.y = plant.Init(x0, u);  

//     // 被控对象的 KF 初始化
//     plant.kf.init(A, B, C, P_kf, Q_kf, R_kf, x0);

//     return plant;
// }

Plant_car plant_car_Init(float dt, Eigen::VectorXd u) {
    // 被控对象的初始化函数
    //系统矩阵
    MatrixXd A(4, 4);
    MatrixXd B1(4, 1);
    MatrixXd B2(4, 1);
    MatrixXd C(4, 4);
    MatrixXd D(4, 1);

    
    //e1 de1 e2 de2
    A <<0, 1, 0, 0,
        0, -(2*C_af+2*C_ar)/(m*Vx),  (2*C_af+2*C_ar)/m,  -(2*C_af*lf - 2*C_ar*lr)/(m*Vx),
        0, 0, 0, 1,
        0, -(2*C_af*lf - 2*C_ar*lr)/(Iz*Vx),  (2*C_af*lf - 2*C_ar*lr)/Iz,  -(2*C_af*lf*lf + 2*C_ar*lr*lr)/(Iz*Vx);
    
    B1 << 0, 
        2*C_af/m, 
        0,
        2*C_af*lf/Iz;

    B2 << 0, 
        -(2*C_af*lf-2*C_ar*lr)/m/Vx-Vx, 
        0,
        -(2*C_af*lf*lf+2*C_ar*lr*lr)/Iz/Vx;

    C << 1, 0, 0, 0, 
        0, 1, 0, 0,
        0, 0, 1, 0, 
        0, 0, 0, 1;

    D << 0,
        0,
        0,
        0;

    // 卡尔曼初始化
    //卡尔曼滤波器参数
    MatrixXd P_kf(4, 4);
    MatrixXd Q_kf(4, 4); 
    MatrixXd R_kf(4, 4); 

    // 测量噪声协方差（传感器精度 ~0.1m, ~0.05rad）
    R_kf = Eigen::MatrixXd::Identity(4,4);
    R_kf(0,0) = 0.1f;   // e_y 测量方差 0.01 m²
    R_kf(1,1) = 0.1f;   // de_y
    R_kf(2,2) = 0.025f; // e_psi 测量方差 0.0025 rad²
    R_kf(3,3) = 0.005f; // de_psi

    P_kf = MatrixXd::Identity(4, 4) * 1.0;
    // 过程噪声协方差（模型不确定性小）
    Q_kf = 0.01f * Eigen::MatrixXd::Identity(4,4);

    Plant_car plant_car = Plant_car(A, B1, B2, C, D);

    //状态初始值
    Eigen::VectorXd x0(4);

    x0 << -1, 0, 0.1, 0;
    plant_car.y = plant_car.Init(x0, u);  

    // 离散化矩阵（用于 KF）
    MatrixXd A_disc = MatrixXd::Identity(4, 4) + A * dt;
    MatrixXd B1_disc = B1 * dt;
    // 被控对象的 KF 初始化（使用离散矩阵）
    plant_car.kf.init(A_disc, B1_disc, C, P_kf, Q_kf, R_kf, x0);

    return plant_car;
}

// LQR LQRInit(Plant &plant) {
//     // LQR 控制器的初始化函数
//     //LQR状态权重矩阵、输入权重矩阵和终端权重矩阵
//     MatrixXd Q(6, 6);
//     MatrixXd R(2, 2);
//     MatrixXd S(6, 6);

//     // Q = MatrixXd::Identity(6, 6) * 0.01;
//     Q = Eigen::MatrixXd::Zero(6,6); // 赋值
//     Q(0,0) = 0.1;
//     Q(3,3) = 0.1;
//     S = MatrixXd::Identity(6, 6) * 0.01;
//     R = MatrixXd::Identity(2, 2) * 0.01;

//     // //LQR控制器 
//     LQR lqr;
//     lqr.Init(plant.A, plant.B, plant.C, Q, R, S);

//     return lqr;
// }

MPC MPCInit(Plant_car &plant_car, int N) {
    // MPC控制器的初始化函数
    //MPC状态权重矩阵、输入权重矩阵和终端权重矩阵
    MatrixXd Q(4, 4);
    MatrixXd R(1, 1);
    MatrixXd S(4, 4);

    Q = Eigen::MatrixXd::Zero(4,4); // 赋值
    Q(0,0) = 100;
    Q(1,1) = 1;
    Q(2,2) = 20;
    Q(3,3) = 1;
    S = MatrixXd::Identity(4, 4) * 1;
    R = MatrixXd::Identity(1, 1) * 0.05;

    // //MPA控制器 
    MPC mpc;
    //连续变离散
    float dt=0.1f;
    MatrixXd A_disc(4, 4);
    MatrixXd B_disc(4, 1);
    A_disc = Eigen::MatrixXd::Identity(4,4) + plant_car.A * dt;   // 或者 matrixExponential
    B_disc = plant_car.B1 * dt;
    mpc.Init(A_disc, B_disc, plant_car.C, Q, R, S, N);

    return mpc;
}



// Object objectInit(float dt) {
//     // 初始化参考目标
//     MatrixXd A(4, 4);
//     MatrixXd C(4, 4);
//     //x,y,dx,dy
//     A << 1, 0, dt, 0,
//         0, 1, 0, dt, 
//         0, 0, cos(w*dt),-sin(w*dt),
//         0, 0, sin(w*dt), cos(w*dt);
//     C << 1, 0, 0, 0,
//         0, 1, 0, 0, 
//         0, 0, 1, 0,
//         0, 0, 0, 1;

//     Object object = Object(A, Eigen::MatrixXd::Zero(4,2), C , Eigen::MatrixXd::Zero(4,2));

//     //目标状态初始值
//     Eigen::VectorXd obj_x0(4);
//     obj_x0 << 0, 1, 1, 0;
//     //目标状态估计值
//     Eigen::VectorXd obj_x_hat(4);
//     //初始化目标系统
//     Eigen::VectorXd obj_y = object.Init(obj_x0);
//     Eigen::VectorXd obj_y_meas(4);// 滤波前的测量值（带噪声）
//     Eigen::VectorXd obj_y_filt(4);// 滤波后的估计值（通过观测矩阵得到位置）

//     MatrixXd obj_P_kf(4, 4);
//     MatrixXd obj_Q_kf(4, 4); 
//     MatrixXd obj_R_kf(4, 4); 
//     float sigma = 0.01f;
//     // 估计测量噪声协方差
//     obj_R_kf = 10.0f*(sigma * sigma / 3.0) * Eigen::MatrixXd::Identity(4,4);
//     obj_P_kf = MatrixXd::Identity(4, 4) * 1;
//     // 估计过程噪声协方差矩阵
//     obj_Q_kf << 1*dt*dt*dt*dt,0, 1*dt*dt*dt,0,
//         0 ,1*dt*dt*dt*dt, 0, 1*dt*dt*dt,
//         1*dt*dt*dt , 0, 1*dt*dt*dt*dt, 0,
//         0, 1*dt*dt*dt ,  0, 1*dt*dt*dt*dt;
//     object.kf.init(A, Eigen::MatrixXd::Zero(4,2), C , obj_P_kf, obj_Q_kf, obj_R_kf, obj_x0);
//     return object;
// }

// PID PIDInit(const Plant &plant) {
//     int nu = plant.B.cols();  // 控制输入维度为2

//     Eigen::VectorXd kp(nu);
//     Eigen::VectorXd ki(nu);
//     Eigen::VectorXd kd(nu);

//     kp << 3.1f, 3.1f;  // x, y 方向的P参数（系统对称，增益一致）
//     ki << 0.0f, 0.0f;     // I参数
//     kd << 5.0f, 5.0f;     // D参数

//     PID pid(nu, kp, ki, kd);
//     return pid;
// }

// 前馈控制：最小化 ||B1*δ + B2*w||² 的最小二乘解
// δ_ff = -(B1ᵀB1)⁻¹ * B1ᵀ * B2 * w = 0.472 * L*κ
// 这样前馈抵消 B2 扰动的主力道，余量由 MPC 反馈填补
float computeFeedforward(float kappa) {
    float L=lr + lf;
    return L * kappa + (lr /( L * C_af))-(lf / (L * C_ar)) * m / 2 * Vx * Vx * kappa;
}


int main() {
    
    // 打开文件用于写入
    ofstream outfile("output/output.txt");
    if (!outfile.is_open()) {
        cerr << "Failed to open output file!" << endl;
        return 1;
    }

    float dt = 0.1f;

    // // 控制量初始化
    // Eigen::VectorXd u(2);
    // Eigen::VectorXd u_fb(2);
    // Eigen::VectorXd u_ff(2);
    // u << 0.0, 0.0;

    // // 被控对象初始化
    // Plant plant = plantInit(dt, u);

    // int N = 50;
    // // 控制器初始化
    // LQR lqr = LQRInit(plant);
    // PID pid = PIDInit(plant);
    // MPC mpc = MPCInit(plant,N);

    // 控制量初始化
    Eigen::VectorXd u(1);
    Eigen::VectorXd u_fb(1);
    Eigen::VectorXd u_ff(1);
    u << 0.0;

    // 被控对象初始化
    Plant_car plant_car = plant_car_Init(dt, u);

    int N = 50;
    // 控制器初始化
    // LQR lqr = LQRInit(plant_car);
    // PID pid = PIDInit(plant_car);
    MPC mpc = MPCInit(plant_car,N);
   
    // 路径初始化：直道 + 弯道 + S弯 + 直角弯组合
    ComplexPath complex_path;
    complex_path.addStraight(50.0f);          // 长直道
    complex_path.addArc(75.4f, 12.0f);        // 完整圆形（半径12m, 360°）
    complex_path.addStraight(20.0f);
    complex_path.addArc(31.4f, 20.0f);        // 左转90°缓弯
    complex_path.addStraight(30.0f);
    complex_path.addArc(9.42f, -6.0f);       // 右转弯
    complex_path.addStraight(30.0f); 
    complex_path.addSlalom(120.0f, 8.0f, 0.1f);  // S弯
    complex_path.addStraight(30.0f);
    complex_path.addArc(9.0f, -12.0f);       // 右转直角弯
    complex_path.addStraight(30.0f);
    complex_path.addArc(37.7f, 12.0f);        // 左转180°调头弯
    complex_path.addSlalom(80.0f, 6.0f, 0.16f);   // 紧凑S弯
    complex_path.addStraight(30.0f);
    complex_path.addArc(23.6f, 15.0f);        // 左转90°
    complex_path.addStraight(30.0f);
    complex_path.addArc(15.7f, -10.0f);       // 右转直角弯
    complex_path.addStraight(30.0f);         
    complex_path.addArc(75.4f, 12.0f);        // 完整圆形（半径12m, 360°）
    complex_path.addStraight(20.0f);            // 终点直道
    complex_path.build();

    outfile << "# REF: " << complex_path.getRefString(dt, Vx) << std::endl;
    outfile << "Step\ttime\te_y\tde_y\te_psi\tde_psi\tsteer" << std::endl;
    Eigen::VectorXd target_y(4);
    target_y << 0.0, 0.0, 0.0, 0.0;

    for (int i = 0; i < 800; i++)
    {
        // 从路径获取当前曲率，计算路径角速度 w
        float s = i * Vx * dt;
        Eigen::VectorXd path_state = complex_path.getState(s);
        float kappa = path_state[3];
        float w_cur = kappa * Vx;


        // 被控对象自身状态感知的卡尔曼滤波更新
        plant_car.kf.x_post = plant_car.kf.update(plant_car.y, u);

        // MPC 反馈控制 + 前馈（路径曲率补偿）
        u_fb = mpc.predict(target_y, plant_car.kf.x_post);
        u(0) = u_fb(0) + computeFeedforward( kappa);

        // 被控对象运行（使用路径曲率对应的角速度）
        plant_car.y = plant_car.step(dt, w_cur, u);

        // debug输出
        std::cout << std::fixed << std::setprecision(2) << "step= " << i << ", "
                  << "u= " << u[0] << ", "
                  << "plant = " << plant_car.x[0] << ", " << plant_car.x[2] << ", " << std::endl;
                //   << "plant = " << plant.kf.y_post[0] << ", " << plant.kf.y_post[1] << ", " << plant.kf.y_post[2] << ", " << plant.kf.y_post[3] << ", "
                //   << "object = " << object.kf.y_post[0] << ", " << object.kf.y_post[1] << std::endl;

        outfile << i << "\t" << i * dt << "\t"
                << plant_car.x[0] << "\t" << plant_car.x[1] << "\t"
                << plant_car.x[2] << "\t" << plant_car.x[3] << "\t"
                // << object.kf.x_post[0] << "\t" << object.kf.x_post[1] << "\t"
                // << object.kf.x_post[2] << "\t" << object.kf.x_post[3] << "\t"
                << u[0] << endl;
    
    }

    outfile.close();

    return 0;
}