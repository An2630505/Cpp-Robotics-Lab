"""
sim_lane_keeping.py — 完整车道保持仿真

复现 main.cpp 的 MPC 车道保持功能（直道 + 圆弧 + S 弯组合路径），
使用 pnc 模块：Path + BicycleModel + MPC + KF + 前馈控制。

用法: python pipeline/sim_lane_keeping.py
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "build2", "pnc"))

import numpy as np
import pnc


# ========================== 车辆参数 ==========================
MASS   = 1573.0
IZ     = 2873.0
LF     = 1.1
LR     = 1.58
C_AF   = 80000.0
C_AR   = 80000.0
VX     = 10.0
DT     = 0.1
N_STEPS = 800
N_HORIZON = 50


def init_bicycle_model():
    """构建自行车模型状态空间矩阵，初始化 KF，返回 BicycleModel 对象."""
    A = np.array([
        [0, 1, 0, 0],
        [0, -(2 * C_AF + 2 * C_AR) / (MASS * VX),
         (2 * C_AF + 2 * C_AR) / MASS,
         -(2 * C_AF * LF - 2 * C_AR * LR) / (MASS * VX)],
        [0, 0, 0, 1],
        [0, -(2 * C_AF * LF - 2 * C_AR * LR) / (IZ * VX),
         (2 * C_AF * LF - 2 * C_AR * LR) / IZ,
         -(2 * C_AF * LF * LF + 2 * C_AR * LR * LR) / (IZ * VX)],
    ])

    B1 = np.array([[0], [2 * C_AF / MASS], [0], [2 * C_AF * LF / IZ]])
    B2 = np.array([
        [0],
        [-(2 * C_AF * LF - 2 * C_AR * LR) / (MASS * VX) - VX],
        [0],
        [-(2 * C_AF * LF * LF + 2 * C_AR * LR * LR) / (IZ * VX)],
    ])
    C_mat = np.eye(4)
    D_mat = np.zeros((4, 1))

    model = pnc.BicycleModel(A, B1, B2, C_mat, D_mat)

    # KF 初始化
    P_kf = np.eye(4) * 1.0
    Q_kf = np.eye(4) * 0.01
    R_kf = np.diag([0.1, 0.1, 0.025, 0.005])

    A_disc = np.eye(4) + A * DT
    B1_disc = B1 * DT

    model.kf.init(A_disc, B1_disc, C_mat, P_kf, Q_kf, R_kf,
                  np.array([-1.0, 0.0, 0.1, 0.0]))

    # 初始状态
    model.init(np.array([-1.0, 0.0, 0.1, 0.0]))

    return model


def build_path() -> pnc.Path:
    """构建多段组合路径（与 main.cpp 相同）."""
    path = pnc.Path()
    path.add_straight(50.0)
    path.add_arc(75.4, 12.0)
    path.add_straight(20.0)
    path.add_arc(31.4, 20.0)
    path.add_straight(30.0)
    path.add_arc(9.42, -6.0)
    path.add_straight(30.0)
    path.add_slalom(120.0, 8.0, 0.1)
    path.add_straight(30.0)
    path.add_arc(9.0, -12.0)
    path.add_straight(30.0)
    path.add_arc(37.7, 12.0)
    path.add_slalom(80.0, 6.0, 0.16)
    path.add_straight(30.0)
    path.add_arc(23.6, 15.0)
    path.add_straight(30.0)
    path.add_arc(15.7, -10.0)
    path.add_straight(30.0)
    path.add_arc(75.4, 12.0)
    path.add_straight(20.0)
    path.build()
    return path


def compute_feedforward(kappa: float) -> float:
    L = LF + LR
    return (L * kappa
            + (LR / (L * C_AF) - LF / (L * C_AR))
            * MASS / 2 * VX * VX * kappa)


def run():
    print("=== 车道保持仿真 ===")

    # 1. 初始化
    model = init_bicycle_model()
    path = build_path()
    print(f"  路径总长: {path.total_length():.1f}m")

    Q = np.diag([100.0, 1.0, 20.0, 1.0])
    R = np.diag([0.05])
    S = np.eye(4) * 1.0

    A_disc = np.eye(4) + np.array([
        [0, 1, 0, 0],
        [0, -(2 * C_AF + 2 * C_AR) / (MASS * VX),
         (2 * C_AF + 2 * C_AR) / MASS,
         -(2 * C_AF * LF - 2 * C_AR * LR) / (MASS * VX)],
        [0, 0, 0, 1],
        [0, -(2 * C_AF * LF - 2 * C_AR * LR) / (IZ * VX),
         (2 * C_AF * LF - 2 * C_AR * LR) / IZ,
         -(2 * C_AF * LF * LF + 2 * C_AR * LR * LR) / (IZ * VX)],
    ]) * DT
    B1_disc = np.array([[0], [2 * C_AF / MASS], [0], [2 * C_AF * LF / IZ]]) * DT
    C_mat = np.eye(4)

    mpc = pnc.MPC()
    mpc.init(A_disc, B1_disc, C_mat, Q, R, S, N_HORIZON)

    target = np.zeros(4)
    log = []
    steer = 0.0  # 初始转向角

    # 2. 仿真循环
    for step in range(N_STEPS):
        s = step * VX * DT
        path_state = path.get_state(s)
        kappa = path_state[3]
        w_cur = kappa * VX

        # KF 更新（使用上一次的控制量）
        model.kf.update(model.y, np.array([steer]))

        # MPC 反馈 + 前馈
        u_fb = mpc.predict(target, model.kf.x_post)
        steer = float(u_fb[0] + compute_feedforward(kappa))

        # 车辆模型更新
        model.step(DT, w_cur, np.array([steer]))

        log.append((step * DT, float(model.x[0]), float(model.x[1]),
                    float(model.x[2]), float(model.x[3]), steer,
                    float(model.kf.x_post[0]), float(model.kf.x_post[2])))

        if step % 100 == 0:
            print(f"  step={step:3d}  e_y={model.x[0]:+.4f}  "
                  f"e_psi={model.x[2]:+.4f}  steer={steer:.4f}")

    # 3. 输出
    os.makedirs("output", exist_ok=True)
    outpath = "output/sim_lane_keeping.txt"
    with open(outpath, "w") as f:
        f.write("# REF: " + path.get_ref_string(DT, VX) + "\n")
        f.write("time\te_y\tde_y\te_psi\tde_psi\tsteer\t"
                "kf_e_y\tkf_e_psi\n")
        for row in log:
            f.write("\t".join(f"{v:.6f}" for v in row) + "\n")

    final_e_y = log[-1][1]
    print(f"\n仿真完成: {N_STEPS} 步, 最终 e_y={final_e_y:.4f}")
    print(f"结果已保存: {outpath}")
    print("✅ pipeline/sim_lane_keeping.py 跑通")


if __name__ == "__main__":
    run()
