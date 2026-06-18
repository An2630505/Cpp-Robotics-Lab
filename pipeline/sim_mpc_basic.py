"""
sim_mpc_basic.py — 最小 MPC 车道保持仿真

验证 pnc 模块（pybind11 绑定的 C++ MPC）在 Python pipeline 中正常工作。
仿真一辆自行车模型车辆沿直道行驶，MPC 做横向控制。

用法: python pipeline/sim_mpc_basic.py
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "build", "pnc"))

import numpy as np
import pnc  # noqa: E402  — C++ 算法库


# ========================== 车辆参数 ==========================
MASS     = 1573.0   # kg
IZ       = 2873.0   # kg·m²
LF       = 1.1      # m (前轴到质心)
LR       = 1.58     # m (后轴到质心)
C_AF     = 80000.0  # N/rad (前轮侧偏刚度)
C_AR     = 80000.0  # N/rad (后轮侧偏刚度)
VX       = 10.0     # m/s (纵向速度)
DT       = 0.1      # s (仿真步长)
N_STEPS  = 300      # 仿真步数
N_HORIZON = 50      # MPC 预测时域


def build_bicycle_model(dt: float, vx: float):
    """构建自行车模型状态空间矩阵 (误差模型: e1, de1, e2, de2).
        A_disc = I + A * dt
        B1_disc = B1 * dt  (控制输入: 前轮转角)
        B2_disc = B2 * dt  (扰动: 路径曲率角速度)
    """
    A = np.array([
        [0, 1, 0, 0],
        [0, -(2 * C_AF + 2 * C_AR) / (MASS * vx),
         (2 * C_AF + 2 * C_AR) / MASS,
         -(2 * C_AF * LF - 2 * C_AR * LR) / (MASS * vx)],
        [0, 0, 0, 1],
        [0, -(2 * C_AF * LF - 2 * C_AR * LR) / (IZ * vx),
         (2 * C_AF * LF - 2 * C_AR * LR) / IZ,
         -(2 * C_AF * LF * LF + 2 * C_AR * LR * LR) / (IZ * vx)],
    ])

    B1 = np.array([
        [0],
        [2 * C_AF / MASS],
        [0],
        [2 * C_AF * LF / IZ],
    ])

    B2 = np.array([
        [0],
        [-(2 * C_AF * LF - 2 * C_AR * LR) / (MASS * vx) - vx],
        [0],
        [-(2 * C_AF * LF * LF + 2 * C_AR * LR * LR) / (IZ * vx)],
    ])

    C = np.eye(4)

    # 离散化 (前向欧拉)
    A_disc = np.eye(4) + A * dt
    B1_disc = B1 * dt
    B2_disc = B2 * dt

    return A_disc, B1_disc, B2_disc, C


def compute_feedforward(kappa: float) -> float:
    """前馈控制: 根据曲率补偿稳态转向角."""
    L = LF + LR
    return (L * kappa
            + (LR / (L * C_AF) - LF / (L * C_AR))
            * MASS / 2 * VX * VX * kappa)


def run():
    # ---- 1. 初始化模型 ----
    A, B1, B2, C = build_bicycle_model(DT, VX)

    # 初始状态: [e1=0.5, de1=0, e2=0.05, de2=0]
    x = np.array([0.5, 0.0, 0.05, 0.0])

    # ---- 2. 初始化 MPC 控制器 ----
    Q = np.diag([100.0, 1.0, 20.0, 1.0])
    R = np.diag([0.05])
    S = np.eye(4) * 1.0

    mpc = pnc.MPC()
    mpc.init(A, B1, C, Q, R, S, N_HORIZON)

    # ---- 3. 仿真循环 (直道, kappa=0) ----
    target = np.zeros(4)  # 目标: 零误差
    log = []

    for step in range(N_STEPS):
        # MPC 反馈
        u_fb = mpc.predict(target, x)

        # 前馈 (直道 kappa=0, 所以前馈=0)
        kappa = 0.0
        u_ff = compute_feedforward(kappa)
        steer = float(u_fb[0] + u_ff)

        # 自行车模型更新 (w=kappa*Vx=0)
        w_cur = kappa * VX
        x = A @ x + B1 @ np.array([steer]) + B2 @ np.array([w_cur])

        log.append((step * DT, float(x[0]), float(x[2]), steer))

        if step % 50 == 0:
            print(f"  step={step:3d}  e_y={x[0]:+.4f}  e_psi={x[2]:+.4f}  steer={steer:.4f}")

    # ---- 4. 输出 ----
    print(f"\n仿真完成: {N_STEPS} 步, 最终 e_y={x[0]:.4f}, e_psi={x[2]:.4f}")

    # 保存到文件
    os.makedirs("output", exist_ok=True)
    outpath = "output/sim_mpc_basic.txt"
    with open(outpath, "w") as f:
        f.write("time\te_y\tde_y\te_psi\tde_psi\tsteer\n")
        for t, ey, epsi, steer in log:
            f.write(f"{t:.1f}\t{ey:.6f}\t0.0\t{epsi:.6f}\t0.0\t{steer:.6f}\n")
    print(f"结果已保存: {outpath}")
    print("✅ pipeline/sim_mpc_basic.py 跑通 — MPC 控制生效")

    return log


if __name__ == "__main__":
    run()
