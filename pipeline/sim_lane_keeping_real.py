"""
sim_lane_keeping_real.py — 基于真实赛道中心线的 MPC 车道保持仿真

管线: path1.jpg → map_parser → centerline → 闭环轨迹 → MPC + BicycleModel + KF
离散化: scipy.linalg.expm (精确矩阵指数, 稳定)

用法: python pipeline/sim_lane_keeping_real.py
"""

from __future__ import annotations

import os, sys, math
import numpy as np
from scipy.linalg import expm
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

_self_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.dirname(_self_dir)
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

# ========================== Vehicle Parameters ==========================
MASS   = 1573.0
IZ     = 2873.0
LF     = 1.1
LR     = 1.58
L_WB   = LF + LR
C_AF   = 80000.0
C_AR   = 80000.0
VX     = 10.0
DT     = 0.1
N_HORIZON = 40
LANE_WIDTH = 3.5
MAX_STEER = math.radians(30.0)   # steering limit


# =====================================================================
#  Continuous state-space matrices
# =====================================================================

def build_cont_matrices():
    """Continuous A(4,4), B1(4,1), B2(4,1)."""
    A = np.array([
        [0, 1, 0, 0],
        [0, -(2*C_AF+2*C_AR)/(MASS*VX),  (2*C_AF+2*C_AR)/MASS,
         -(2*C_AF*LF-2*C_AR*LR)/(MASS*VX)],
        [0, 0, 0, 1],
        [0, -(2*C_AF*LF-2*C_AR*LR)/(IZ*VX),  (2*C_AF*LF-2*C_AR*LR)/IZ,
         -(2*C_AF*LF*LF+2*C_AR*LR*LR)/(IZ*VX)],
    ])
    B1 = np.array([[0], [2*C_AF/MASS], [0], [2*C_AF*LF/IZ]])
    B2 = np.array([
        [0],
        [-(2*C_AF*LF-2*C_AR*LR)/(MASS*VX) - VX],
        [0],
        [-(2*C_AF*LF*LF+2*C_AR*LR*LR)/(IZ*VX)],
    ])
    return A, B1, B2


def discretize(A: np.ndarray, B: np.ndarray, dt: float,
               n_sub: int = 4) -> tuple[np.ndarray, np.ndarray]:
    """
    expm-based discretization with sub-step refinement for B_d.
    A_d = expm(A*dt)
    B_d = int_0^dt expm(A*tau) dt * B  (via numerical quadrature)
    """
    nx = A.shape[0]
    A_d = expm(A * dt)
    B_d = np.zeros_like(B)
    n_steps = n_sub * 4
    for k in range(n_steps):
        tau = (k + 0.5) / n_steps * dt
        B_d += expm(A * tau) @ B * (dt / n_steps)
    return A_d, B_d


# =====================================================================
#  Bicycle model: discrete-time linear error dynamics + kinematic pose
# =====================================================================

class BicycleModel:
    """x_{k+1} = A_d x_k + B1_d u + B2_d w  (error dynamics)
       pose updated via kinematic bicycle model."""

    def __init__(self, A_d, B1_d, B2_d):
        self.A_d = A_d
        self.B1_d = B1_d.ravel()   # (4,)
        self.B2_d = B2_d.ravel()   # (4,)
        self.x_err = np.zeros(4)
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.psi   = 0.0

    def set_pose(self, x, y, psi):
        self.x_pos, self.y_pos, self.psi = x, y, psi

    def step(self, steer: float, w_curv: float):
        """Discrete-time error dynamics + kinematic pose integration."""
        self.x_err = (self.A_d @ self.x_err
                      + self.B1_d * steer
                      + self.B2_d * w_curv)
        # kinematic bicycle pose update
        beta = math.atan(LR / L_WB * math.tan(steer))
        self.x_pos += VX * math.cos(self.psi + beta) * DT
        self.y_pos += VX * math.sin(self.psi + beta) * DT
        self.psi   += VX / L_WB * math.tan(steer) * DT


# =====================================================================
#  Kalman filter
# =====================================================================

class KalmanFilter:
    def __init__(self, A_d, B1_d):
        self.A_d = A_d
        self.B1_d = B1_d
        self.C = np.eye(4)
        self.P = np.eye(4) * 1.0
        self.Q = np.eye(4) * 0.01
        self.R = np.diag([0.1, 0.1, 0.025, 0.005])
        self.x_post = np.array([-0.3, 0.0, 0.05, 0.0])

    def update(self, y_meas, u):
        x_pred = self.A_d @ self.x_post + self.B1_d.ravel() * float(u[0])
        P_pred = self.A_d @ self.P @ self.A_d.T + self.Q
        S = self.C @ P_pred @ self.C.T + self.R
        K = P_pred @ self.C.T @ np.linalg.inv(S)
        self.x_post = x_pred + K @ (y_meas - self.C @ x_pred)
        self.P = (np.eye(4) - K @ self.C) @ P_pred


# =====================================================================
#  MPC  (closed-form unconstrained QP)
# =====================================================================

class MPC:
    """min_U  U^T H U + 2 x0^T E^T U   s.t. U = [u0; u1; ...; u_{N-1}]"""

    def __init__(self, A_d, B1_d, C, Q, R, S_term, N_horizon):
        nx, nu = A_d.shape[0], B1_d.shape[1]

        # precompute A_d^i
        A_pow = [np.eye(nx)]
        for _ in range(N_horizon):
            A_pow.append(A_d @ A_pow[-1])

        # M: N*nx x N*nu  (lower-triangular block Toeplitz)
        M = np.zeros((N_horizon * nx, N_horizon * nu))
        for i in range(N_horizon):
            for j in range(i + 1):
                M[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = C @ A_pow[i-j] @ B1_d

        # G: N*nx x nx
        G = np.zeros((N_horizon * nx, nx))
        for i in range(N_horizon):
            G[i*nx:(i+1)*nx, :] = C @ A_pow[i+1]

        Qbar = np.kron(np.eye(N_horizon), Q)
        Qbar[-nx:, -nx:] = S_term
        Rbar = np.kron(np.eye(N_horizon), R)

        self.H = M.T @ Qbar @ M + Rbar
        self.H = (self.H + self.H.T) * 0.5
        self.E = M.T @ Qbar @ G
        self.nu = nu

    def predict(self, y_ref, x_obs):
        rhs = self.E @ (x_obs - y_ref)
        try:
            L = np.linalg.cholesky(self.H)
            U = -np.linalg.solve(L.T, np.linalg.solve(L, rhs))
        except np.linalg.LinAlgError:
            U = -np.linalg.solve(self.H, rhs)
        return U[:self.nu]


def feedforward(kappa):
    """kinematic + dynamic steering feedforward."""
    L = L_WB
    return L * kappa + (LR/(L*C_AF) - LF/(L*C_AR)) * MASS/2 * VX*VX * kappa


# =====================================================================
#  Trajectory  (from centerline waypoints)
# =====================================================================

class Trajectory:
    def __init__(self, points: np.ndarray):
        self.points = points
        n = len(points)

        diffs = np.diff(points, axis=0)
        seg_lens = np.sqrt(np.sum(diffs**2, axis=1))
        self.cum_s = np.concatenate([[0.0], np.cumsum(seg_lens)])
        self.total_len = float(self.cum_s[-1])

        self.psi = np.zeros(n)
        self.psi[0] = math.atan2(points[1,1]-points[0,1], points[1,0]-points[0,0])
        self.psi[-1]= math.atan2(points[-1,1]-points[-2,1], points[-1,0]-points[-2,0])
        for i in range(1, n-1):
            self.psi[i] = math.atan2(points[i+1,1]-points[i-1,1],
                                      points[i+1,0]-points[i-1,0])

        self.kappa = np.zeros(n)
        for i in range(1, n-1):
            dx  = points[i+1,0] - points[i-1,0]
            dy  = points[i+1,1] - points[i-1,1]
            ddx = points[i+1,0] - 2*points[i,0] + points[i-1,0]
            ddy = points[i+1,1] - 2*points[i,1] + points[i-1,1]
            den = (dx*dx + dy*dy)**1.5
            self.kappa[i] = (dx*ddy - dy*ddx) / den if den > 1e-8 else 0.0

        # smooth kappa a bit to reduce noise
        from scipy.ndimage import uniform_filter1d
        self.kappa = uniform_filter1d(self.kappa, size=5, mode='wrap')

    def get_state(self, s: float) -> np.ndarray:
        """[x, y, psi, kappa] at arc-length s."""
        s_mod = s % self.total_len if self.total_len > 0 else s
        idx = min(np.searchsorted(self.cum_s, s_mod), len(self.cum_s)-1)
        if idx == 0:
            return np.array([self.points[0,0], self.points[0,1],
                             self.psi[0], self.kappa[0]])
        s0, s1 = self.cum_s[idx-1], self.cum_s[idx]
        t = (s_mod - s0) / (s1 - s0) if s1 > s0 else 0.0
        x = (1-t)*self.points[idx-1,0] + t*self.points[idx,0]
        y = (1-t)*self.points[idx-1,1] + t*self.points[idx,1]
        d = self.psi[idx] - self.psi[idx-1]
        d = (d + math.pi) % (2*math.pi) - math.pi
        psi = self.psi[idx-1] + t * d
        k = (1-t)*self.kappa[idx-1] + t*self.kappa[idx]
        return np.array([x, y, psi, k])

    def project(self, cx, cy):
        """project vehicle pose onto reference: returns (e_y, e_psi)."""
        di = np.sum((self.points - np.array([cx, cy]))**2, axis=1)
        idx = int(np.argmin(di))
        px, py = self.points[idx, 0], self.points[idx, 1]
        psi_r = float(self.psi[idx])
        e_y = (cx-px)*(-math.sin(psi_r)) + (cy-py)*math.cos(psi_r)
        e_psi = 0.0  # caller fills in from vehicle heading
        return e_y, psi_r


# =====================================================================
#  Generic "go-straight" circuit assembly
# =====================================================================

def _normalize_angle(a: float) -> float:
    """Normalize angle to [-pi, pi] using atan2 — robust near ±pi boundaries."""
    return float(math.atan2(math.sin(a), math.cos(a)))


def _compute_edge_terminals(edge: dict) -> dict:
    """
    Add terminal fields to an edge dict (mutates in place).

    Terminal A = pts[0] end  (side nearest to from-node in graph)
    Terminal B = pts[-1] end (side nearest to to-node in graph)

    Each terminal stores:
      pos:     physical (x, y) coordinates
      out_psi: tangent direction LEAVING this terminal (pointing along the edge)
      in_psi:  tangent direction ARRIVING at this terminal (pointing into terminal)
    """
    pts = np.array(edge["points"])
    N = len(pts)
    s = max(1, min(5, N // 10, N - 2))

    # --- Terminal A (pts[0]) ---
    v_out = pts[s] - pts[0]
    psi_out = float(math.atan2(v_out[1], v_out[0]))
    edge["A_pos"] = pts[0].copy()
    edge["A_out"] = psi_out
    edge["A_in"] = _normalize_angle(psi_out + math.pi)

    # --- Terminal B (pts[-1]) ---
    v_in = pts[-1] - pts[max(0, N - 1 - s)]
    psi_in = float(math.atan2(v_in[1], v_in[0]))
    edge["B_pos"] = pts[-1].copy()
    edge["B_in"] = psi_in
    edge["B_out"] = _normalize_angle(psi_in + math.pi)
    return edge


def _angle_diff(a: float, b: float) -> float:
    """Shortest angle difference in radians, [0, pi]."""
    return abs(_normalize_angle(a - b))


def _adaptive_connect_threshold(edges: list[dict]) -> float:
    """
    Compute a reasonable connection threshold from edge data.

    Collects all terminal-to-terminal distances, uses 3x the median of the
    closest half (those likely within junctions), clamped to [3m, 12m].
    """
    all_positions = []
    for e in edges:
        all_positions.append(e["A_pos"])
        all_positions.append(e["B_pos"])
    all_positions = np.array(all_positions)
    diffs = np.linalg.norm(all_positions[:, None, :] - all_positions[None, :, :], axis=-1)
    triu = diffs[np.triu_indices(len(all_positions), k=1)]
    if len(triu) == 0:
        return 5.0
    # Use the median of the closest third — these are the "same-junction" distances
    n_close = max(2, len(triu) // 3)
    closest = np.sort(triu)[:n_close]
    return max(3.0, min(12.0, 2.5 * float(np.median(closest))))


def assemble_go_straight_circuit(graph: dict) -> np.ndarray:
    """
    Generic "go-straight" algorithm: walk all centerline edges,
    at each physical junction picking the unvisited edge whose departure
    direction best continues the current heading ("straight through").

    Works for any graph topology (self-loops, multi-node graphs, etc.)
    because adjacency is determined by physical endpoint proximity,
    not graph node IDs.

    Returns concatenated loop points in order.
    """
    edges = [e.copy() for e in graph["edges"]]
    for e in edges:
        _compute_edge_terminals(e)

    n_total = len(edges)
    if n_total == 0:
        raise ValueError("Centerline graph has no edges")
    if n_total == 1:
        return np.array(edges[0]["points"])

    connect_thresh = _adaptive_connect_threshold(edges)

    visited: set[int] = set()
    segments: list[np.ndarray] = []
    path_labels: list[str] = []

    # Start from edge 0, forward
    cur_e = edges[0]
    visited.add(cur_e["id"])
    segments.append(np.array(cur_e["points"]))
    path_labels.append(f"e{cur_e['id']}")
    cur_pos = cur_e["B_pos"].copy()
    arr_psi = cur_e["B_in"]

    while len(visited) < n_total:
        # Collect unvisited candidates whose terminal is near current position
        candidates: list[tuple[float, dict, str]] = []  # (angle_diff, edge, trav)
        for cand in edges:
            if cand["id"] in visited:
                continue

            dA = float(np.linalg.norm(cand["A_pos"] - cur_pos))
            dB = float(np.linalg.norm(cand["B_pos"] - cur_pos))

            if dA < connect_thresh:
                diff = _angle_diff(cand["A_out"], arr_psi)
                candidates.append((diff, cand, "fw"))
            if dB < connect_thresh:
                diff = _angle_diff(cand["B_out"], arr_psi)
                candidates.append((diff, cand, "rv"))

        if not candidates:
            # Relax: pick nearest unvisited edge endpoint
            candidates_raw: list[tuple[float, dict, str]] = []
            for cand in edges:
                if cand["id"] in visited:
                    continue
                dA = float(np.linalg.norm(cand["A_pos"] - cur_pos))
                dB = float(np.linalg.norm(cand["B_pos"] - cur_pos))
                if dA <= dB:
                    diff = _angle_diff(cand["A_out"], arr_psi)
                    candidates_raw.append((diff, cand, "fw"))
                else:
                    diff = _angle_diff(cand["B_out"], arr_psi)
                    candidates_raw.append((diff, cand, "rv"))
            candidates_raw.sort(key=lambda x: x[0])
            candidates = candidates_raw

        # Pick best (smallest angle difference = "straight through")
        candidates.sort(key=lambda x: x[0])
        best_diff, best_edge, trav = candidates[0]

        # Orient and append
        pts = np.array(best_edge["points"])
        if trav == "rv":
            pts = pts[::-1]
            path_labels.append(f"e{best_edge['id']}(rev)")
        else:
            path_labels.append(f"e{best_edge['id']}")

        segments.append(pts[1:])
        visited.add(best_edge["id"])

        # Update state
        if trav == "fw":
            cur_pos = pts[-1].copy()
            arr_psi = best_edge["B_in"]
        else:
            cur_pos = pts[-1].copy()
            arr_psi = best_edge["A_in"]

    loop = np.concatenate(segments)
    total_m = float(np.sum(np.linalg.norm(np.diff(loop, axis=0), axis=1)))
    print(f"  Circuit: {' -> '.join(path_labels)}")
    print(f"          {len(loop)} pts, {total_m:.0f}m, "
          f"{n_total} edges, connect_thresh={connect_thresh:.1f}m")
    return loop


# =====================================================================
#  Main simulation
# =====================================================================

def run():
    print("=" * 60)
    print("  Lane Keeping on Real Track — MPC + Bicycle Model")
    print("=" * 60)

    from pipeline.map_parser import parse_map
    from pipeline.centerline import extract_centerline_graph

    # Step 1: parse boundaries
    img = os.path.join(_self_dir, "map_parser", "path1.jpg")
    print(f"\n[1/4] Parsing: {img}")
    bounds = parse_map(img, pixels_per_meter=12.8, smoothing_factor=0.0,
                       num_control_points=200, resample_spacing_m=0.1)
    outer = np.array(bounds["outer_boundary"])
    holes = [np.array(h) for h in bounds["holes"]]
    print(f"  Outer: {len(outer)} pts, Holes: {len(holes)}")

    # Step 2: centerline
    print("[2/4] Extracting centerline ...")
    graph = extract_centerline_graph(bounds["outer_boundary"], bounds["holes"],
                                     pixels_per_meter=12.8, smoothing_factor=0.02,
                                     resample_spacing_m=0.2)
    print(f"  {len(graph['nodes'])} nodes, {len(graph['edges'])} edges")

    # Step 3: trajectory
    print("[3/4] Building reference trajectory ...")
    loop_pts = assemble_go_straight_circuit(graph)
    traj = Trajectory(loop_pts)
    print(f"  Length: {traj.total_len:.1f} m, {len(traj.points)} waypoints")

    # Step 4: discretize + init controllers
    A_c, B1_c, B2_c = build_cont_matrices()
    A_d, B1_d_mat = discretize(A_c, B1_c, DT)
    _,  B2_d_mat = discretize(A_c, B2_c, DT)
    B1_d = B1_d_mat
    B2_d = B2_d_mat

    # verify stability
    eig = np.abs(np.linalg.eigvals(A_d))
    print(f"  |eig(A_d)| = {[f'{e:.4f}' for e in sorted(eig, reverse=True)]}")

    model = BicycleModel(A_d, B1_d, B2_d)
    kf = KalmanFilter(A_d, B1_d)

    # initial condition: small lateral offset
    ref0 = traj.get_state(0.0)
    model.set_pose(
        ref0[0] - (-0.3)*math.sin(ref0[2]),
        ref0[1] + (-0.3)*math.cos(ref0[2]),
        ref0[2] + 0.05
    )
    model.x_err = np.array([-0.3, 0.0, 0.05, 0.0])
    kf.x_post = model.x_err.copy()

    Q = np.diag([80.0, 0.5, 15.0, 0.5])
    R = np.array([[0.1]])
    S_term = np.eye(4) * 1.0
    mpc = MPC(A_d, B1_d, np.eye(4), Q, R, S_term, N_HORIZON)

    target = np.zeros(4)
    N_STEPS = min(int(traj.total_len / (VX * DT)), 2500)

    print(f"\n[4/4] Simulating {N_STEPS} steps ({N_STEPS*DT:.1f}s) ...")
    log = []
    steer = 0.0

    for step in range(N_STEPS):
        t = step * DT
        s_travel = t * VX
        ref = traj.get_state(s_travel)
        kappa_cur = float(ref[3])

        # projection onto reference for error measurement
        e_y_proj, psi_r = traj.project(model.x_pos, model.y_pos)
        e_psi_proj = model.psi - psi_r
        e_psi_proj = (e_psi_proj + math.pi) % (2*math.pi) - math.pi

        # feed measurements into error state
        model.x_err[0] = e_y_proj
        model.x_err[2] = e_psi_proj

        # KF update
        kf.update(model.x_err.copy(), np.array([steer]))

        # MPC feedback + feedforward
        u_fb = mpc.predict(target, kf.x_post)
        steer_ff = feedforward(kappa_cur)
        steer = float(u_fb[0] + steer_ff)
        steer = max(-MAX_STEER, min(MAX_STEER, steer))

        # vehicle step
        model.step(steer, kappa_cur * VX)

        log.append((step, t, float(model.x_err[0]), float(model.x_err[1]),
                    float(model.x_err[2]), float(model.x_err[3]), steer))

        if step % 200 == 0 or step == N_STEPS-1:
            print(f"  {step:4d}  e_y={model.x_err[0]:+.4f}m  "
                  f"e_psi={math.degrees(model.x_err[2]):+.2f}deg  "
                  f"str={math.degrees(steer):+.1f}deg")

    # ---- save ----
    os.makedirs("output", exist_ok=True)
    out_txt = "output/sim_lane_keeping_real.txt"
    with open(out_txt, "w") as f:
        f.write(f"# REF: type=real_centerline len={traj.total_len:.1f}m "
                f"dt={DT} Vx={VX}\n")
        f.write("Step\ttime\te_y\tde_y\te_psi\tde_psi\tsteer\n")
        for row in log:
            f.write(f"{int(row[0])}\t" +
                    "\t".join(f"{v:.6f}" for v in row[1:]) + "\n")

    final = log[-1]
    print(f"\n[OK] {out_txt}")
    print(f"  Final: e_y={final[2]:.4f}m  e_psi={math.degrees(final[4]):.2f}deg")

    # ---- visualize ----
    visualize(log, traj, outer, holes)


# =====================================================================
#  Visualization
# =====================================================================

def visualize(log, traj, outer, holes):
    arr = np.array(log)
    N = len(arr)
    ts = arr[:, 1]
    ey = arr[:, 2]
    ep = arr[:, 4]
    st = arr[:, 6]

    # reconstruct reference and vehicle positions
    x_ref = np.zeros(N); y_ref = np.zeros(N); psi_ref = np.zeros(N)
    car_x = np.zeros(N); car_y = np.zeros(N)
    for i in range(N):
        rs = traj.get_state(i * VX * DT)
        x_ref[i], y_ref[i], psi_ref[i] = rs[0], rs[1], rs[2]
        car_x[i] = x_ref[i] - ey[i] * math.sin(psi_ref[i])
        car_y[i] = y_ref[i] + ey[i] * math.cos(psi_ref[i])

    # lane boundaries
    llx = x_ref - LANE_WIDTH/2 * np.sin(psi_ref)
    lly = y_ref + LANE_WIDTH/2 * np.cos(psi_ref)
    lrx = x_ref + LANE_WIDTH/2 * np.sin(psi_ref)
    lry = y_ref - LANE_WIDTH/2 * np.cos(psi_ref)

    fig = plt.figure(figsize=(20, 14))
    fig.suptitle("MPC Lane Keeping on Real Track (path1.jpg)", fontsize=16)

    # --- top-down map ---
    ax = fig.add_subplot(2, 3, (1, 4))
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_title("Track, Centerline & Vehicle Trajectory")

    ax.plot(outer[:,0], outer[:,1], "k-", lw=1.5, alpha=0.5, label="Track boundary")
    for i, h in enumerate(holes):
        ax.fill(h[:,0], h[:,1], fc="white", ec="k", lw=0.8, alpha=0.95,
                label=f"Hole {i}" if i==0 else "")
    ax.plot(traj.points[:,0], traj.points[:,1], "b--", lw=1.0, alpha=0.5,
            label="Centerline")
    ax.plot(llx, lly, "gray", lw=0.5, ls=":", alpha=0.5, label="Lane bounds")
    ax.plot(lrx, lry, "gray", lw=0.5, ls=":", alpha=0.5)

    skip = max(1, N // 500)
    ax.plot(car_x[::skip], car_y[::skip], "r-", lw=1.5, alpha=0.85,
            label="Vehicle path")
    ax.plot(car_x[0], car_y[0], "go", ms=10, label="Start")
    ax.plot(car_x[-1], car_y[-1], "mo", ms=10, label="End")
    for i in range(0, N, max(1, N//6)):
        ax.plot(car_x[i], car_y[i], "r.", ms=6)
        hdg = psi_ref[i] + ep[i]
        ax.arrow(car_x[i], car_y[i], math.cos(hdg)*2, math.sin(hdg)*2,
                 head_width=1.0, fc="r", ec="r", alpha=0.5)
    ax.legend(loc="upper right", fontsize=8)

    # --- curvature ---
    ax = fig.add_subplot(2, 3, 2)
    kr = np.array([traj.get_state(i*VX*DT)[3] for i in range(N)])
    ax.plot(ts, kr, "b-", lw=1.2, alpha=0.7, label="Ref kappa")
    ax.plot(ts, np.degrees([feedforward(k) for k in kr]),
            "orange", lw=1.0, alpha=0.7, label="FF steer (deg)")
    ax.set_title("Curvature & Feedforward")
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # --- lateral error ---
    ax = fig.add_subplot(2, 3, 3)
    ax.plot(ts, ey, "b-", lw=1.5)
    ax.axhline(0, color="k", ls="--", lw=0.5)
    ax.set_ylabel("e_y (m)")
    ax.set_title("Lateral Error")
    ax.grid(True, alpha=0.3)

    # --- heading error ---
    ax = fig.add_subplot(2, 3, 5)
    ax.plot(ts, np.degrees(ep), "r-", lw=1.5)
    ax.axhline(0, color="k", ls="--", lw=0.5)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("e_psi (deg)")
    ax.set_title("Heading Error")
    ax.grid(True, alpha=0.3)

    # --- steering ---
    ax = fig.add_subplot(2, 3, 6)
    ax.plot(ts, np.degrees(st), "g-", lw=1.5)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Steer (deg)")
    ax.set_title("Steering Angle")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    # stats
    skip_warmup = max(1, N//10)
    ey_rms = float(np.sqrt(np.mean(ey[skip_warmup:]**2)))
    ep_rms = float(np.sqrt(np.mean(ep[skip_warmup:]**2)))
    print(f"  RMS (after warmup): e_y={ey_rms:.4f}m  e_psi={math.degrees(ep_rms):.2f}deg")

    out_png = "output/sim_lane_keeping_real.png"
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"  Plot: {out_png}")
    plt.show()


if __name__ == "__main__":
    run()
