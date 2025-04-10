import numpy as np


class TVLQRController:
    def __init__(self, start, mode=1, N=50, v0=0.2, L=0.215, wheel_radius=0.062):
        self.v0 = v0
        self.L = L
        self.r = wheel_radius
        self.N = N
        self.mode = mode
        self.step = 0
        self.Q = np.diag([10, 10, 5])
        self.R = np.diag([0.1, 0.1])
        self.Qf = self.Q.copy()

        self.x_refs = generate_reference_trajectory(start, N, mode)
        theta_refs = [x[2] for x in self.x_refs]
        _, _, _, self.K_list = finite_horizon_tvlqr(theta_refs, v0, L, self.Q, self.R, self.Qf, N)

    def get_control(self, x_curr):
        if self.step >= self.N:
            self.step = self.N  # freeze at end
            return 0.0, 0.0

        x_ref = self.x_refs[self.step]
        K = self.K_list[self.step]
        u = -K @ (x_curr - x_ref)

        # Convert to RPMs
        rpm_L = (u[0] / (2 * np.pi * self.r)) * 60
        rpm_R = (u[1] / (2 * np.pi * self.r)) * 60

        self.step += 1
        return rpm_L, rpm_R


def generate_reference_trajectory(start, N, mode=1):
    """
    Generate various predefined reference trajectories based on a selected mode.

    Args:
        start: (x0, y0, theta0)
        N: number of time steps
        mode: type of trajectory
            1 - straight up (along y) for 20cm
            2 - straight right (along x) for 20cm
            3 - circular arc with 20cm radius
            4 - L-shape: 10cm up, then 10cm right

    Returns:
        List of reference states: [ [x, y, theta], ..., ]
    """
    x0, y0, theta0 = start
    x_vals, y_vals = np.zeros(N), np.zeros(N)

    if mode == 1:
        # Straight up 20 cm
        y_vals = np.linspace(y0, y0 + 0.2, N)
        x_vals = np.full(N, x0)

    elif mode == 2:
        # Straight right 20 cm
        x_vals = np.linspace(x0, x0 + 0.2, N)
        y_vals = np.full(N, y0)

    elif mode == 3:
        # Circle (quarter arc) with radius 20 cm
        r = 0.1  # 10 cm radius
        angles = np.linspace(0, np.pi / 2, N)
        x_vals = x0 + r * np.sin(angles)
        y_vals = y0 + r * (1 - np.cos(angles))

    elif mode == 4:
        # L-shape: 10cm up, then 10cm right
        split = N // 2
        y_up = np.linspace(y0, y0 + 0.1, split)
        x_up = np.full(split, x0)
        x_right = np.linspace(x0, x0 + 0.1, N - split)
        y_right = np.full(N - split, y0 + 0.1)
        x_vals = np.concatenate((x_up, x_right))
        y_vals = np.concatenate((y_up, y_right))

    # Estimate theta from path direction
    theta_vals = np.arctan2(np.gradient(y_vals), np.gradient(x_vals))

    x_refs = [np.array([x_vals[i], y_vals[i], theta_vals[i]]) for i in range(N)]
    return x_refs


def linearize_dynamics(theta, v0, L):
    """
    Returns A and B matrices for the differential-drive model linearized at given theta.
    """
    A = np.array([
        [0, 0, -v0 * np.sin(theta)],
        [0, 0,  v0 * np.cos(theta)],
        [0, 0,  0]
    ])

    B = np.array([
        [0.5 * np.cos(theta), 0.5 * np.cos(theta)],
        [0.5 * np.sin(theta), 0.5 * np.sin(theta)],
        [-1/L, 1/L]
    ])

    return A, B


def finite_horizon_tvlqr(theta_refs, v0, L, Q, R, Qf, N):
    """
    TVLQR with re-linearization at each reference theta.
    Returns time-varying gains and dynamics.
    """
    nx = 3
    nu = 2

    A_list = []
    B_list = []
    P = [np.zeros((nx, nx)) for _ in range(N + 1)]
    K = [np.zeros((nu, nx)) for _ in range(N)]

    P[N] = Qf.copy()

    # Precompute linearized A, B for each step
    for k in range(N):
        A_k, B_k = linearize_dynamics(theta_refs[k], v0, L)
        A_list.append(A_k)
        B_list.append(B_k)

    # Backward pass
    for k in reversed(range(N)):
        A = A_list[k]
        B = B_list[k]
        BT_PB = B.T @ P[k+1] @ B
        BT_PA = B.T @ P[k+1] @ A
        K[k] = np.linalg.inv(R + BT_PB) @ BT_PA
        P[k] = Q + A.T @ P[k+1] @ A - A.T @ P[k+1] @ B @ K[k]

    return A_list, B_list, P, K


def finite_horizon_tvlqr(theta_refs, v0, L, Q, R, Qf, N):
    """
    Finite-Horizon Time-Varying LQR (TVLQR) with per-step re-linearization.
    
    Args:
        theta_refs: list of heading angles [theta_0, ..., theta_N-1]
        v0: nominal forward velocity (used in linearization)
        L: wheelbase (m)
        Q, R, Qf: cost matrices
        N: number of time steps

    Returns:
        A_list: list of A matrices [A0, A1, ..., AN-1]
        B_list: list of B matrices [B0, B1, ..., BN-1]
        P_list: list of Riccati cost-to-go matrices
        K_list: list of gain matrices K_t for each time step
    """
    # Automatically determine dimensions from Q and R
    nx = Q.shape[0]
    nu = R.shape[0]

    A_list = []
    B_list = []
    P = [np.zeros((nx, nx)) for _ in range(N + 1)]
    K = [np.zeros((nu, nx)) for _ in range(N)]

    P[N] = Qf.copy()

    for k in range(N):
        A_k, B_k = linearize_dynamics(theta_refs[k], v0, L)
        A_list.append(A_k)
        B_list.append(B_k)

    for k in reversed(range(N)):
        A = A_list[k]
        B = B_list[k]
        BT_PB = B.T @ P[k+1] @ B
        BT_PA = B.T @ P[k+1] @ A
        K[k] = np.linalg.inv(R + BT_PB) @ BT_PA
        P[k] = Q + A.T @ P[k+1] @ A - A.T @ P[k+1] @ B @ K[k]

    return A_list, B_list, P, K
