import numpy as np

class TVLQRController:
    def __init__(self, start, mode=1, N=10, v0=0.2, L=0.215, wheel_radius=0.062):
        self.v0 = v0
        self.L = L
        self.r = wheel_radius
        self.N = N
        self.mode = mode
        self.step = 0
        self.Q = np.diag([10, 10, 1])
        self.R = np.diag([0.01, 0.01])
        self.Qf = np.diag([20, 20, 5])

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

        rpm_L = (u[0] / (2 * np.pi * self.r)) * 60
        rpm_R = (u[1] / (2 * np.pi * self.r)) * 60

        self.step += 1
        print(f"[TVLQR] step: {self.step}, x_curr: {x_curr}, x_ref: {x_ref}")
        print(f"         u: {u}, rpm: ({rpm_L:.1f}, {rpm_R:.1f})")
        return rpm_L, rpm_R


class RecedingTVLQRController:
    def __init__(self, full_ref_traj, v0=0.2, L=0.215, wheel_radius=0.062, N=10):
        self.v0 = v0
        self.L = L
        self.r = wheel_radius
        self.N = N
        self.step = 0
        self.Q = np.diag([10, 10, 5])
        self.R = np.diag([0.1, 0.1])
        self.Qf = np.diag([100, 100, 50])
        self.ref_traj = full_ref_traj

    def get_control(self, x_curr):
        if self.step >= len(self.ref_traj) - 1:
            return 0.0, 0.0

        horizon_end = min(self.step + self.N, len(self.ref_traj))
        x_refs = self.ref_traj[self.step:horizon_end]
        N_eff = len(x_refs)

        theta_refs = [x[2] for x in x_refs]
        _, _, _, K_list = finite_horizon_tvlqr(theta_refs, self.v0, self.L, self.Q, self.R, self.Qf, N_eff)

        x_ref = x_refs[0]
        K = K_list[0]
        u = -K @ (x_curr - x_ref)

        rpm_L = (u[0] / (2 * np.pi * self.r)) * 60
        rpm_R = (u[1] / (2 * np.pi * self.r)) * 60

        self.step += 1
        print(f"[RecedingTVLQR] step: {self.step}, x_curr: {x_curr}, x_ref: {x_ref}")
        print(f"                 u: {u}, rpm: ({rpm_L:.1f}, {rpm_R:.1f})")
        return rpm_L, rpm_R


def generate_reference_trajectory(start, N, mode=1):
    x0, y0, theta0 = start
    x_vals, y_vals = np.zeros(N), np.zeros(N)

    if mode == 1:
        y_vals = np.linspace(y0, y0 + 0.2, N)
        x_vals = np.full(N, x0)
    elif mode == 2:
        x_vals = np.linspace(x0, x0 + 0.2, N)
        y_vals = np.full(N, y0)
    elif mode == 3:
        r = 0.1
        angles = np.linspace(0, np.pi / 2, N)
        x_vals = x0 + r * np.sin(angles)
        y_vals = y0 + r * (1 - np.cos(angles))
    elif mode == 4:
        split = N // 2
        y_up = np.linspace(y0, y0 + 0.1, split)
        x_up = np.full(split, x0)
        x_right = np.linspace(x0, x0 + 0.1, N - split)
        y_right = np.full(N - split, y0 + 0.1)
        x_vals = np.concatenate((x_up, x_right))
        y_vals = np.concatenate((y_up, y_right))

    theta_vals = np.arctan2(np.gradient(y_vals), np.gradient(x_vals))
    return [np.array([x_vals[i], y_vals[i], theta_vals[i]]) for i in range(N)]


def linearize_dynamics(theta, v0, L):
    A = np.array([
        [0, 0, -v0 * np.sin(theta)],
        [0, 0,  v0 * np.cos(theta)],
        [0, 0,  0]
    ])

    B = np.array([
        [np.cos(theta), 0],
        [np.sin(theta), 0],
        [0, 1]
    ])

    return A, B


def finite_horizon_tvlqr(theta_refs, v0, L, Q, R, Qf, N):
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
        BT_PB = B.T @ P[k + 1] @ B
        BT_PA = B.T @ P[k + 1] @ A
        K[k] = np.linalg.inv(R + BT_PB + 1e-6 * np.eye(nu)) @ BT_PA
        P[k] = Q + A.T @ P[k + 1] @ A - A.T @ P[k + 1] @ B @ K[k]

        # Debug output
        print(f"Step {k} theta = {theta_refs[k]:.3f}")
        print(f"A =\n{A}")
        print(f"B =\n{B}")
        print(f"K =\n{K}\n")

    return A_list, B_list, P, K