import numpy as np

def jacobian(fun, x, eps=1e-6):
    x = np.asarray(x, dtype=float)
    f0 = fun(x)
    output_dim = f0.size
    input_dim = x.size
    J = np.zeros((output_dim, input_dim))
    for i in range(input_dim):
        dx = np.zeros_like(x)
        dx[i] = eps
        f_plus = fun(x + dx)
        f_minus = fun(x - dx)
        J[:, i] = (f_plus - f_minus) / (2 * eps)
    return J

def rk4(x, u, dt, dynamics_func):
    k1 = dt * dynamics_func(x, u)
    k2 = dt * dynamics_func(x + k1/2, u)
    k3 = dt * dynamics_func(x + k2/2, u)
    k4 = dt * dynamics_func(x + k3, u)
    return x + (k1 + 2*k2 + 2*k3 + k4) / 6

def generate_reference_trajectory(start, N, mode=1, dt=0.05):
    """
    Generates a reference trajectory using geometric path generation.
    Also calculates feedforward inputs [v, omega] from numerical derivatives.

    Parameters:
    - start: initial state [x0, y0, theta0]
    - N: number of steps
    - mode: trajectory mode (1=up, 2=right, 3=quarter circle, 4=elbow)
    - dt: timestep used to compute derivatives

    Returns:
    - trajectory: list of N states [x, y, theta]
    - inputs: list of N-1 inputs [v, omega] from numerical derivative
    """
    x0, y0, theta0 = start
    x_vals = np.zeros(N)
    y_vals = np.zeros(N)

    if mode == 1:# Up
        theta_vals = np.full(N, 0)
        y_vals = np.linspace(y0, y0 + 0.2, N)
        x_vals = np.full(N, x0)

    elif mode == 2:# Left
        x_vals = np.linspace(x0, x0 - 0.5, N)
        y_vals = np.full(N, y0)
        theta_vals = np.full(N, -1/2*np.pi)  # Keep theta constant

    elif mode == 3:# Right
        x_vals = np.linspace(x0, x0 + 0.5, N)
        y_vals = np.full(N, y0)
        theta_vals = np.full(N, 1/2*np.pi)

    elif mode == 4:
        split = N // 2
        y_up = np.linspace(y0, y0 + 0.1, split)
        x_up = np.full(split, x0)
        x_right = np.linspace(x0, x0 + 0.1, N - split)
        y_right = np.full(N - split, y0 + 0.1)
        x_vals = np.concatenate((x_up, x_right))
        y_vals = np.concatenate((y_up, y_right))

    else:
        raise ValueError("Unsupported trajectory mode.")

    # Compute heading angle (theta) from gradient
    dx = np.gradient(x_vals)
    dy = np.gradient(y_vals)
    #theta_vals = np.arctan2(dy, dx) - np.pi / 2  # Subtract 90° to make 0 = up
    print(theta_vals)
    #theta_vals = (theta_vals + np.pi) % (2 * np.pi) - np.pi  # wrap to [-pi, pi]
    
    # Create state trajectory
    trajectory = [np.array([x_vals[i], y_vals[i], theta_vals[i]]) for i in range(N)]

    # Compute time derivatives to get v and omega
    dtheta = np.gradient(theta_vals, dt)
    v_vals = np.sqrt(dx**2 + dy**2) / dt
    omega_vals = dtheta

    # Create input list (length N - 1)
    inputs = [np.array([v_vals[i], omega_vals[i]]) for i in range(N - 1)]

    inputs[-1] = np.array([0.0, 0.0])  # Stop at the final input
    return trajectory, inputs

class TVLQRController:
    def __init__(self, start, mode=1, N=50, dt=0.4):
        self.N = N
        self.dt = dt
        self.nx = 3  # state dimension: [x, y, theta]
        self.nu = 2  # input dimension: [v, omega]
        
        # Cost matrices
        self.Q = np.diag([1.0, 1.0, 10.0])
        self.Qf = 10 * self.Q
        self.R = 0.1 * np.eye(self.nu)

        # Generate trajectory and feedforward inputs
        self.reference_trajectory, self.reference_inputs = generate_reference_trajectory(
            start, N=self.N, mode=mode
        )
        print("[DEBUG] Reference Y values:", [s[1] for s in self.reference_trajectory])


        # Compute gain matrices for time-varying LQR
        self.K_list = self.compute_tvlqr_gains()
        self.step_counter = 0

    def dynamics(self, x, u):
        v, omega = u
        theta = x[2]
        return np.array([
            v * np.cos(theta),  # dx
            v * np.sin(theta),  # dy
            omega               # dtheta
        ])

    def compute_tvlqr_gains(self):
        K_list = [np.zeros((self.nu, self.nx)) for _ in range(self.N - 1)]
        P = self.Qf.copy()

        for k in range(self.N - 2, -1, -1):
            x_ref_next = self.reference_trajectory[k + 1]
            u_ref = self.reference_inputs[k]

            A_matrix = jacobian(lambda x: rk4(x, u_ref, self.dt, self.dynamics), x_ref_next)
            B_matrix = jacobian(lambda u: rk4(x_ref_next, u, self.dt, self.dynamics), u_ref)

            BT_P = B_matrix.T @ P
            gain_K = np.linalg.solve(self.R + BT_P @ B_matrix, BT_P @ A_matrix)

            K_list[k] = gain_K
            P = self.Q + A_matrix.T @ P @ (A_matrix - B_matrix @ gain_K)

            # === Add this to print gain matrix ===
            print(f"[K Matrix] Step {k}:")
            print(gain_K)

        return K_list


    def get_control(self, x_current):
        x_ref = self.reference_trajectory[self.step_counter]
        u_ref = self.reference_inputs[self.step_counter]
        Kk = self.K_list[self.step_counter]

        state_error = x_current - x_ref
        #print(f"[DEBUG] State error: {state_error}")
        state_error[2] = (state_error[2] + np.pi) % (2 * np.pi) - np.pi  # wrap θ error to [-π, π]

        control_output = u_ref - Kk @ state_error  # [v, omega]

        v, omega = control_output  # forward velocity and angular velocity

        # === Robot geometry ===
        L = 0.215  # wheelbase in meters
        r = 0.03  # wheel radius in meters

        # === Convert to wheel linear velocities and RPM ===
        #omega = 2 * omega
        v_left = v - omega * (L / 2)
        v_right = v + omega * (L / 2)
        #print(f"[DEBUG] Wheel velocities: v_left: {v_left:.3f}, v_right: {v_right:.3f}")
        rpm_m1 = (v_left / (2 * np.pi * r)) * 60
        rpm_m2 = (v_right / (2 * np.pi * r)) * 60
        #rpm_m1 = np.clip(rpm_m1, -300, 300)
        #rpm_m2 = np.clip(rpm_m2, -300, 300)
        
        print(f"[TVLQR] step {self.step_counter} | u_ref: [{u_ref[0]:.3f}, {u_ref[1]:.3f}] → v: {v:.3f}, ω: {omega:.3f}")

        return rpm_m1, rpm_m2

