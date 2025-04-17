import threading

state_lock = threading.Lock()
state = {
    # Main control state (used by LQR)
    'x': 0.0,
    'y': 0.0,
    'theta': 0.0,

    # Raw camera measurement
    'cam_x': None,
    'cam_y': None,
    'cam_theta': None,
    'cam_timestamp': 0.0,

    # Accumulated pose tracking from AprilTag
    'cam_pose_initialized': False,
    'cam_x_acc': 0.0,
    'cam_y_acc': 0.0,
    'cam_theta_acc': 0.0,

    'last_cam_x': None,
    'last_cam_y': None,
    'last_cam_theta': None,
}