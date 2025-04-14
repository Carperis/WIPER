import threading

state_lock = threading.Lock()
state = {
    'x': 0.0, 'y': 0.0, 'theta': 0.0,
    'cam_x': None, 'cam_y': None, 'cam_theta': None,
    'cam_timestamp': 0.0
}