import numpy as np
import cv2
import pyrealsense2 as rs
import threading
from pupil_apriltags import Detector

# Shared state dictionary (can also be passed externally)
state = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

class AprilTagTracker(threading.Thread):
    def __init__(self, tag_size=0.04, tag_robot=0, tag_ref=1, state_ref=None):
        super().__init__()
        self.daemon = True
        self.tag_size = tag_size
        self.tag_robot = tag_robot
        self.tag_ref = tag_ref
        self.state = state_ref if state_ref is not None else state
        self.running = True

        # Setup camera
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # Align to color stream
        self.align = rs.align(rs.stream.color)

        # Setup detector
        self.detector = Detector(
            families='tagStandard41h12',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # Camera intrinsics will be obtained on first frame
        self.camera_matrix = None
        self.dist_coeffs = np.zeros((4, 1))

    def stop(self):
        self.running = False
        self.pipeline.stop()

    def run(self):
        while self.running:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()

            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())

            # Get intrinsics once
            if self.camera_matrix is None:
                intr = color_frame.profile.as_video_stream_profile().intrinsics
                self.camera_matrix = np.array([[intr.fx, 0, intr.ppx],
                                               [0, intr.fy, intr.ppy],
                                               [0, 0, 1]])

            # Detect tags
            detections = self.detector.detect(
                color_image, estimate_tag_pose=True, camera_params=self._get_cam_params(), tag_size=self.tag_size)

            tag_dict = {d.tag_id: d for d in detections}
            if self.tag_robot not in tag_dict or self.tag_ref not in tag_dict:
                continue  # Need both tags to update pose

            # Extract tag poses
            pose_robot = tag_dict[self.tag_robot]
            pose_ref = tag_dict[self.tag_ref]

            rvec_robot = pose_robot.pose_R
            tvec_robot = pose_robot.pose_t.reshape(3, 1)

            rvec_ref = pose_ref.pose_R
            tvec_ref = pose_ref.pose_t.reshape(3, 1)

            # Invert reference tag pose
            R_ref_inv = rvec_ref.T
            t_ref_inv = -R_ref_inv @ tvec_ref

            # Transform robot pose into reference frame
            t_robot_world = R_ref_inv @ tvec_robot + t_ref_inv
            R_robot_world = R_ref_inv @ rvec_robot

            x = t_robot_world[0, 0]
            y = t_robot_world[1, 0]
            theta = np.arctan2(R_robot_world[1, 0], R_robot_world[0, 0])  # heading from rot matrix

            self.state['x'] = x
            self.state['y'] = y
            self.state['theta'] = np.degrees(theta) % 360

            print(f"[APRILTAG] x: {x:.3f}, y: {y:.3f}, θ: {self.state['theta']:.2f}")

    def _get_cam_params(self):
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        return (fx, fy, cx, cy)
    
def transform_tag_pose_to_world(rvec_robot, tvec_robot, rvec_ref, tvec_ref):
    """
    Compute the robot pose in the world frame (Tag 1) given:
    - rvec_robot, tvec_robot: robot tag pose in camera frame
    - rvec_ref, tvec_ref: reference tag pose in camera frame
    Returns (x, y, theta) in world frame
    """
    # Invert reference tag pose
    R_ref_inv = rvec_ref.T
    t_ref_inv = -R_ref_inv @ tvec_ref

    # Transform robot pose into reference frame
    t_robot_world = R_ref_inv @ tvec_robot + t_ref_inv
    R_robot_world = R_ref_inv @ rvec_robot

    x = t_robot_world[0, 0]
    y = t_robot_world[1, 0]
    theta = np.arctan2(R_robot_world[1, 0], R_robot_world[0, 0])  # heading from rotation matrix

    return x, y, np.degrees(theta) % 360
